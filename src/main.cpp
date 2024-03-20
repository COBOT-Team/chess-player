#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "chess_player/chess_player_node.hpp"

using namespace std;
using chess_msgs::action::FindBestMove;
using libchess::Side;
using namespace std::chrono_literals;

/**
 * The result of an operation.
 */
enum class Result {
  OK,         // The operation was successful.
  ERR_RETRY,  // The operation failed, but can be retried.
  ERR_FATAL,  // The operation failed and cannot be retried.
};

/**
 * Spin the node for a given duration. This function will always wait at least the given duration,
 * but may wait longer if there are callbacks to process.
 *
 * @param[in] node The node to spin.
 * @param[in] duration The duration to spin for.
 */
void spin_for(rclcpp::Node::SharedPtr node, chrono::milliseconds duration)
{
  const auto start = chrono::steady_clock::now();
  while (chrono::steady_clock::now() - start < duration) {
    rclcpp::spin_some(node);
  }
}

/**
 * Query the action server for the best move.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result find_best_move(ChessPlayerNode& chess_player)
{
  // Create a goal for the action server.
  const FindBestMove::Goal goal = [&chess_player] {
    FindBestMove::Goal goal;
    goal.fen.fen = chess_player.get_position().get_fen();
    goal.time.black_time_left = chess_player.get_time_left(Side::Black);
    goal.time.white_time_left = chess_player.get_time_left(Side::White);
    goal.analysis_mode = false;
    return goal;
  }();

  // Send the goal to the action server and get a handle.
  const auto goal_handle = [&chess_player, &goal] {
    const auto future = chess_player.find_best_move_client->async_send_goal(goal);
    const auto result = rclcpp::spin_until_future_complete(chess_player.node, future);
    switch (result) {
      case rclcpp::FutureReturnCode::SUCCESS:
        return future.get();
      case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_ERROR(chess_player.get_logger(),
                     "Interrupted while waiting for action server goal handle");
        return rclcpp_action::ClientGoalHandle<FindBestMove>::SharedPtr();
      case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_ERROR(chess_player.get_logger(),
                     "Timeout while waiting for action server goal handle");
        return rclcpp_action::ClientGoalHandle<FindBestMove>::SharedPtr();
      default:
        return rclcpp_action::ClientGoalHandle<FindBestMove>::SharedPtr();  // Unreachable.
    }
  }();
  if (!goal_handle) return Result::ERR_RETRY;

  // Get the result of the action server.
  static const auto err_result = [] {
    rclcpp_action::ClientGoalHandle<FindBestMove>::WrappedResult wrapped_result;
    wrapped_result.code = rclcpp_action::ResultCode::ABORTED;
    return wrapped_result;
  }();
  const auto goal_result = [&chess_player, &goal_handle] {
    try {
      const auto future = chess_player.find_best_move_client->async_get_result(goal_handle);
      const auto result = rclcpp::spin_until_future_complete(chess_player.node, future);
      switch (result) {
        case rclcpp::FutureReturnCode::SUCCESS:
          return future.get();
        case rclcpp::FutureReturnCode::INTERRUPTED:
          RCLCPP_ERROR(chess_player.get_logger(),
                       "Interrupted while waiting for action server result");
          return err_result;
        case rclcpp::FutureReturnCode::TIMEOUT:
          RCLCPP_ERROR(chess_player.get_logger(), "Timeout while waiting for action server result");
          return err_result;
        default:
          return err_result;  // Unreachable.
      }
    } catch (const exception* e) {
      RCLCPP_ERROR(chess_player.get_logger(), "Cannot get action result: %s", e->what());
      return err_result;
    }
  }();
  if (goal_result.code != rclcpp_action::ResultCode::SUCCEEDED) return Result::ERR_RETRY;

  // Get the best move from the result.
  const auto move = goal_result.result->move;
  RCLCPP_INFO(chess_player.get_logger(), "Found best move: %s",
              move.draw   ? "draw" :
              move.resign ? "resign" :
                            move.move.c_str());
  chess_player.best_move = move;

  return Result::OK;
}

/**
 * This function is run in a loop, forever (until the node exits). It spins the node until a turn
 * should be made, then makes the turn.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return Whether the node should continue running or not.
 */
bool loop_fn(ChessPlayerNode& chess_player)
{
  // Wait for the node to be ready to take a turn.
  while (chess_player.get_state() != ChessPlayerNode::State::WAITING_FOR_OPTIMAL_MOVE) {
    rclcpp::spin_some(chess_player.node);
  }

  // Take the node's mutex. This ensures that the node cannot make more than one move at a time. It
  // will be released when the function returns.
  unique_lock<mutex> lock(chess_player.make_move_mutex, try_to_lock);
  if (!lock.owns_lock()) {
    RCLCPP_WARN(chess_player.get_logger(), "Attempted to make move while already making move");
    return true;
  }

  // Find the best move.
  while (true) {
    switch (find_best_move(chess_player)) {
      case Result::OK:
        break;
      case Result::ERR_RETRY:
        RCLCPP_WARN(chess_player.get_logger(), "Failed to find best move, retrying in 100ms");
        spin_for(chess_player.node, 100ms);
        continue;
      case Result::ERR_FATAL:
        RCLCPP_ERROR(chess_player.get_logger(), "Failed to find best move, exiting");
        return false;
    }
  }
  chess_player.set_state(ChessPlayerNode::State::FOUND_OPTIMAL_MOVE);

  // TODO: Interpret move and make it.

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ChessPlayerNode chess_player;

  while (rclcpp::ok()) {
    if (!loop_fn(chess_player)) break;
  }

  rclcpp::shutdown();
  return 0;
}
