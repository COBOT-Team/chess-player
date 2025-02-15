#include <behaviortree_cpp_v3/behavior_tree.h>
#include "chess_player/bt_nodes.hpp"
#include "chess_player/bt_tree.xml"
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "chess_player/chess_player_node.hpp"

using namespace std;
using chess_msgs::action::FindBestMove;
using libchess::Side;
using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;

Result ChessPlayerNode::find_best_move_()
{
  // Create a goal for the action server.
  const FindBestMove::Goal goal = [this] {
    FindBestMove::Goal goal;
    goal.fen.fen = get_position().get_fen();
    goal.time.black_time_left = get_time_left(Side::Black);
    goal.time.white_time_left = get_time_left(Side::White);
    goal.analysis_mode = false;
    return goal;
  }();

  // Send the goal to the action server and get a handle.
  const auto goal_handle = [this, &goal] {
    const auto future = find_best_move_client->async_send_goal(goal);
    return future.get();
  }();
  if (!goal_handle) return Result::ERR_RETRY;
  RCLCPP_INFO(get_logger(), "Got goal handle");

  // Get the result of the action server.
  static const auto err_result = [] {
    rclcpp_action::ClientGoalHandle<FindBestMove>::WrappedResult wrapped_result;
    wrapped_result.code = rclcpp_action::ResultCode::ABORTED;
    return wrapped_result;
  }();
  const auto goal_result = [this, &goal_handle] {
    try {
      const auto future = find_best_move_client->async_get_result(goal_handle);
      return future.get();
    } catch (const exception* e) {
      RCLCPP_ERROR(get_logger(), "Cannot get action result: %s", e->what());
      return err_result;
    }
  }();
  if (goal_result.code != rclcpp_action::ResultCode::SUCCEEDED) return Result::ERR_RETRY;

  // Get the best move from the result.
  const auto move = goal_result.result->move;
  RCLCPP_INFO(get_logger(), "Found best move: %s",
              move.draw   ? "draw" :
              move.resign ? "resign" :
                            move.move.c_str());
  best_move = move;

  this_thread::sleep_for(500ms);

  return Result::OK;
}

class ChessPlayerNode : public rclcpp:Node{
    public:
        ChessPlayerNode() : Node("chess_player_node"){
            init_behavior_tree_();
        }

    private: 
        BT::Tree behavior_tree_; 

        void init_behavior_tree_(){
            using namespace BT; 

            BehaviorTreeFactory factory; 

            ChessPlayerNode* chess_node = new ChessPlayerNode("chess_player_node");  
            auto turn_setup_action = std::make_shared<TurnSetupAction>("TurnSetup", BT::NodeConfiguration{}, chess_node);

            factory.registerNodeType<TurnSetupAction>("TurnSetup");
            factory.registerNodeType<FindBestMoveAction>("FindBestMove");
            factory.registerNodeType<ParseMoveAction>("ParseMove");
            factory.registerNodeType<CapturePieceAction>("CapturePiece");
            factory.registerNodeType<MovePieceAction>("MovePiece");
            factory.registerNodeType<HitClockAction>("HitClock");
            factory.registerNodeType<MoveHomeAction>("MoveHome");

            //making a change here: verifying that the tree loads correctly to start
            try{
            auto tree = factory.createTreeFromFile("bt_tree.xml"); 
            behavior_tree_ = std::move(tree); 
            RCLCPP_INFO(this->get_logger(), "behavior tree initialized successful"); 
            } 
            catch(const std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "failed to initialize behavior tree %s", e.what()); 
            }
        }

        void turn_setup(){
            // Take the node's mutex. This ensures that the node cannot make more than one move at a time. It
            // will be released when the function returns.
            unique_lock<mutex> lock(make_move_mutex, try_to_lock);
            if (!lock.owns_lock()) {
                RCLCPP_WARN(get_logger(), "Attempted to make move while already making move");
                return true;
            }

            #if 0

            const auto pose = [&]{
                tf2::Quaternion q;
                q.setRPY(0, M_PI, -M_PI / 6);

                Pose p;
                // p.position.x = -0.01 + 0.0;
                p.position.x = 0.0;
                // p.position.y = 0.0125 - 0.15;
                p.position.y = 0.0;
                p.position.z = 245.0/1000.0;
                p.orientation = tf2::toMsg(q);

                return p;
            }();
            main_move_group->setPoseTarget(pose);
            const auto [success, plan] = [&] {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                const auto ok = static_cast<bool>(main_move_group->plan(plan));
                return make_pair(ok, plan);
            }();
            if (!success) {
                RCLCPP_ERROR(node->get_logger(), "Failed to plan motion");
                return true;
            }

            RCLCPP_INFO(get_logger(), "EXECUTING");
            const auto execute_result = main_move_group->execute(plan);
            switch (execute_result.val) {
                case moveit::core::MoveItErrorCode::SUCCESS:
                case moveit::core::MoveItErrorCode::TIMED_OUT:
                return true;
                default:
                RCLCPP_ERROR(node->get_logger(), "Failed to execute motion");
                return true;
            }

            return true;

            #endif

            RCLCPP_INFO(this->get_logger(), "starting move"); 
        }

        void take_turn_() {
            // Tick the root node of the Behavior Tree
            BT::NodeStatus status = behavior_tree_.tickRoot();
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Turn completed successfully.");
                RCLCPP_INFO(this->get_logger(), "Current FEN: %s", get_position().get_fen().c_str());//log current game staate
            } else if{
                RCLCPP_ERROR(this->get_logger(), "Turn failed.");
            }
        }

        void _find_best_move_(){
              for (auto result = find_best_move_(); result != Result::OK; result = find_best_move_()) {
                    switch (result) {
                    case Result::OK:
                        break;
                    case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to find best move, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                    case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to find best move, exiting");
                        return false;
                    }
                }
                set_state(State::FOUND_OPTIMAL_MOVE);
        }

        void parse_move_(const std::string& move){
                // Parse the move.
                if (best_move.draw) {
                    RCLCPP_INFO(get_logger(), "Draw, not making a move");
                    return true;
                }
                if (best_move.resign) {
                    RCLCPP_INFO(get_logger(), "Resign, not making a move");
                    return true;
                }
                if (best_move.move.empty()) {
                    RCLCPP_WARN(get_logger(), "No move found, not making a move");
                    return true;
                }
                libchess::Move move;
                try {
                    move = get_position().parse_move(best_move.move);
                } catch (const exception* e) {
                    RCLCPP_ERROR(get_logger(), "Cannot parse chess move, not making a move");
                    return true;
                }
             }

            void capture_piece_(const std::string& move){
                // If we need to capture a piece, do so.
                if (move.is_capturing()) {
                    set_state(ChessPlayerNode::State::TAKING_PIECE);
                    const auto capture_result = [this, &move] {
                    while (1) {
                        const auto result = capture_at_(move.to());
                        switch (result) {
                        case Result::OK:
                            return true;
                        case Result::ERR_RETRY:
                            RCLCPP_WARN(get_logger(), "Failed to capture piece, retrying in 100ms");
                            this_thread::sleep_for(100ms);
                            continue;
                        case Result::ERR_FATAL:
                            RCLCPP_ERROR(get_logger(), "Failed to capture piece, exiting");
                            return false;
                        }
                    }
                    }();
                    if (!capture_result) return false;
                }
            }

            void move_piece_(const std::string& move){
                // Move the piece.
                set_state(ChessPlayerNode::State::MOVING_PIECE);
                const auto move_result = [this, &move] {
                    while (1) {
                    const auto result = move_piece_(move);
                    switch (result) {
                        case Result::OK:
                        return true;
                        case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to move piece, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                        case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to move piece, exiting");
                        return false;
                        }
                    }
                }();
                if (!move_result) return false;

                 RCLCPP_INFO(this->get_logger(), "Executing move: %s", move.c_str());
        }

        void hit_clock_(){
                // Hit the clock.
                set_state(ChessPlayerNode::State::HITTING_CLOCK);
                const auto hit_clock_result = [this] {
                    while (1) {
                    const auto result = hit_clock_();
                    switch (result) {
                        case Result::OK:
                        return true;
                        case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to hit clock, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                        case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to hit clock, exiting");
                        return false;
                    }
                    }
                }();
                if (!hit_clock_result) return false; 

                RCLCPP_INFO(this->get_logger(), "Hitting the clock.");
        }

        void move_home_(){
                // Move to home.
                set_state(ChessPlayerNode::State::MOVING_TO_HOME);
                const auto move_home_result = [this] {
                    while (1) {
                    const auto result = move_home_();
                    switch (result) {
                        case Result::OK:
                        return true;
                        case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to move to home, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                        case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to move to home, exiting");
                        return false;
                    }
                    }
                }();
                if (!move_home_result) return false;

                // Move to home.
                set_state(ChessPlayerNode::State::MOVING_TO_HOME);
                const auto move_home_result1 = [this] {
                    while (1) {
                    const auto result = move_home_();
                    switch (result) {
                        case Result::OK:
                        return true;
                        case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to move to home, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                        case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to move to home, exiting");
                        return false;
                    }
                    }
                }();
                if (!move_home_result1) return false;

                // Move to home.
                set_state(ChessPlayerNode::State::MOVING_TO_HOME);
                const auto move_home_result2 = [this] {
                    while (1) {
                    const auto result = move_home_();
                    switch (result) {
                        case Result::OK:
                        return true;
                        case Result::ERR_RETRY:
                        RCLCPP_WARN(get_logger(), "Failed to move to home, retrying in 100ms");
                        this_thread::sleep_for(100ms);
                        continue;
                        case Result::ERR_FATAL:
                        RCLCPP_ERROR(get_logger(), "Failed to move to home, exiting");
                        return false;
                    }
                    }
                }();
                if (!move_home_result2) return false;

                RCLCPP_INFO(this->get_logger(), "moving home");

        }
}
