#ifndef CHESS_PLAYER_NODE__CHESS_PLAYER_HPP_
#define CHESS_PLAYER_NODE__CHESS_PLAYER_HPP_

#include <pcl/point_cloud.h>

#include <chess_msgs/action/find_best_move.hpp>
#include <chess_msgs/msg/chess_move_uci.hpp>
#include <chess_msgs/msg/chess_time.hpp>
#include <chess_msgs/msg/cobot_enabled.hpp>
#include <chess_msgs/msg/cobot_speed.hpp>
#include <chess_msgs/msg/cobot_state.hpp>
#include <chess_msgs/msg/full_fen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ChessPlayerNode
{
public:
  /**
   * Construct a new Chess Player Node object.
   */
  explicit ChessPlayerNode(std::string node_name = "chess_player");

private:
  /**
   * A color of a player or a piece.
   */
  enum class Color { WHITE = 0, BLACK = 1 };

  /**
   * The current state of the cobot.
   */
  struct CobotState {
    enum class State {
      WAITING_FOR_GAME,
      WAITING_FOR_TURN,
      WAITING_FOR_OPTIMAL_MOVE,
      FOUND_OPTIMAL_MOVE,
      TAKING_PIECE,
      MOVING_PIECE,
      HITTING_CLOCK,
      MOVING_TO_HOME,
      DISABLED,
      ERROR,
    };

    State state;
  } cobot_state_;

  /**
   * Manipulate the AR3 to make a move.
   */
  void take_turn_();

  /**
   * Get the time remaining for a specific color.
   *
   * @param color The color to get the time for.
   * @return The time remaining for the specified color in milliseconds.
   */
  uint32_t get_time_left_(Color color) const;

  /**
   * Update the current state of the cobot and publish a string message to the GUI.
   *
   */
  void set_state_(CobotState::State state);

  /**
   * Callback that is called for updates to the list of TOF pieces.
   *
   * @param[in] msg The message containing the point cloud of pieces.
   */
  void tof_pieces_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * Callback that is called for updates to the game state.
   *
   * @param[in] msg The message containing the game state in FEN notation.
   */
  void game_state_callback_(const chess_msgs::msg::FullFEN::SharedPtr msg);

  /**
   * Callback that is called for updates to the time remaining for each player.
   *
   * @param[in] msg The message containing the time remaining for each player.
   */
  void time_callback_(const chess_msgs::msg::ChessTime::SharedPtr msg);

  /**
   * Callback that is called for updates to the cobot enabled state.
   *
   * @param[in] msg The message containing the cobot enabled state.
   */
  void enabled_callback_(const chess_msgs::msg::CobotEnabled::SharedPtr msg);

  /**
   * Callback that is called for updates to the cobot's max speed.
   *
   * @param[in] msg The message containing the cobot max speed.
   */
  void speed_callback_(const chess_msgs::msg::CobotSpeed::SharedPtr msg);

  /**
   * Callback that is called with the response of the chess engine's action server.
   *
   * @param[in] goal_handle The goal handle for the action server.
   */
  void goal_response_callback_(
      const rclcpp_action::ClientGoalHandle<chess_msgs::action::FindBestMove>::SharedPtr&
          goal_handle);

  /**
   * Callback that is called with the feedback of the chess engine's action server.
   *
   * @param[in] goal_handle Goal handle for the action server.
   * @param[in] feedback The feedback from the action server.
   */
  void feedback_callback_(
      rclcpp_action::ClientGoalHandle<chess_msgs::action::FindBestMove>::SharedPtr goal_handle,
      const std::shared_ptr<const chess_msgs::action::FindBestMove::Feedback> feedback);

  /**
   * Callback that is called with the result of the chess engine's action server.
   *
   * @param[in] result The result from the action server.
   */
  void result_callback_(
      const rclcpp_action::ClientGoalHandle<chess_msgs::action::FindBestMove>::WrappedResult&
          result);

  Color cobot_color_;                        // The color that the cobot is playing as.
  bool enabled_;                             // Whether the cobot is enabled or not.
  float max_speed_;                          // The maximum speed of the cobot in m/s.
  std::string state_msg_;                    // The state of the cobot to be displayed in the GUI.
  bool game_started_;                        // Whether the game has started or not.
  chess_msgs::msg::ChessMoveUCI best_move_;  // The best move found by the chess engine.

  std::vector<pcl::PointXYZ> last_tof_pieces_;  // Last detected pieces from the ToF camera.
  std::string game_fen_;                        // Current game state in FEN notation.
  uint32_t white_time_left_;                    // White player's time left in milliseconds.
  uint32_t black_time_left_;                    // Black player's time left in milliseconds.

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<chess_player_params::ParamListener> param_listener_;
  std::unique_ptr<chess_player_params::Params> params_;

  std::shared_ptr<rclcpp::Publisher<chess_msgs::msg::CobotState>> cobot_state_pub_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> tof_pieces_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::FullFEN>> game_state_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::ChessTime>> time_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::CobotEnabled>> enabled_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::CobotSpeed>> speed_sub_;
};

#endif