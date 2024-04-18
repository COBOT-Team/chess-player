#ifndef CHESS_PLAYER_NODE__CHESS_PLAYER_HPP_
#define CHESS_PLAYER_NODE__CHESS_PLAYER_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chess_msgs/action/find_best_move.hpp>
#include <chess_msgs/msg/camera_points.hpp>
#include <chess_msgs/msg/chess_move_uci.hpp>
#include <chess_msgs/msg/chess_time.hpp>
#include <chess_msgs/msg/clock_buttons.hpp>
#include <chess_msgs/msg/cobot_enabled.hpp>
#include <chess_msgs/msg/cobot_speed.hpp>
#include <chess_msgs/msg/cobot_state.hpp>
#include <chess_msgs/msg/full_fen.hpp>
#include <chess_player_params.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <libchess/position.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "chess_player/result.hpp"

/**
 * A point in 3D space.
 */
struct Point {
  int x;
  int y;
};

class ChessPlayerNode
{
public:
  /**
   * State of the cobot.
   */
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
    DRAW,
    WIN,
    LOSE,
    STALEMATE,
  };

  rclcpp::Node::SharedPtr node;             // The ROS 2 node for the chess player.
  libchess::Side cobot_color;               // The color that the cobot is playing as.
  chess_msgs::msg::ChessMoveUCI best_move;  // The best move found by the chess engine.
  bool clock_btn_pressed;                   // True if this cobot's clock button is pressed.
  std::mutex make_move_mutex;               // Mutex for planning and making a move.

  // The move group for the arm of the cobot.
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> main_move_group;

  // The move group for the gripper.
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;

  // The action client for finding the best move.
  std::shared_ptr<rclcpp_action::Client<chess_msgs::action::FindBestMove>> find_best_move_client;

  // Transform buffer for the TF listener.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  // Servo used for realtime servoing.
  std::unique_ptr<moveit_servo::Servo> servo;

  // Twist publisher for servoing.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_cmd_pub;

  // Transform between planning frame and chessboard frame.
  geometry_msgs::msg::TransformStamped chessboard_transform;

  /**
   * Construct a new Chess Player Node object.
   */
  explicit ChessPlayerNode(std::string node_name = "chess_player");

  /**
   * Get the node's logger.
   *
   * @return The node's logger.
   */
  rclcpp::Logger get_logger() const;

  /**
   * Update the current state of the cobot and publish a string message to the GUI.
   *
   * @param[in] state The new state of the cobot.
   */
  void set_state(State state);

  /**
   * Get state of the cobot.
   *
   * @return The state of the cobot.
   */
  State get_state() const;

  /**
   * Get the parameters for the chess player.
   *
   * @return The parameters for the chess player.
   */
  const chess_player_params::Params& get_params() const;

  /**
   * Get whether the cobot is enabled or not.
   *
   * @return Whether the cobot is enabled or not.
   */
  bool get_enabled() const;

  /**
   * Get the maximum speed of the cobot.
   *
   * @return The maximum speed of the cobot in m/s.
   */
  float get_max_speed() const;

  /**
   * Get whether the game has started or not.
   *
   * @return Whether the game has started or not.
   */
  bool game_started() const;

  /**
   * Get the current position of the chess board.
   *
   * @return The current position of the chess board.
   */
  libchess::Position get_position() const;

  /**
   * Get a list of the last detected pieces from the ToF camera.
   *
   * @return A list of the last detected pieces from the ToF camera.
   */
  const std::vector<Point>& tof_pieces() const;

  /**
   * Get the time remaining for a specific color.
   *
   * @param color The color to get the time for.
   * @return The time remaining for the specified color in milliseconds.
   */
  uint32_t get_time_left(libchess::Side color) const;

private:
  /**
   * The current state of the cobot.
   */
  State cobot_state_;

  /**
   * Callback that is called for updates to the list of TOF pieces.
   *
   * @param[in] msg The message containing the point cloud of pieces.
   */
  void tof_pieces_callback_(const chess_msgs::msg::CameraPoints::SharedPtr msg);

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
   * Callback that is called for updates to the clock buttons.
   *
   * @param[in] msg The message containing the clock buttons.
   */
  void clock_btns_callback_(const chess_msgs::msg::ClockButtons::SharedPtr msg);

  /**
   * Callback that is called periodically because to update the chessboard transform.
   *
   */
  void cb_transform_timer_callback_();

  /**
   * Query the action server for the best move.
   *
   * @return The result of the operation.
   */
  Result find_best_move_();

  /**
   * Capture a piece at a given square. This will pick up the piece at the given square and remove
   * it from the board.
   *
   * @param[in] square The square to capture a piece at.
   * @return The result of the operation.
   */
  Result capture_at_(const libchess::Square& square);

  /**
   * Move a piece from one square to another. This will move the piece from the first square to the
   * second square.
   *
   * @param[in] move The move to make.
   * @return The result of the operation.
   */
  Result move_piece_(const libchess::Move& move);

  /**
   * Press the clock to end the turn.
   *
   * @return The result of the operation.
   */
  Result hit_clock_();

  /**
   * Move the cobot to the home position. This is used when the cobot is waiting to take a turn.
   *
   * @return The result of the operation.
   */
  Result move_home_();

  /**
   * Move the cobot out of the way of the board. This is used when the cobot is disabled.
   *
   * @return The result of the operation.
   */
  Result move_out_of_way_();

  /**
   * Move the cobot to take a turn in the chess game.
   *
   * @return False if a fatal error has ocurred, true otherwise.
   */
  bool take_turn_();

  bool enabled_;                 // Whether the cobot is enabled or not.
  float max_speed_;              // The maximum speed of the cobot in m/s.
  std::string state_msg_;        // The state of the cobot to be displayed in the GUI.
  bool game_started_;            // Whether the game has started or not.
  libchess::Position position_;  // The current position of the chess board.

  std::vector<Point> last_tof_pieces_;  // Last detected pieces from the ToF camera.
  std::string game_fen_;                // Current game state in FEN notation.
  uint32_t white_time_left_;            // White player's time left in milliseconds.
  uint32_t black_time_left_;            // Black player's time left in milliseconds.

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<chess_player_params::ParamListener> param_listener_;
  std::unique_ptr<chess_player_params::Params> params_;

  rclcpp::Publisher<chess_msgs::msg::CobotState>::SharedPtr cobot_state_pub_;

  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;
  rclcpp::CallbackGroup::SharedPtr move_cb_group_;

  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::CameraPoints>> tof_pieces_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::FullFEN>> game_state_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::ChessTime>> time_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::CobotEnabled>> enabled_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::CobotSpeed>> speed_sub_;
  std::shared_ptr<rclcpp::Subscription<chess_msgs::msg::ClockButtons>> clock_btns_sub_;
  rclcpp::TimerBase::SharedPtr cb_transform_timer_;
};

#endif