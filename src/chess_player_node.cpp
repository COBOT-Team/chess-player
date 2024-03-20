#include "chess_player/chess_player_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <chess_player_params.hpp>

using namespace std;
using namespace chess_player_params;

using chess_msgs::action::FindBestMove;
using placeholders::_1;
using rclcpp_action::ClientGoalHandle;

//                                                                                                //
// ======================================== Constructor ========================================= //
//                                                                                                //

ChessPlayerNode::ChessPlayerNode(std::string node_name)
  : game_fen_("")
  , white_time_left_(0)
  , black_time_left_(0)
  , enabled_(false)
  , max_speed_(0.1f)
  , game_started_(false)
{
  // Init node with parameters configured for MoveIt2.
  node_ = rclcpp::Node::make_shared(
      "chess_player", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Set up parameters and parameter listener.
  param_listener_ = std::make_unique<ParamListener>(node_);
  params_ = std::make_unique<Params>(param_listener_->get_params());

  // Determine which color the cobot is playing as.
  if (params_->cobot_color == "white")
    cobot_color_ = Color::WHITE;
  else if (params_->cobot_color == "black")
    cobot_color_ = Color::BLACK;
  else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid cobot color: %s", params_->cobot_color.c_str());
    throw std::runtime_error("Invalid cobot color");
  }

  // Init publishers.
  cobot_state_pub_ =
      node_->create_publisher<chess_msgs::msg::CobotState>(params_->pub_topics.state, 10);

  // Init subscribers.
  tof_pieces_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_->sub_topics.tof_points, 10, bind(&ChessPlayerNode::tof_pieces_callback_, this, _1));
  game_state_sub_ = node_->create_subscription<chess_msgs::msg::FullFEN>(
      params_->sub_topics.game_state, 10, bind(&ChessPlayerNode::game_state_callback_, this, _1));
  time_sub_ = node_->create_subscription<chess_msgs::msg::ChessTime>(
      params_->sub_topics.time, 10, bind(&ChessPlayerNode::time_callback_, this, _1));
  enabled_sub_ = node_->create_subscription<chess_msgs::msg::CobotEnabled>(
      params_->sub_topics.enabled, 10, bind(&ChessPlayerNode::enabled_callback_, this, _1));
  speed_sub_ = node_->create_subscription<chess_msgs::msg::CobotSpeed>(
      params_->sub_topics.max_speed, 10, bind(&ChessPlayerNode::speed_callback_, this, _1));

  // Set up state.
  set_state_(CobotState::State::WAITING_FOR_GAME);
}

//                                                                                                //
// ====================================== Main MoveIt Code ====================================== //
//                                                                                                //

void ChessPlayerNode::take_turn_() {}

//                                                                                                //
// ====================================== Boring Functions ====================================== //
//                                                                                                //

uint32_t ChessPlayerNode::get_time_left_(Color color) const
{
  return color == Color::WHITE ? white_time_left_ : black_time_left_;
}

void ChessPlayerNode::set_state_(CobotState::State state)
{
  cobot_state_.state = state;
  chess_msgs::msg::CobotState msg;
  switch (state) {
    case CobotState::State::WAITING_FOR_GAME:
      msg.state = "Waiting for game to start";
      break;
    case CobotState::State::WAITING_FOR_TURN:
      msg.state = "Waiting for turn";
      break;
    case CobotState::State::WAITING_FOR_OPTIMAL_MOVE:
      msg.state = "Calculating optimal move";
      break;
    case CobotState::State::FOUND_OPTIMAL_MOVE:
      msg.state = "Planning motion";
      break;
    case CobotState::State::TAKING_PIECE:
      msg.state = "Taking opponent's piece";
      break;
    case CobotState::State::MOVING_PIECE:
      msg.state = "Moving piece";
      break;
    case CobotState::State::HITTING_CLOCK:
      msg.state = "Hitting the clock";
      break;
    case CobotState::State::MOVING_TO_HOME:
      msg.state = "Moving to home position";
      break;
    case CobotState::State::DISABLED:
      msg.state = "Disabled";
      break;
    case CobotState::State::ERROR:
      msg.state = "Error";
      break;
  }
  cobot_state_pub_->publish(msg);
}

//                                                                                                //
// ====================================== Topic Callbacks ======================================= //
//                                                                                                //

void ChessPlayerNode::tof_pieces_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  last_tof_pieces_.clear();
  last_tof_pieces_.insert(last_tof_pieces_.begin(), cloud->begin(), cloud->end());
}

void ChessPlayerNode::game_state_callback_(const chess_msgs::msg::FullFEN::SharedPtr msg)
{
  game_fen_ = msg->fen;
  // TODO: This is where the fun begins.
}

void ChessPlayerNode::time_callback_(const chess_msgs::msg::ChessTime::SharedPtr msg)
{
  white_time_left_ = msg->white_time_left;
  black_time_left_ = msg->black_time_left;
}

void ChessPlayerNode::enabled_callback_(const chess_msgs::msg::CobotEnabled::SharedPtr msg)
{
  enabled_ = msg->enabled;
  if (enabled_ && game_started_)
    set_state_(CobotState::State::WAITING_FOR_TURN);
  else if (enabled_ && !game_started_)
    set_state_(CobotState::State::WAITING_FOR_GAME);
  else
    set_state_(CobotState::State::DISABLED);
}

void ChessPlayerNode::speed_callback_(const chess_msgs::msg::CobotSpeed::SharedPtr msg)
{
  max_speed_ = msg->speed;
}

//                                                                                                //
// ====================================== Action Callbacks ====================================== //
//                                                                                                //

void ChessPlayerNode::goal_response_callback_(
    const ClientGoalHandle<FindBestMove>::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Chess engine action server rejected goal");
    set_state_(CobotState::State::ERROR);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Chess engine is calculating the best move");
    set_state_(CobotState::State::WAITING_FOR_OPTIMAL_MOVE);
  }
}

void ChessPlayerNode::feedback_callback_(
    ClientGoalHandle<FindBestMove>::SharedPtr goal_handle,
    const std::shared_ptr<const FindBestMove::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "Engine feedback: %s %s", feedback->info.type.c_str(),
              feedback->info.value.c_str());
}

void ChessPlayerNode::result_callback_(const ClientGoalHandle<FindBestMove>::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      best_move_ = result.result->move;
      RCLCPP_INFO(node_->get_logger(), "Engine found the best move: %s",
                  best_move_.draw   ? "draw" :
                  best_move_.resign ? "resign" :
                                      best_move_.move.c_str());
      set_state_(CobotState::State::FOUND_OPTIMAL_MOVE);
      break;
    }
    case rclcpp_action::ResultCode::ABORTED: {
      RCLCPP_ERROR(node_->get_logger(), "Chess engine action was aborted");
      set_state_(CobotState::State::ERROR);
      break;
    }
    case rclcpp_action::ResultCode::CANCELED: {
      RCLCPP_ERROR(node_->get_logger(), "Chess engine action was canceled");
      set_state_(CobotState::State::WAITING_FOR_TURN);
      break;
    }
    default: {
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code from chess engine action");
      set_state_(CobotState::State::ERROR);
      break;
    }
  }
}