#include "chess_player/chess_player_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std;
using namespace chess_player_params;
using namespace std::chrono_literals;

using chess_msgs::action::FindBestMove;
using libchess::Position;
using libchess::Side;
using placeholders::_1;
using placeholders::_2;
using rclcpp_action::ClientGoalHandle;

//                                                                                                //
// ======================================== Constructor ========================================= //
//                                                                                                //

ChessPlayerNode::ChessPlayerNode(std::string nodename)
  : game_fen_("")
  , white_time_left_(0)
  , black_time_left_(0)
  , enabled_(false)
  , max_speed_(0.1f)
  , game_started_(false)
  , position_("startpos")
{
  // Init node with parameters configured for MoveIt2.
  node = rclcpp::Node::make_shared(
      nodename, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Set up parameters and parameter listener.
  param_listener_ = std::make_unique<ParamListener>(node);
  params_ = std::make_unique<Params>(param_listener_->get_params());

  // Determine which color the cobot is playing as.
  if (params_->cobot_color == "white")
    cobot_color = Side::White;
  else if (params_->cobot_color == "black")
    cobot_color = Side::Black;
  else {
    RCLCPP_ERROR(node->get_logger(), "Invalid cobot color: %s", params_->cobot_color.c_str());
    throw std::runtime_error("Invalid cobot color");
  }

  // Init action client.
  find_best_move_client_ =
      rclcpp_action::create_client<FindBestMove>(node, params_->move_select_action);

  // Init publishers.
  cobot_state_pub_ =
      node->create_publisher<chess_msgs::msg::CobotState>(params_->pub_topics.state, 10);

  // Init subscribers.
  tof_pieces_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_->sub_topics.tof_points, 10, bind(&ChessPlayerNode::tof_pieces_callback_, this, _1));
  game_state_sub_ = node->create_subscription<chess_msgs::msg::FullFEN>(
      params_->sub_topics.game_state, 10, bind(&ChessPlayerNode::game_state_callback_, this, _1));
  time_sub_ = node->create_subscription<chess_msgs::msg::ChessTime>(
      params_->sub_topics.time, 10, bind(&ChessPlayerNode::time_callback_, this, _1));
  enabled_sub_ = node->create_subscription<chess_msgs::msg::CobotEnabled>(
      params_->sub_topics.enabled, 10, bind(&ChessPlayerNode::enabled_callback_, this, _1));
  speed_sub_ = node->create_subscription<chess_msgs::msg::CobotSpeed>(
      params_->sub_topics.max_speed, 10, bind(&ChessPlayerNode::speed_callback_, this, _1));

  // Set up state.
  set_state(State::WAITING_FOR_GAME);
}

//                                                                                                //
// ====================================== Public Functions ====================================== //
//                                                                                                //

void ChessPlayerNode::set_state(State state)
{
  cobot_state_ = state;
  chess_msgs::msg::CobotState msg;
  switch (state) {
    case State::WAITING_FOR_GAME:
      msg.state = "Waiting for game to start";
      break;
    case State::WAITING_FOR_TURN:
      msg.state = "Waiting for turn";
      break;
    case State::WAITING_FOR_OPTIMAL_MOVE:
      msg.state = "Calculating optimal move";
      break;
    case State::FOUND_OPTIMAL_MOVE:
      msg.state = "Planning motion";
      break;
    case State::TAKING_PIECE:
      msg.state = "Taking opponent's piece";
      break;
    case State::MOVING_PIECE:
      msg.state = "Moving piece";
      break;
    case State::HITTING_CLOCK:
      msg.state = "Hitting the clock";
      break;
    case State::MOVING_TO_HOME:
      msg.state = "Moving to home position";
      break;
    case State::DISABLED:
      msg.state = "Disabled";
      break;
    case State::ERROR:
      msg.state = "Error";
      break;
    case State::DRAW:
      msg.state = "Game over (draw)";
      break;
    case State::WIN:
      msg.state = "Game over (win)";
      break;
    case State::LOSE:
      msg.state = "Game over (lose)";
      break;
    case State::STALEMATE:
      msg.state = "Game over (stalemate)";
      break;
  }
  RCLCPP_INFO(node->get_logger(), "Cobot state: %s", msg.state.c_str());
  cobot_state_pub_->publish(msg);
}

ChessPlayerNode::State ChessPlayerNode::get_state() const
{
  return cobot_state_;
}

bool ChessPlayerNode::get_enabled() const
{
  return enabled_;
}

float ChessPlayerNode::get_max_speed() const
{
  return max_speed_;
}

bool ChessPlayerNode::game_started() const
{
  return game_started_;
}

libchess::Position ChessPlayerNode::get_position() const
{
  return position_;
}

const std::vector<Point>& ChessPlayerNode::tof_pieces() const
{
  return last_tof_pieces_;
}

uint32_t ChessPlayerNode::get_time_left(Side color) const
{
  return color == Side::White ? white_time_left_ : black_time_left_;
}

//                                                                                                //
// ====================================== Topic Callbacks ======================================= //
//                                                                                                //

void ChessPlayerNode::tof_pieces_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Verify format of message.
  static const array<sensor_msgs::msg::PointField, 3> expected_fields = []() {
    array<sensor_msgs::msg::PointField, 3> fields;
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;
    return fields;
  }();
  if (msg->fields.size() != expected_fields.size()) {
    RCLCPP_ERROR(node->get_logger(), "Invalid TOF message format");
    return;
  }
  for (size_t i = 0; i < expected_fields.size(); i++) {
    if (msg->fields[i].name != expected_fields[i].name ||
        msg->fields[i].offset != expected_fields[i].offset ||
        msg->fields[i].datatype != expected_fields[i].datatype ||
        msg->fields[i].count != expected_fields[i].count) {
      RCLCPP_ERROR(node->get_logger(), "Invalid TOF message format");
      return;
    }
  }

  // Extract points from message. We iterate over each X field and, since we know that Y and Z come
  // immediately after X, we can safely index into the iterator to get those fields as well.
  last_tof_pieces_.clear();
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  for (; iter_x != iter_x.end(); ++iter_x) {
    last_tof_pieces_.emplace_back(Point{ iter_x[0], iter_x[1], iter_x[2] });
  }
}

void ChessPlayerNode::game_state_callback_(const chess_msgs::msg::FullFEN::SharedPtr msg)
{
  // Update game state.
  game_fen_ = msg->fen;
  position_.set_fen(game_fen_);
  game_started_ = true;
  RCLCPP_INFO(node->get_logger(), "Game state updated: %s", game_fen_.c_str());

  if (!enabled_) return;

  // If the game is over, set the state and return.
  if (position_.is_terminal()) {
    if (position_.is_stalemate())
      set_state(State::STALEMATE);
    else if (position_.is_draw())
      set_state(State::DRAW);
    else
      set_state(cobot_color == position_.turn() ? State::LOSE : State::WIN);

    return;
  }

  // If it is not our turn, set the state and return.
  if (position_.turn() != cobot_color) {
    set_state(State::WAITING_FOR_TURN);
    return;
  }

  // If it is our turn, set the state so that the main loop knows it's time to take its turn.
  set_state(State::WAITING_FOR_OPTIMAL_MOVE);
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
    set_state(State::WAITING_FOR_TURN);
  else if (enabled_ && !game_started_)
    set_state(State::WAITING_FOR_GAME);
  else
    set_state(State::DISABLED);
}

void ChessPlayerNode::speed_callback_(const chess_msgs::msg::CobotSpeed::SharedPtr msg)
{
  max_speed_ = msg->speed;
}
