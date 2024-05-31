#include "chess_player/chess_player_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std;
using namespace chess_player_params;
using namespace chrono_literals;

using chess_msgs::action::FindBestMove;
using libchess::Position;
using libchess::Side;
using moveit::planning_interface::MoveGroupInterface;
using placeholders::_1;
using placeholders::_2;
using rclcpp_action::ClientGoalHandle;

//                                                                                                //
// ======================================== Constructor ========================================= //
//                                                                                                //

ChessPlayerNode::ChessPlayerNode(string nodename)
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
  param_listener_ = make_unique<ParamListener>(node);
  params_ = make_unique<Params>(param_listener_->get_params());

  // Determine which color the cobot is playing as.
  if (params_->cobot_color == "white")
    cobot_color = Side::White;
  else if (params_->cobot_color == "black")
    cobot_color = Side::Black;
  else {
    RCLCPP_ERROR(node->get_logger(), "Invalid cobot color: %s", params_->cobot_color.c_str());
    throw runtime_error("Invalid cobot color");
  }
  clock_btn_pressed = false;

  // Init move groups.
  main_move_group = make_shared<MoveGroupInterface>(node, params_->move_groups.cobot);
  main_move_group->setMaxAccelerationScalingFactor(1.0);
  gripper_move_group = make_shared<MoveGroupInterface>(node, params_->move_groups.gripper);

  // Init action client.
  find_best_move_client =
      rclcpp_action::create_client<FindBestMove>(node, params_->actions.find_best_move);

  // Init TF listener.
  tf_buffer = make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Setup chessboard transform clock.
  chessboard_transform.header.frame_id = "";
  cb_transform_timer_ = node->create_wall_timer(
      500ms, std::bind(&ChessPlayerNode::cb_transform_timer_callback_, this));

  // Init publishers.
  RCLCPP_INFO(get_logger(), "Init publishers");
  string prefix = params_->cobot_ns.empty() ? "" : params_->cobot_ns + "/";
  cobot_state_pub_ =
      node->create_publisher<chess_msgs::msg::CobotState>(prefix + params_->pub_topics.state, 10);
  servo_twist_cmd_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      prefix + "servo/delta_twist_cmds", 10);

  // Init callback groups.
  reentrant_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  move_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions reentrant_options;
  reentrant_options.callback_group = reentrant_cb_group_;
  rclcpp::SubscriptionOptions move_options;
  move_options.callback_group = move_cb_group_;

  // Init subscribers.
  tof_pieces_sub_ = node->create_subscription<chess_msgs::msg::CameraPoints>(
      params_->sub_topics.tof_points, 10, bind(&ChessPlayerNode::tof_pieces_callback_, this, _1),
      reentrant_options);
  game_state_sub_ = node->create_subscription<chess_msgs::msg::FullFEN>(
      params_->sub_topics.game_state, 10, bind(&ChessPlayerNode::game_state_callback_, this, _1),
      move_options);
  time_sub_ = node->create_subscription<chess_msgs::msg::ChessTime>(
      params_->sub_topics.time, 10, bind(&ChessPlayerNode::time_callback_, this, _1),
      reentrant_options);
  enabled_sub_ = node->create_subscription<chess_msgs::msg::CobotEnabled>(
      prefix + params_->sub_topics.enabled, 10, bind(&ChessPlayerNode::enabled_callback_, this, _1),
      reentrant_options);
  speed_sub_ = node->create_subscription<chess_msgs::msg::CobotSpeed>(
      prefix + params_->sub_topics.max_speed, 10, bind(&ChessPlayerNode::speed_callback_, this, _1),
      reentrant_options);
  clock_btns_sub_ = node->create_subscription<chess_msgs::msg::ClockButtons>(
      "chess/clock_buttons", 10, bind(&ChessPlayerNode::clock_btns_callback_, this, _1),
      reentrant_options);

  // Set up state.
  set_state(State::WAITING_FOR_GAME);
}

//                                                                                                //
// ====================================== Public Functions ====================================== //
//                                                                                                //

rclcpp::Logger ChessPlayerNode::get_logger() const
{
  return node->get_logger();
}

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

const Params& ChessPlayerNode::get_params() const
{
  return *params_;
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

const vector<Point>& ChessPlayerNode::tof_pieces() const
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

void ChessPlayerNode::tof_pieces_callback_(const chess_msgs::msg::CameraPoints::SharedPtr msg)
{
  last_tof_pieces_.clear();
  for (const auto& mp : msg->points) {
    Point p;
    p.x = mp.x;
    p.y = mp.y;
    last_tof_pieces_.emplace_back(p);
  }
}

void ChessPlayerNode::game_state_callback_(const chess_msgs::msg::FullFEN::SharedPtr msg)
{
  game_started_ = true;
  if (game_fen_ == msg->fen) return;

  // Update game state.
  game_fen_ = msg->fen;
  position_.set_fen(game_fen_);
  RCLCPP_INFO(node->get_logger(), "Game state updated: %s", game_fen_.c_str());

  if (!enabled_) return;

  // If we are currently making a move, return without updating the state.
  switch (cobot_state_) {
    case State::WAITING_FOR_GAME:
    case State::WAITING_FOR_TURN:
    case State::DISABLED:
      break;
    default:
      RCLCPP_WARN(node->get_logger(), "Received game state while making move");
      return;
  }

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

  // If it is our turn, set the state and take a turn.
  set_state(State::WAITING_FOR_OPTIMAL_MOVE);
  take_turn_();
}

void ChessPlayerNode::time_callback_(const chess_msgs::msg::ChessTime::SharedPtr msg)
{
  white_time_left_ = msg->white_time_left;
  black_time_left_ = msg->black_time_left;
}

void ChessPlayerNode::enabled_callback_(const chess_msgs::msg::CobotEnabled::SharedPtr msg)
{
  if (enabled_ == msg->enabled) return;
  enabled_ = msg->enabled;
  if (enabled_) {
    if (game_started_) {
      set_state(State::WAITING_FOR_TURN);
      if (position_.turn() == cobot_color) take_turn_();
    } else
      set_state(State::WAITING_FOR_GAME);
  } else {
    main_move_group->stop();
    gripper_move_group->stop();
    set_state(State::DISABLED);
    // this_thread::sleep_for(100ms);
    move_out_of_way_();
  }
}

void ChessPlayerNode::speed_callback_(const chess_msgs::msg::CobotSpeed::SharedPtr msg)
{
  max_speed_ = msg->speed;
  main_move_group->setMaxVelocityScalingFactor(max_speed_);
}

void ChessPlayerNode::cb_transform_timer_callback_()
{
  try {
    const string chessboard_frame = get_params().frames.chessboard;
    const string planning_frame = main_move_group->getPlanningFrame();
    const auto transform =
        tf_buffer->lookupTransform(planning_frame, chessboard_frame, tf2::TimePointZero);
    chessboard_transform = transform;
  } catch (tf2::ExtrapolationException&) {
  } catch (tf2::LookupException&) {
  }
}

void ChessPlayerNode::clock_btns_callback_(const chess_msgs::msg::ClockButtons::SharedPtr msg)
{
  clock_btn_pressed = (msg->white_pressed && cobot_color == Side::White) ||
                      (msg->black_pressed && cobot_color == Side::Black);
}
