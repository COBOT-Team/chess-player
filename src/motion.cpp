#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <libchess/position.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "chess_player/chess_player_node.hpp"
#include "chess_player/result.hpp"

using namespace std;

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;

//                                                                                                //
// ===================================== Utility Functions ====================================== //
//                                                                                                //

/**
 * Return the pose of the end effector over a square on the chessboard.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] square The square on the chessboard.
 * @param[in] dist_above_board The distance above the board to hover.
 * @return The pose of the end effector over the square, in the base_link frame.
 */
Pose get_pose_over_square(ChessPlayerNode& chess_player, const libchess::Square& square,
                          double dist_above_board)
{
  const auto params = chess_player.get_params();
  const double square_size = params.measurements.chessboard_square_size;

  // Extreme rows and columns are offset from the center by 3.5x the square size.
  const double col_zero = -square_size * 3.5;
  const double row_zero = -square_size * 3.5;

  // Pick the correct pose in the chessboard frame.
  const auto pose = [&] {
    const int row = square.rank();
    const int col = square.file();

    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);

    Pose p;
    p.position.x = col_zero + col * square_size;
    p.position.y = row_zero + row * square_size;
    p.position.z = dist_above_board;
    p.orientation = tf2::toMsg(q);

    return p;
  }();

  // Transform the pose to the planning frame.
  const auto base_link_pose = [&] {
    const string chessboard_frame = chess_player.get_params().frames.chessboard;
    const string planning_frame = chess_player.main_move_group->getPlanningFrame();
    const auto transform = chess_player.tf_buffer->lookupTransform(planning_frame, chessboard_frame,
                                                                   tf2::TimePointZero);

    Pose base_link_pose;
    tf2::doTransform(pose, base_link_pose, transform);
    return base_link_pose;
  }();

  return base_link_pose;
}

/**
 * Move the end effector to a pose.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] pose The pose to move to.
 * @return The result of the operation.
 */
Result move_to_pose(ChessPlayerNode& chess_player, const Pose& pose)
{
  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Set the pose target.
  if (!chess_player.main_move_group->setPoseTarget(pose)) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to set pose target");
    return Result::ERR_RETRY;
  }

  // Plan the motion.
  const auto [success, plan] = [&] {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto ok = static_cast<bool>(chess_player.main_move_group->plan(plan));
    return make_pair(ok, plan);
  }();
  if (!success) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to plan motion");
    return Result::ERR_FATAL;
  }

  // Make sure the cobot wasn't disabled during motion planning.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; discarding planned motion");
    return Result::ERR_FATAL;
  }

  // Execute the motion.
  const auto execute_result = chess_player.main_move_group->execute(plan);
  switch (execute_result.val) {
    case moveit::core::MoveItErrorCode::SUCCESS:
    case moveit::core::MoveItErrorCode::TIMED_OUT:
      return Result::OK;
    default:
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to execute motion");
      return Result::ERR_FATAL;
  }
}

/**
 * Move the end effector to a pose using Cartesian motion. This is useful for moving the end
 * effector in a straight line to a pose.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] pose The pose to move to.
 * @return The result of the operation.
 */
Result move_to_pose_cartesian(ChessPlayerNode& chess_player, const Pose& pose)
{
  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Set the waypoints. The first waypoint is the current pose, and the second waypoint is the
  // target pose.
  vector<Pose> waypoints;
  waypoints.push_back(chess_player.main_move_group->getCurrentPose().pose);
  waypoints.push_back(pose);

  // Plan the motion.
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
      chess_player.main_move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
  if (fraction < 1.0) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to plan Cartesian motion");
    return Result::ERR_FATAL;
  }

  // Make sure the cobot wasn't disabled during motion planning.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; discarding planned motion");
    return Result::ERR_FATAL;
  }

  // Execute the motion.
  const auto execute_result = chess_player.main_move_group->execute(trajectory);
  switch (execute_result.val) {
    case moveit::core::MoveItErrorCode::SUCCESS:
    case moveit::core::MoveItErrorCode::TIMED_OUT:
      return Result::OK;
    default:
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to execute motion");
      return Result::ERR_FATAL;
  }
}

//                                                                                                //
// ===================================== Motion Components ====================================== //
//                                                                                                //

Result move_above_square(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  const auto target_pose = get_pose_over_square(
      chess_player, square, chess_player.get_params().measurements.hover_above_board);
  return move_to_pose(chess_player, target_pose);
}

Result align_to_piece(ChessPlayerNode& chess_player)
{
  if (chess_player.tof_pieces().empty()) {
    RCLCPP_WARN(chess_player.get_logger(), "No pieces detected; not aligning");
    return Result::OK;
  }

  // Find the point nearest to the center of the TOF camera.
  const auto [nearest_piece_dist, nearest_piece] = [&chess_player] {
    double min_dist = numeric_limits<double>::infinity();
    Point nearest_piece;
    for (auto& piece : chess_player.tof_pieces()) {
      const auto dist = piece.x * piece.x + piece.y * piece.y;
      if (dist < min_dist) {
        min_dist = dist;
        nearest_piece = piece;
      }
    }

    return make_pair(min_dist, nearest_piece);
  }();

  // If we're already close enough, don't bother realigning.
  if (nearest_piece_dist < chess_player.get_params().measurements.min_realign_dist)
    return Result::OK;

  // Move above the piece.
  const auto new_pose = [&] {
    Pose pose_tof_frame;
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    pose_tof_frame.position.x = nearest_piece.x;
    pose_tof_frame.position.y = nearest_piece.y;
    pose_tof_frame.position.z = nearest_piece.z + 0.2;
    pose_tof_frame.orientation = tf2::toMsg(q);

    const string tof_frame = chess_player.get_tof_frame_id();
    const string planning_frame = chess_player.main_move_group->getPlanningFrame();
    const auto transform =
        chess_player.tf_buffer->lookupTransform(planning_frame, tof_frame, tf2::TimePointZero);

    Pose pose_planning_frame;
    tf2::doTransform(pose_tof_frame, pose_planning_frame, transform);
    return pose_planning_frame;
  }();
  return move_to_pose(chess_player, new_pose);
}

/**
 * Pick up a piece from a square that the gripper is above.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to pick up a piece from.
 * @return The result of the operation.
 */
Result pick_up_piece(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  // Move the gripper above the square.
  {
    const auto result = move_above_square(chess_player, square);
    if (result != Result::OK) return result;
  }

  // Align the cobot with the piece.
  {
    const auto result = align_to_piece(chess_player);
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Calculate the pose at the center of the square, on the board surface. This is used to calculate
  // relative poses later.
  const auto pose_at_square = [&] {
    auto pose = chess_player.main_move_group->getCurrentPose().pose;
    pose.position.z -= chess_player.get_params().measurements.hover_above_board;
    return pose;
  }();

  // Move the end effector down to pick up the piece.
  {
    const auto down_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z += chess_player.get_params().measurements.min_grasp_height;
      return pose;
    }();
    // const auto result = move_to_pose_cartesian(chess_player, down_pose);
    const auto result = move_to_pose(chess_player, down_pose);
    if (result != Result::OK) return result;
  }

  // Close the gripper.
  {
    chess_player.gripper_move_group->setNamedTarget("close");
    if (!chess_player.gripper_move_group->move()) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to close gripper");
      return Result::ERR_RETRY;
    }
  }

  // Move the end effector back up.
  {
    const auto up_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z += chess_player.get_params().measurements.hover_above_board;
      return pose;
    }();
    // const auto result = move_to_pose_cartesian(chess_player, up_pose);
    const auto result = move_to_pose(chess_player, up_pose);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

/**
 * Place a held piece at a square.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to place a piece at.
 * @return The result of the operation.
 */
Result place_piece(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  // Move the gripper above the square.
  {
    const auto result = move_above_square(chess_player, square);
    if (result != Result::OK) return result;
  }

  // Calculate the pose at the center of the square, on the board surface.
  const auto pose_at_square = [&] {
    auto pose = chess_player.main_move_group->getCurrentPose().pose;
    pose.position.z -= chess_player.get_params().measurements.hover_above_board;
    return pose;
  }();

  // Move the end effector down to place the piece.
  {
    const auto down_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z += chess_player.get_params().measurements.min_grasp_height;
      return pose;
    }();
    const auto result = move_to_pose_cartesian(chess_player, down_pose);
    if (result != Result::OK) return result;
  }

  // Open the gripper.
  {
    chess_player.gripper_move_group->setNamedTarget("open");
    if (!chess_player.gripper_move_group->move()) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to open gripper");
      return Result::ERR_RETRY;
    }
  }

  // Move the end effector back up.
  {
    const auto up_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z += chess_player.get_params().measurements.hover_above_board;
      return pose;
    }();
    const auto result = move_to_pose_cartesian(chess_player, up_pose);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

/**
 * Deposit a captured piece somewhere off of the board.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result deposit_captured_piece(ChessPlayerNode& chess_player)
{
  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Move the gripper to the deposit location.
  chess_player.main_move_group->setNamedTarget("deposit");
  if (!chess_player.main_move_group->move()) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to move to deposit location");
    return Result::ERR_RETRY;
  }

  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Open the gripper.
  chess_player.gripper_move_group->setNamedTarget("open");
  if (!chess_player.gripper_move_group->move()) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to open gripper");
    return Result::ERR_RETRY;
  }

  return Result::OK;
}

//                                                                                                //
// ====================================== Class Functions ======================================= //
//                                                                                                //

Result ChessPlayerNode::capture_at_(const libchess::Square& square)
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Pick up the piece.
  {
    const auto result = pick_up_piece(*this, square);
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Deposit the piece.
  {
    const auto result = deposit_captured_piece(*this);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_piece_(const libchess::Move& move)
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Pick up the piece at the from square.
  {
    const auto result = pick_up_piece(*this, move.from());
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Place the piece at the to square.
  {
    const auto result = place_piece(*this, move.to());
    if (result != Result::OK) return result;
  }

  // TODO: Promote the piece if necessary.

  return Result::OK;
}

Result ChessPlayerNode::hit_clock_()
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Move the gripper to the clock location.
  main_move_group->setNamedTarget("above_clock");
  if (!main_move_group->move()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move to clock location");
    return Result::ERR_RETRY;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Press the clock.
  gripper_move_group->setNamedTarget("press_clock");
  if (!gripper_move_group->move()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to press clock");
    return Result::ERR_RETRY;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Move the gripper back up.
  gripper_move_group->setNamedTarget("above_clock");
  if (!gripper_move_group->move()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move back up");
    return Result::ERR_RETRY;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_home_()
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  main_move_group->setNamedTarget("home");
  if (!main_move_group->move()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move to home location");
    return Result::ERR_RETRY;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_out_of_way_()
{
  main_move_group->setNamedTarget("out_of_way");
  if (!main_move_group->move()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move out of the way");
    return Result::ERR_RETRY;
  }

  return Result::OK;
}