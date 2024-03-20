#include "chess_player/motion.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;

/**
 * Return the pose of the square on the chessboard.
 * 
 * @param[in] chess_player 
 * @param[in] square 
 * @return
 */
Pose get_pose_at_square(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  const auto params = chess_player.get_params();
  const double square_size = params.chessboard_square_size;

  // Extreme rows and columns are offset from the center by 3.5x the square size.
  const double col_zero = -square_size * 3.5;
  const double row_zero = -square_size * 3.5;

  // Pick the correct transform from the center of the board.
  const auto transform = [&] {
    const int row = square.rank();
    const int col = square.file();
    Transform t;
    t.translation.x = col_zero + col * square_size;
    t.translation.y = row_zero + row * square_size;
  };
}

Result move_above_square(ChessPlayerNode& chess_player, const libchess::Square& square) {}