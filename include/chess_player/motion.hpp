#ifndef MOTION__CHESS_PLAYER_HPP_
#define MOTION__CHESS_PLAYER_HPP_

#include <libchess/position.hpp>

#include "chess_player/chess_player_node.hpp"
#include "chess_player/result.hpp"

/**
 * Move the gripper above a square on the board.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to move the gripper above.
 * @return The result of the operation.
 */
Result move_above_square(ChessPlayerNode& chess_player, const libchess::Square& square);

/**
 * Uses the TOF sensor to align the cobot with the nearest piece. This should be used before picking
 * up a piece to ensure that the cobot is properly aligned to it.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result align_to_piece(ChessPlayerNode& chess_player);

/**
 * Pick up a piece from a square that the gripper is above.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to pick up a piece from.
 * @return The result of the operation.
 */
Result pick_up_piece(ChessPlayerNode& chess_player, const libchess::Square& square);

/**
 * Place a held piece at a square.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to place a piece at.
 * @return The result of the operation.
 */
Result place_piece(ChessPlayerNode& chess_player, const libchess::Square& square);

/**
 * Deposit a captured piece somewhere off of the board.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result deposit_captured_piece(ChessPlayerNode& chess_player);

/**
 * Press the clock to end the turn.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result hit_clock(ChessPlayerNode& chess_player);

/**
 * Move the cobot to the home position. This is used when the cobot is waiting to take a turn.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result move_home(ChessPlayerNode& chess_player);

/**
 * Move the cobot out of the way of the board. This is used when the cobot is disabled.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result move_out_of_the_way(ChessPlayerNode& chess_player);

#endif