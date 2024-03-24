#ifndef RESULT__CHESS_PLAYER_HPP_
#define RESULT__CHESS_PLAYER_HPP_

/**
 * The result of an operation.
 */
enum class Result {
  OK,         // The operation was successful.
  ERR_RETRY,  // The operation failed, but can be retried.
  ERR_FATAL,  // The operation failed and cannot be retried.
};

#endif