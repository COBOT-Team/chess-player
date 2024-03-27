#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "chess_player/chess_player_node.hpp"
#include "chess_player/result.hpp"

using namespace std;
using chess_msgs::action::FindBestMove;
using libchess::Side;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ChessPlayerNode chess_player;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(chess_player.node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
