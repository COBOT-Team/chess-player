#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "chess_player/chess_player_node.hpp"
#include "chess_player/behavior_tree_player.cpp"
#include "chess_player/result.hpp"


using namespace std;
using chess_msgs::action::FindBestMove;
using libchess::Side;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
 // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create an executor to manage the lifecycle of nodes
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create the ChessPlayerNode
    auto chess_player_node = std::make_shared<ChessPlayerNode>();

    // Initialize the behavior tree
    chess_player_node->init_behavior_tree();

    // Add the node to the executor
    executor.add_node(chess_player_node);

    // Spin the node, handling callbacks
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Chess Player node...");

    // Main execution loop for the node
    while (rclcpp::ok()) {
        // Tick the behavior tree (this is where the decision-making happens)
        chess_player_node->take_turn();

        // Spin the executor (process any incoming messages, services, or actions)
        executor.spin_once();
}
