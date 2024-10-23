#include <behaviortree_cpp_v3/behavior_tree.h>
#include <string> 
#include "chess_player/chess_player_node.hpp"

#ifndef BY_NODES_HPP
#define BT_NODES_HPP

// Custom BT node to set up the bot for its turn 
class TurnSetupAction : public BT::SyncActionNode {
public:
    TurnSetupAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        if (turn_setup()) { // Just call the existing function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Custom BT node to find the best move
class FindBestMoveAction : public BT::SyncActionNode {
public:
    FindBestMoveAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        if (_find_best_move_()) { // Call the function directly
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};


// Custom BT node to parse moves 
class ParseMoveAction : public BT::SyncActionNode {
public:
    ParseMoveAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        std::string move;
        getInput("move", move);

        if (parse_move_(move)) { // Call the function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("move"), BT::OutputPort<libchess::Move>("parsed_move") };
    }
};

// Custom BT node to capture pieces
class CapturePieceAction : public BT::SyncActionNode {
public:
    CapturePieceAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        std::string move;
        getInput("parsed_move", move);

        if (capture_piece_(move)) { // Call the function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("parsed_move") };
    }
};

// Custom BT node to move the piece 
class MovePieceAction : public BT::SyncActionNode {
public:
    MovePieceAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        std::string move;
        getInput("parsed_move", move);

        if (move_piece_(move)) { // Call the function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("parsed_move") };
    }
};

// Custom BT node to hit the chess clock
class HitClockAction : public BT::SyncActionNode {
public:
    HitClockAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        if (hit_clock_()) { // Call the function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Custom BT node to move home 
class MoveHomeAction : public BT::SyncActionNode {
public:
    MoveHomeAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        if (move_home_()) { // Call the function
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};

#endif 
