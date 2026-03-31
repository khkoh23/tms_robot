#ifndef BT_EXECUTOR_H
#define BT_EXECUTOR_H

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

// #include <chrono>
// #include <fstream>
// #include <memory>
// #include <mutex>
// #include <atomic>
// #include <string>
// #include <stdexcept>

#include <tms_robot_control/force_higher.h>
#include <tms_robot_control/force_lower.h>
#include <tms_robot_control/move_tcp_z.h>

class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : Node("tms_robot_executor") {

    }

    void initialize() {
        RCLCPP_INFO(this->get_logger(), "BTExecutor node starting up ...");
        blackboard_ = BT::Blackboard::create();
        auto shared_this = shared_from_this();

        factory_.registerBuilder<ForceHigherCondition>("ForceHigher", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<ForceHigherCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<ForceLowerCondition>("ForceLower", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<ForceLowerCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<MoveTcpZAction>("MoveTcpZ", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<MoveTcpZAction>(name, config, shared_this);
            });
        
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&BTExecutor::tick_callback, this));
    }

    ~BTExecutor() {
        RCLCPP_INFO(this->get_logger(), "BTExecutor node shutting down. Cleaning up resources ...");
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
    }

private:
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;

    void tick_callback() {
        auto s = tree_.rootNode()->status();
        if (s == BT::NodeStatus::IDLE || s == BT::NodeStatus::RUNNING) {
          tree_.tickOnce();
        }
    }

}; 

#endif // BT_EXECUTOR_H