#ifndef FORCE_HIGHER_CONDITION_H
#define FORCE_HIGHER_CONDITION_H

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

// #include <chrono>
// #include <memory>
// #include <string>
// #include <thread>     

class ForceHigherCondition : public BT::ConditionNode {
public:
    ForceHigherCondition(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::ConditionNode(name, config), node_(std::move(node_ptr)), bt_node_name_(name) {
        RCLCPP_INFO(node_->get_logger(), "BT condition %s started.", bt_node_name_.c_str());
        subscriber_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>("/robotiq_force_torque_sensor_broadcaster/wrench", rclcpp::SensorDataQoS(), std::bind(&ForceHigherCondition::ftCallback, this, std::placeholders::_1));
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>(std::string("min")),
        }; 
    }
    BT::NodeStatus tick() override {
        configureOnce(); 
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_msg_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[%s] Waiting for /robotiq_force_torque_sensor_broadcaster/wrench ...", bt_node_name_.c_str());
            return BT::NodeStatus::RUNNING; 
        }
        const auto& ft = *last_msg_;
        if (ft.wrench.force.z > min_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] force_z value is higher than %.2f N", bt_node_name_.c_str(), min_);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            RCLCPP_INFO(node_->get_logger(), "[%s] force_z value is lower than %.2f N", bt_node_name_.c_str(), min_);
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string bt_node_name_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;
    geometry_msgs::msg::WrenchStamped::SharedPtr last_msg_;
    std::mutex mutex_;
    void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_msg_ = msg;
    }
    float min_{};
    bool configured_{false};
    void configureOnce() { 
        if (configured_) return;
        (void)getInput<float>(std::string("min"), min_);
        configured_ = true;
    }
    
};

#endif // FORCE_HIGHER_CONDITION_H