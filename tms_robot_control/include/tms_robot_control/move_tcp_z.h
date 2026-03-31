#ifndef MOVE_TCP_Z_ACTION_H
#define MOVE_TCP_Z_ACTION_H

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp> 
#include <tms_robot_interfaces/action/move_tcp_z.hpp>
// #include <memory>
// #include <chrono>

class MoveTcpZAction : public BT::StatefulActionNode {
public:
    using MoveTcpZ = tms_robot_interfaces::action::MoveTcpZ;
    using GoalHandleMovePose = rclcpp_action::ClientGoalHandle<MoveTcpZ>;
    using WrappedResult = GoalHandleMovePose::WrappedResult;
    using ResultFuture = std::shared_future<WrappedResult>;

    MoveTcpZAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config), node_(std::move(node_ptr)), bt_node_name_(name) {
        RCLCPP_INFO(node_->get_logger(), "BT action %s started.", bt_node_name_.c_str());
        client_ = rclcpp_action::create_client<MoveTcpZ>(node_, "move_tcp_z");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>(std::string("distance")),
        };
    }

    BT::NodeStatus onStart() override {
        clearGoalState();
        if (!getInput<float>(std::string("distance"), distance_)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input ports: distance", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Action server 'move_tcp_z' is not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        MoveTcpZ::Goal goal_msg;
        goal_msg.distance = distance_;
        rclcpp_action::Client<MoveTcpZ>::SendGoalOptions send_goal_options;
        try {
            future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while sending move_pose goal: %s", bt_node_name_.c_str(), e.what());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_tcp_z is sent. Waiting for response...", bt_node_name_.c_str());        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] ROS shutting down.", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!goal_handle_) { // Wait for the goal handle once
            if (!future_goal_handle_.valid() || future_goal_handle_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            try {
                goal_handle_ = future_goal_handle_.get();
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError while obtaining move_tcp_z goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while obtaining move_tcp_z goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (!goal_handle_) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Action goal for move_tcp_z was rejected by the server.", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            try { // Once have a valid handle, request the result exactly once
                future_result_ = client_->async_get_result(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError in async_get_result (move_tcp_z): %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception in async_get_result (move_tcp_z): %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
        }
        // Poll for the result
        if (future_result_.valid() && future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            WrappedResult wrapped_result;
            try {
                wrapped_result = future_result_.get();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while getting move_tcp_z result: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_tcp_z succeeded!", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::SUCCESS;
            } 
            else {
                RCLCPP_WARN(node_->get_logger(), "[%s] Action goal for move_tcp_z failed. Result code: %d", bt_node_name_.c_str(), static_cast<int>(wrapped_result.code));
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { 
        RCLCPP_WARN(node_->get_logger(), "BT action %s is halted externally.", bt_node_name_.c_str());
        if (goal_handle_) {
            try {
                (void)client_->async_cancel_goal(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] UnknownGoalHandleError while canceling move_tcp_z goal: %s", bt_node_name_.c_str(), e.what());
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Exception while canceling move_tcp_z goal: %s", bt_node_name_.c_str(), e.what());
            }
        }
        clearGoalState();
    }


private:
    rclcpp::Node::SharedPtr node_;
    std::string bt_node_name_;
    rclcpp_action::Client<MoveTcpZ>::SharedPtr client_;
    GoalHandleMovePose::SharedPtr goal_handle_{nullptr};
    std::shared_future<GoalHandleMovePose::SharedPtr> future_goal_handle_;
    ResultFuture future_result_;
    float distance_{0.f};

    void clearGoalState() {
        goal_handle_.reset();
        future_goal_handle_ = std::shared_future<GoalHandleMovePose::SharedPtr>();
        future_result_ = ResultFuture();
    }


}; 

#endif // MOVE_TCP_Z_ACTION_H