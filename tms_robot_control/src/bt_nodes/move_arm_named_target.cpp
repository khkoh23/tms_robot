#include "tms_robot_control/bt_nodes/move_arm_named_target.hpp"

#include <chrono>
#include <exception>
#include <future>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

using namespace std::chrono_literals;

MoveArmNamedTargetNode::MoveArmNamedTargetNode(const std::string & name, const BT::NodeConfig & config) : BT::StatefulActionNode(name, config) {
}

BT::PortsList MoveArmNamedTargetNode::providedPorts() {
  return {
    BT::InputPort<std::string>("target", "SRDF named target"),
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group")
  };
}

BT::NodeStatus MoveArmNamedTargetNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveArmNamedTargetNode"), "Cancel requested before motion start");
    return BT::NodeStatus::FAILURE;
  }
  auto target = getInput<std::string>("target");
  auto planning_group = getInput<std::string>("planning_group");
  if (!target) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Missing required input port: target");
    return BT::NodeStatus::FAILURE;
  }
  if (!planning_group) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Missing required input port: planning_group");
    return BT::NodeStatus::FAILURE;
  }
  target_name_ = target.value();
  planning_group_ = planning_group.value();
  try {
    if (!initialized_) {
      auto ros_node = get_ros_node_from_blackboard(config());
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(ros_node, planning_group_);
      initialized_ = true;
    }
    RCLCPP_INFO(rclcpp::get_logger("MoveArmNamedTargetNode"), "Planning move to named target '%s' using planning group '%s'", target_name_.c_str(), planning_group_.c_str());
    move_group_->setNamedTarget(target_name_);
    const auto plan_result = move_group_->plan(plan_);
    if (plan_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Planning failed for target '%s'. MoveIt error code: %d", target_name_.c_str(), plan_result.val);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("MoveArmNamedTargetNode"), "Planning succeeded. Starting execution.");
    execution_future_ = std::async(std::launch::async, [this]() { 
      return move_group_->execute(plan_); 
    });
    return BT::NodeStatus::RUNNING;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Exception during planning/execution start: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveArmNamedTargetNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveArmNamedTargetNode"), "Cancel requested. Stopping MoveIt execution.");
    if (move_group_) {
      move_group_->stop();
      move_group_->clearPoseTargets();
    }
    return BT::NodeStatus::FAILURE;
  }
  if (!execution_future_.valid()) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Execution future is invalid");
    return BT::NodeStatus::FAILURE;
  }
  const auto future_status = execution_future_.wait_for(0ms);
  if (future_status != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }
  try {
    const auto execution_result = execution_future_.get();
    if (execution_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("MoveArmNamedTargetNode"), "MoveIt execution succeeded");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "MoveIt execution failed. Error code: %d", execution_result.val);
    return BT::NodeStatus::FAILURE;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "Exception while getting execution result: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void MoveArmNamedTargetNode::onHalted() {
  RCLCPP_WARN(rclcpp::get_logger("MoveArmNamedTargetNode"), "BT node halted. Stopping MoveIt execution.");
  if (move_group_) {
    move_group_->stop();
    move_group_->clearPoseTargets();
  }
}