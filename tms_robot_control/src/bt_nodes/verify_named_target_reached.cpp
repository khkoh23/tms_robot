#include "tms_robot_control/bt_nodes/verify_named_target_reached.hpp"

#include <atomic>
#include <cmath>
#include <limits>
#include <sstream>
#include <moveit/robot_state/robot_state.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

VerifyNamedTargetReachedNode::VerifyNamedTargetReachedNode(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config) {
}

BT::PortsList VerifyNamedTargetReachedNode::providedPorts() {
  return {
    BT::InputPort<std::string>("target", "SRDF named target to verify"), 
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<double>("tolerance", 0.02, "Allowed joint error in radians")
  };
}

bool VerifyNamedTargetReachedNode::initializeMoveGroup() {
  if (initialized_) {
    return true;
  }
  try {
    auto ros_node = get_ros_node_from_blackboard(config());
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(ros_node, planning_group_);
    initialized_ = true;
    return true;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Failed to initialize MoveGroupInterface: %s", e.what());
    return false;
  }
}

BT::NodeStatus VerifyNamedTargetReachedNode::tick() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Cancel requested before verification"); 
    return BT::NodeStatus::FAILURE;
  }
  auto target = getInput<std::string>("target");
  auto group = getInput<std::string>("planning_group");
  auto tolerance = getInput<double>("tolerance");
  if (!target) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Missing required input port: target");
    return BT::NodeStatus::FAILURE;
  }
  if (!group) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Missing required input port: planning_group");
    return BT::NodeStatus::FAILURE;
  }
  target_name_ = target.value();
  planning_group_ = group.value();
  const double tolerance_value = tolerance ? tolerance.value() : 0.02;
  if (!initializeMoveGroup()) {
    return BT::NodeStatus::FAILURE;
  }
  try {
    const auto target_values = move_group_->getNamedTargetValues(target_name_);
    if (target_values.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Named target '%s' not found for group '%s'", target_name_.c_str(), planning_group_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    auto current_state = move_group_->getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Failed to get current robot state");
      return BT::NodeStatus::FAILURE;
    }
    double max_error = 0.0;
    std::string worst_joint;
    for (const auto & [joint_name, target_position] : target_values) {
      const double current_position = current_state->getVariablePosition(joint_name);
      const double error = std::abs(current_position - target_position);
      if (error > max_error) {
        max_error = error;
        worst_joint = joint_name;
      }
      if (error > tolerance_value) {
        RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Target verification failed. Joint '%s' error %.6f exceeds tolerance %.6f",
          joint_name.c_str(),
          error,
          tolerance_value);
        return BT::NodeStatus::FAILURE;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Named target '%s' verified. Max joint error %.6f on joint '%s'",
      target_name_.c_str(),
      max_error,
      worst_joint.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Exception during target verification: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}