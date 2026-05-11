#include "tms_robot_control/bt_nodes/check_system_ready.hpp"

#include <atomic>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

CheckSystemReadyNode::CheckSystemReadyNode(const std::string & name, const BT::NodeConfig & config) : BT::ConditionNode(name, config) {
}

BT::PortsList CheckSystemReadyNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("target", std::string(""), "Optional SRDF named target to check")
  };
}

bool CheckSystemReadyNode::initializeMoveGroup() {
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
    RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "Failed to initialize MoveGroupInterface: %s", e.what());
    return false;
  }
}

BT::NodeStatus CheckSystemReadyNode::tick() {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "ROS is not OK");
    return BT::NodeStatus::FAILURE;
  }
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("CheckSystemReadyNode"), "Cancel requested before system check");
    return BT::NodeStatus::FAILURE;
  }
  auto group = getInput<std::string>("planning_group");
  auto target = getInput<std::string>("target");
  if (!group) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "Missing planning_group input");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = group.value();
  if (!initializeMoveGroup()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto joint_names = move_group_->getJointNames();
  if (joint_names.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "Planning group '%s' has no joints", planning_group_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (target && !target.value().empty()) {
    const auto target_values = move_group_->getNamedTargetValues(target.value());
    if (target_values.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "Named target '%s' not found for group '%s'", target.value().c_str(), planning_group_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("CheckSystemReadyNode"), "System ready for planning group '%s'", planning_group_.c_str());
  return BT::NodeStatus::SUCCESS;
}