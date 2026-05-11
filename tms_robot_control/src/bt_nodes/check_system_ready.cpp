#include "tms_robot_control/bt_nodes/check_system_ready.hpp"
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

CheckSystemReadyNode::CheckSystemReadyNode(const std::string & name, const BT::NodeConfig & config) : BT::ConditionNode(name, config) {
}

BT::PortsList CheckSystemReadyNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("target", std::string(""), "Optional SRDF named target to check")
  };
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
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  const std::string target_value = target ? target.value() : "";
  if (!moveit_context->checkSystemReady(group.value(), target_value, error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckSystemReadyNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}