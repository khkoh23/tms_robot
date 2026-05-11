#include "tms_robot_control/bt_nodes/verify_named_target_reached.hpp"
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

VerifyNamedTargetReachedNode::VerifyNamedTargetReachedNode(const std::string & name, const BT::NodeConfig & config) : BT::ConditionNode(name, config) {
}

BT::PortsList VerifyNamedTargetReachedNode::providedPorts() {
  return {
    BT::InputPort<std::string>("target", "SRDF named target to verify"),
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<double>("tolerance", 0.02, "Allowed joint error in radians")
  };
}

BT::NodeStatus VerifyNamedTargetReachedNode::tick() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Cancel requested before verification");
    return BT::NodeStatus::FAILURE;
  }
  auto target = getInput<std::string>("target");
  auto planning_group = getInput<std::string>("planning_group");
  auto tolerance = getInput<double>("tolerance");
  if (!target) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Missing required input port: target");
    return BT::NodeStatus::FAILURE;
  }
  if (!planning_group) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Missing required input port: planning_group");
    return BT::NodeStatus::FAILURE;
  }
  const double tolerance_value = tolerance ? tolerance.value() : 0.02;
  double max_error = 0.0;
  std::string worst_joint;
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->verifyNamedTargetReached(
      planning_group.value(),
      target.value(),
      tolerance_value,
      max_error,
      worst_joint,
      error_msg))
  {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("VerifyNamedTargetReachedNode"), "Named target '%s' verified. Max joint error %.6f on joint '%s'",
    target.value().c_str(),
    max_error,
    worst_joint.c_str());
  return BT::NodeStatus::SUCCESS;
}