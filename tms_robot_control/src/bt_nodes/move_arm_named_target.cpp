#include "tms_robot_control/bt_nodes/move_arm_named_target.hpp"
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

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
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveToNamedTarget(planning_group_, target_name_, error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArmNamedTargetNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveArmNamedTargetNode"), "Cancel requested. Stopping MoveIt execution.");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    return BT::NodeStatus::FAILURE;
  }
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  const auto status = moveit_context->pollMotionResult(error_msg);
  if (status == MoveItContext::MotionStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (status == MoveItContext::MotionStatus::SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(rclcpp::get_logger("MoveArmNamedTargetNode"), "%s", error_msg.c_str());
  return BT::NodeStatus::FAILURE;
}

void MoveArmNamedTargetNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}