#include "tms_robot_control/bt_nodes/set_allowed_collision.hpp"

#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

SetAllowedCollisionNode::SetAllowedCollisionNode(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config) {
}

BT::PortsList SetAllowedCollisionNode::providedPorts() {
  return {
    BT::InputPort<std::string>("link1", std::string("iccoil"), "First collision link"),
    BT::InputPort<std::string>("link2", std::string("dummy_head"), "Second collision link"),
    BT::InputPort<bool>("allowed", false, "Whether collision should be allowed") 
  };
}

BT::NodeStatus SetAllowedCollisionNode::tick() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto link1 = getInput<std::string>("link1");
  auto link2 = getInput<std::string>("link2");
  auto allowed = getInput<bool>("allowed");
  if (!link1 || !link2 || !allowed) {
    RCLCPP_ERROR(rclcpp::get_logger("SetAllowedCollisionNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->setAllowedCollision(link1.value(), link2.value(), allowed.value(), error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("SetAllowedCollisionNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("SetAllowedCollisionNode"), "Set allowed collision: %s <-> %s = %s",
    link1.value().c_str(),
    link2.value().c_str(),
    allowed.value() ? "true" : "false");
  return BT::NodeStatus::SUCCESS;
}