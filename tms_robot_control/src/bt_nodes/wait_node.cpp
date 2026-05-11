#include "tms_robot_control/bt_nodes/wait_node.hpp"
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

WaitNode::WaitNode(const std::string & name, const BT::NodeConfig & config) : BT::StatefulActionNode(name, config) {
}

BT::PortsList WaitNode::providedPorts() {
  return {
    BT::InputPort<int>("msec", 1000, "Duration to wait in milliseconds")
  };
}

BT::NodeStatus WaitNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto msec = getInput<int>("msec");
  if (msec) {
    duration_ms_ = msec.value();
  }
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed >= duration_ms_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitNode::onHalted() {
}