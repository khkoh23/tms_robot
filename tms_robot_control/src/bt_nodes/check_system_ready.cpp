#include "tms_robot_control/bt_nodes/check_system_ready.hpp"

CheckSystemReadyNode::CheckSystemReadyNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList CheckSystemReadyNode::providedPorts()
{
  return {};
}

BT::NodeStatus CheckSystemReadyNode::tick()
{
  // Placeholder for:
  // - controller checks
  // - safety checks
  // - robot state freshness
  return BT::NodeStatus::SUCCESS;
}