#include "tms_robot_control/bt_nodes/report_status.hpp"
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

ReportStatusNode::ReportStatusNode(
  const std::string & name,
  const BT::NodeConfig & config,
  std::function<void(const std::string &)> log_fn)
: BT::SyncActionNode(name, config),
  log_fn_(log_fn)
{}

BT::PortsList ReportStatusNode::providedPorts()
{
  return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus ReportStatusNode::tick()
{
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }

  auto msg = getInput<std::string>("message");
  if (msg) {
    log_fn_(msg.value());
  }

  return BT::NodeStatus::SUCCESS;
}