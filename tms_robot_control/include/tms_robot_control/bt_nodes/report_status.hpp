#pragma once

#include <behaviortree_cpp/action_node.h>
#include <functional>
#include <string>

class ReportStatusNode : public BT::SyncActionNode {
public:
  ReportStatusNode(const std::string & name, const BT::NodeConfig & config, std::function<void(const std::string &)> log_fn);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::function<void(const std::string &)> log_fn_;
};