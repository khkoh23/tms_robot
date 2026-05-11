#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

class VerifyNamedTargetReachedNode : public BT::ConditionNode {
public:
  VerifyNamedTargetReachedNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};