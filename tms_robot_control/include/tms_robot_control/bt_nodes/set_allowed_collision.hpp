#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

class SetAllowedCollisionNode : public BT::SyncActionNode {
public:
  SetAllowedCollisionNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};