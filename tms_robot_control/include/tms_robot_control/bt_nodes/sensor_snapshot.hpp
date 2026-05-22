#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

class SensorSnapshotNode : public BT::SyncActionNode {
public:
  SensorSnapshotNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};