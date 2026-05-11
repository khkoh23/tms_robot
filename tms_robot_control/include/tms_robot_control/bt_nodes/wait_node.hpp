#pragma once

#include <chrono>
#include <string>
#include <behaviortree_cpp/action_node.h>

class WaitNode : public BT::StatefulActionNode {
public:
  WaitNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int duration_ms_{1000};
  std::chrono::steady_clock::time_point start_time_;
};