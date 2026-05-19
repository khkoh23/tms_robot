#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

class MoveTcpRelativeZNode : public BT::StatefulActionNode {
public:
  MoveTcpRelativeZNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string planning_group_;
  std::string tcp_link_;
  double distance_m_{0.0};
  double velocity_scale_{0.02};
  double acceleration_scale_{0.02};
  double eef_step_{0.0001};
  double min_fraction_{0.90};
  double max_abs_distance_m_{0.05};
};