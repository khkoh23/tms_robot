#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

class MoveToFrameOffsetPoseNode : public BT::StatefulActionNode {
public:
  MoveToFrameOffsetPoseNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string planning_group_;
  std::string tcp_link_;
  std::string reference_frame_;
  double x_{0.0};
  double y_{0.0};
  double z_{0.5};
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
  double velocity_scale_{0.10};
  double acceleration_scale_{0.10};
};