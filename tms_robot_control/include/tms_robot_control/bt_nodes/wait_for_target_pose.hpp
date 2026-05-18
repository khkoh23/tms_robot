#pragma once

#include <chrono>
#include <string>
#include <behaviortree_cpp/action_node.h>

class WaitForTargetPoseNode : public BT::StatefulActionNode {
public:
  WaitForTargetPoseNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{10.0};
  double freshness_sec_{1.0};
  double transform_timeout_sec_{0.2};
  std::string target_frame_{"ur10e_base_link"};
  std::string blackboard_key_{"tcp_target_pose"};
};