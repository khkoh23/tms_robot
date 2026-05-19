#pragma once

#include <chrono>
#include <string>
#include <behaviortree_cpp/action_node.h>

class ApproachTcpZForceBandNode : public BT::StatefulActionNode {
public:
  ApproachTcpZForceBandNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class StepState {
    IDLE, MOVING
  };
  std::string planning_group_;
  std::string tcp_link_;
  double min_force_z_{-8.0};
  double max_force_z_{-3.0};
  double hard_min_force_z_{-10.0};
  double step_distance_{0.0005};
  double max_total_advance_{0.050};
  double force_freshness_sec_{0.20};
  double force_wait_timeout_sec_{5.0};
  double velocity_scale_{0.01};
  double acceleration_scale_{0.01};
  double eef_step_{0.0001};
  double min_fraction_{0.90};
  double total_motion_abs_{0.0};
  int step_count_{0};
  StepState step_state_{StepState::IDLE};
  std::chrono::steady_clock::time_point start_time_;
};