#pragma once

#include <chrono>
#include <string>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TreatmentForceBandHoldNode : public BT::StatefulActionNode {
public:
  TreatmentForceBandHoldNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class InternalState { MONITORING, MOVING_STEP, RETRACTING };
  enum class RetractOutcome { SUCCESS_AFTER_DURATION,
    FAILURE_AFTER_HARD_LIMIT,
    FAILURE_AFTER_DISTANCE,
    FAILURE_AFTER_ERROR
  };
  bool startTcpStep(double step);
  bool startRecoveryRetract(RetractOutcome outcome);
  BT::NodeStatus pollActiveMotion();
  bool checkDistanceGuard(bool & violated);
  void publishTreatmentStatus(const std::string & status);
  void publishFinalSummary(const std::string & exit_reason);
  void publishFinalUiStatus(const std::string & exit_reason);
  std::chrono::steady_clock::time_point start_time_;
  InternalState state_{InternalState::MONITORING};
  RetractOutcome retract_outcome_{RetractOutcome::SUCCESS_AFTER_DURATION};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr treatment_status_pub_;
  std::string planning_group_{"ur_arm"};
  std::string tcp_link_{"ur10e_tcp"};
  std::string duration_key_{"treatment_duration_sec"};
  std::string min_distance_key_{"min_distance_m"};
  double duration_sec_{10.0};
  double min_force_z_{-8.0};
  double max_force_z_{-3.0};
  double hard_min_force_z_{-10.0};
  double step_distance_{0.0005};
  double retract_distance_{-0.050};
  double max_total_adjustment_{0.050};
  bool enable_distance_guard_{true};
  double min_distance_m_{0.063};
  double distance_freshness_sec_{0.20};
  double force_freshness_sec_{0.20};
  double velocity_scale_{0.01};
  double acceleration_scale_{0.01};
  double eef_step_{0.0001};
  double min_fraction_{0.90};
  double total_adjustment_abs_{0.0};
  int step_count_{0};
  bool recovery_started_{false};
  std::chrono::steady_clock::time_point recovery_start_time_;
};