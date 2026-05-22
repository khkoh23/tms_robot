#pragma once

#include <chrono>
#include <string>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SensorWatchNode : public BT::StatefulActionNode {
public:
  SensorWatchNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishStatus(const std::string & status);
  std::string buildStatusString(double elapsed_sec);
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  double duration_sec_{10.0};
  double publish_period_sec_{1.0};
  bool watch_force_{true};
  bool watch_distance_{true};
  bool watch_target_pose_{true};
  double force_freshness_sec_{0.20};
  double distance_freshness_sec_{0.20};
  double target_pose_freshness_sec_{1.00};
  std::string status_topic_{"treatment_status"};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};