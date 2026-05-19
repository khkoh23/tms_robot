#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MoveToTcpTargetPoseOffsetNode : public BT::StatefulActionNode {
public:
  MoveToTcpTargetPoseOffsetNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  geometry_msgs::msg::PoseStamped applyLocalZOffset(const geometry_msgs::msg::PoseStamped & input_pose, double offset_z_m) const;
  void publishOffsetPoseMarker(const geometry_msgs::msg::PoseStamped & offset_pose);
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::string marker_topic_{"/debug/tcp_target_offset_marker"};
  std::string planning_group_;
  std::string tcp_link_;
  std::string pose_key_;
  std::string offset_z_key_;
  double offset_z_m_{0.10};
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  double planning_time_{10.0};
  int planning_attempts_{5};
  double position_tolerance_{0.01};
  double orientation_tolerance_{0.10};
};