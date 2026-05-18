#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SensorContext {
public:
  explicit SensorContext(rclcpp::Node::SharedPtr node);
  bool getLatestForceZ(double & force_z, rclcpp::Time & stamp) const;
  bool getLatestDistance(double & distance_m, rclcpp::Time & stamp) const;
  bool getLatestTcpTargetPose(geometry_msgs::msg::PoseStamped & pose, rclcpp::Time & stamp) const;
  bool getLatestTcpTargetPoseInFrame(const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & pose_out,
    rclcpp::Time & stamp,
    double transform_timeout_sec,
    std::string & error_msg);
  bool isForceFresh(double timeout_sec) const;
  bool isDistanceFresh(double timeout_sec) const;
  bool isTcpTargetPoseFresh(double timeout_sec) const;

private:
  void forceCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void distanceCallback(std_msgs::msg::Float32::SharedPtr msg);
  void tcpTargetPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_target_pose_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::WrenchStamped::SharedPtr last_force_;
  std_msgs::msg::Float32::SharedPtr last_distance_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_tcp_target_pose_;
  rclcpp::Time last_force_receive_time_;
  rclcpp::Time last_distance_receive_time_;
  rclcpp::Time last_tcp_target_pose_receive_time_;
};