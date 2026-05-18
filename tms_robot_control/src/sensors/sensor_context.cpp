#include "tms_robot_control/sensors/sensor_context.hpp"

#include <utility>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SensorContext::SensorContext(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  tf_buffer_(node_->get_clock()),
  last_force_receive_time_(0, 0, node_->get_clock()->get_clock_type()),
  last_distance_receive_time_(0, 0, node_->get_clock()->get_clock_type()),
  last_tcp_target_pose_receive_time_(0, 0, node_->get_clock()->get_clock_type()) {
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, node_, false);
  tf_buffer_.setUsingDedicatedThread(true);
  force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "robotiq_force_torque_sensor_broadcaster/wrench",
      rclcpp::SensorDataQoS(),
      std::bind(&SensorContext::forceCallback,
        this,
        std::placeholders::_1));
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "distance",
      rclcpp::QoS(10),
      std::bind(&SensorContext::distanceCallback,
        this,
        std::placeholders::_1));
  tcp_target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/camera/tcp_target_pose",
      rclcpp::QoS(10),
      std::bind(&SensorContext::tcpTargetPoseCallback,
        this,
        std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "SensorContext initialized: force, distance, tcp target pose, and TF listener active");
}

void SensorContext::forceCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_force_ = msg;
  last_force_receive_time_ = node_->now();
}

void SensorContext::distanceCallback(std_msgs::msg::Float32::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_distance_ = msg;
  last_distance_receive_time_ = node_->now();
}

void SensorContext::tcpTargetPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_tcp_target_pose_ = msg;
  last_tcp_target_pose_receive_time_ = node_->now();
}

bool SensorContext::getLatestForceZ(double & force_z, rclcpp::Time & stamp) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_force_) {
    return false;
  }
  force_z = last_force_->wrench.force.z;
  stamp = last_force_receive_time_;
  return true;
}

bool SensorContext::getLatestDistance(double & distance_m, rclcpp::Time & stamp) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_distance_) {
    return false;
  }
  distance_m = static_cast<double>(last_distance_->data);
  stamp = last_distance_receive_time_;
  return true;
}

bool SensorContext::getLatestTcpTargetPose(geometry_msgs::msg::PoseStamped & pose, rclcpp::Time & stamp) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_tcp_target_pose_) {
    return false;
  }
  pose = *last_tcp_target_pose_;
  stamp = last_tcp_target_pose_receive_time_;
  return true;
}

bool SensorContext::getLatestTcpTargetPoseInFrame(const std::string & target_frame, 
  geometry_msgs::msg::PoseStamped & pose_out,
  rclcpp::Time & stamp,
  double transform_timeout_sec,
  std::string & error_msg) {
  geometry_msgs::msg::PoseStamped pose_in;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_tcp_target_pose_) {
      error_msg = "No TCP target pose has been received yet";
      return false;
    }
    pose_in = *last_tcp_target_pose_;
    stamp = last_tcp_target_pose_receive_time_;
  }
  if (pose_in.header.frame_id.empty()) {
    error_msg = "TCP target pose has empty frame_id";
    return false;
  }
  if (target_frame.empty() || pose_in.header.frame_id == target_frame) {
    pose_out = pose_in;
    return true;
  }
  try {
    pose_out = tf_buffer_.transform(pose_in,
        target_frame,
        tf2::durationFromSec(transform_timeout_sec));
    return true;
  }
  catch (const tf2::TransformException & ex) {
    error_msg = "Failed to transform TCP target pose from frame '" +
      pose_in.header.frame_id +
      "' to frame '" +
      target_frame +
      "': " +
      ex.what();
    return false;
  }
}

bool SensorContext::isForceFresh(double timeout_sec) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_force_) {
    return false;
  }
  return (node_->now() - last_force_receive_time_).seconds() <= timeout_sec;
}

bool SensorContext::isDistanceFresh(double timeout_sec) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_distance_) {
    return false;
  }
  return (node_->now() - last_distance_receive_time_).seconds() <= timeout_sec;
}

bool SensorContext::isTcpTargetPoseFresh(double timeout_sec) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_tcp_target_pose_) {
    return false;
  }
  return (node_->now() - last_tcp_target_pose_receive_time_).seconds() <= timeout_sec;
}