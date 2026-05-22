#include "tms_robot_control/bt_nodes/sensor_watch.hpp"

#include <algorithm>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

SensorWatchNode::SensorWatchNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList SensorWatchNode::providedPorts() {
  return {
    BT::InputPort<double>("duration_sec", 10.0, "Duration to watch sensors"),
    BT::InputPort<double>("publish_period_sec", 1.0, "Status publish/log period"),
    BT::InputPort<bool>("watch_force", true, "Watch FTS force data"),
    BT::InputPort<bool>("watch_distance", true, "Watch UC4 distance data"),
    BT::InputPort<bool>("watch_target_pose", true, "Watch camera TCP target pose"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Maximum allowed force sample age"),
    BT::InputPort<double>("distance_freshness_sec", 0.20, "Maximum allowed distance sample age"),
    BT::InputPort<double>("target_pose_freshness_sec", 1.00, "Maximum allowed TCP target pose age"),
    BT::InputPort<std::string>("status_topic", std::string("treatment_status"), "Topic for UI diagnostic status")
  };
}

BT::NodeStatus SensorWatchNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto duration_sec = getInput<double>("duration_sec");
  auto publish_period_sec = getInput<double>("publish_period_sec");
  auto watch_force = getInput<bool>("watch_force");
  auto watch_distance = getInput<bool>("watch_distance");
  auto watch_target_pose = getInput<bool>("watch_target_pose");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto distance_freshness_sec = getInput<double>("distance_freshness_sec");
  auto target_pose_freshness_sec = getInput<double>("target_pose_freshness_sec");
  auto status_topic = getInput<std::string>("status_topic");
  if (!duration_sec || !publish_period_sec ||
      !watch_force || !watch_distance || !watch_target_pose ||
      !force_freshness_sec || !distance_freshness_sec ||
      !target_pose_freshness_sec || !status_topic) {
    RCLCPP_ERROR(rclcpp::get_logger("SensorWatchNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  duration_sec_ = std::clamp(duration_sec.value(), 1.0, 300.0);
  publish_period_sec_ = std::clamp(publish_period_sec.value(), 0.1, 10.0);
  watch_force_ = watch_force.value();
  watch_distance_ = watch_distance.value();
  watch_target_pose_ = watch_target_pose.value();
  force_freshness_sec_ = force_freshness_sec.value();
  distance_freshness_sec_ = distance_freshness_sec.value();
  target_pose_freshness_sec_ = target_pose_freshness_sec.value();
  status_topic_ = status_topic.value();
  auto ros_node = get_ros_node_from_blackboard(config());
  status_pub_ = ros_node->create_publisher<std_msgs::msg::String>(status_topic_, 10);
  start_time_ = std::chrono::steady_clock::now();
  last_publish_time_ = start_time_ - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(publish_period_sec_));
  RCLCPP_INFO(rclcpp::get_logger("SensorWatchNode"),
    "Starting sensor watch for %.1f sec, period %.1f sec",
    duration_sec_,
    publish_period_sec_);
  publishStatus("SensorWatch START");
  return BT::NodeStatus::RUNNING;
}

std::string SensorWatchNode::buildStatusString(double elapsed_sec) {
  auto ros_node = get_ros_node_from_blackboard(config());
  auto sensor_context = get_sensor_context_from_blackboard(config());
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << "SensorWatch "
     << elapsed_sec << "/" << duration_sec_ << "s";
  if (watch_force_) {
    if (!sensor_context->isForceFresh(force_freshness_sec_)) {
      ss << " | Fz=STALE";
    } 
    else {
      double force_z = 0.0;
      rclcpp::Time stamp;
      if (sensor_context->getLatestForceZ(force_z, stamp)) {
        ss << " | Fz=" << force_z << " N";
      } 
      else {
        ss << " | Fz=NO_SAMPLE";
      }
    }
  }
  if (watch_distance_) {
    if (!sensor_context->isDistanceFresh(distance_freshness_sec_)) {
      ss << " | D=STALE";
    } 
    else {
      double distance_m = 0.0;
      rclcpp::Time stamp;
      if (sensor_context->getLatestDistance(distance_m, stamp)) {
        ss << " | D=" << distance_m * 1000.0 << " mm";
      } 
      else {
        ss << " | D=NO_SAMPLE";
      }
    }
  }
  if (watch_target_pose_) {
    if (!sensor_context->isTcpTargetPoseFresh(target_pose_freshness_sec_)) {
      ss << " | Pose=STALE";
    } 
    else {
      geometry_msgs::msg::PoseStamped pose;
      rclcpp::Time stamp;
      if (sensor_context->getLatestTcpTargetPose(pose, stamp)) {
        const double pose_age_sec = (ros_node->now() - stamp).seconds();
        ss << " | PoseFrame=" << pose.header.frame_id
           << " | PoseAge=" << pose_age_sec << "s"
           << " | PoseXYZ=["
           << pose.pose.position.x << ","
           << pose.pose.position.y << ","
           << pose.pose.position.z << "]";
      } 
      else {
        ss << " | Pose=NO_SAMPLE";
      }
    }
  }
  return ss.str();
}

void SensorWatchNode::publishStatus(const std::string & status) {
  if (status_pub_) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }
  RCLCPP_INFO(rclcpp::get_logger("SensorWatchNode"), "%s", status.c_str());
}

BT::NodeStatus SensorWatchNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    publishStatus("SensorWatch CANCELLED");
    return BT::NodeStatus::FAILURE;
  }
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec = std::chrono::duration<double>(now - start_time_).count();
  if (elapsed_sec >= duration_sec_) {
    publishStatus("SensorWatch DONE");
    return BT::NodeStatus::SUCCESS;
  }
  const double since_last_publish_sec = std::chrono::duration<double>(now - last_publish_time_).count();
  if (since_last_publish_sec >= publish_period_sec_) {
    last_publish_time_ = now;
    const auto status = buildStatusString(elapsed_sec);
    publishStatus(status);
  }
  return BT::NodeStatus::RUNNING;
}

void SensorWatchNode::onHalted() {
  publishStatus("SensorWatch HALTED");
}