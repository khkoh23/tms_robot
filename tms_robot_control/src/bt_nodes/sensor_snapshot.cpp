#include "tms_robot_control/bt_nodes/sensor_snapshot.hpp"

#include <sstream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

SensorSnapshotNode::SensorSnapshotNode(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config) {
}

BT::PortsList SensorSnapshotNode::providedPorts() {
  return {
    BT::InputPort<bool>("require_force", true, "Require latest FTS force sample"),
    BT::InputPort<bool>("require_distance", true, "Require latest UC4 distance sample"),
    BT::InputPort<bool>("require_target_pose", true, "Require latest camera TCP target pose"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Maximum allowed force sample age"),
    BT::InputPort<double>("distance_freshness_sec", 0.20, "Maximum allowed distance sample age"),
    BT::InputPort<double>("target_pose_freshness_sec", 1.00, "Maximum allowed TCP target pose age"),
    BT::InputPort<std::string>("status_topic", std::string("treatment_status"), "Topic for UI diagnostic status")
  };
}

BT::NodeStatus SensorSnapshotNode::tick() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto require_force = getInput<bool>("require_force");
  auto require_distance = getInput<bool>("require_distance");
  auto require_target_pose = getInput<bool>("require_target_pose");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto distance_freshness_sec = getInput<double>("distance_freshness_sec");
  auto target_pose_freshness_sec = getInput<double>("target_pose_freshness_sec");
  auto status_topic = getInput<std::string>("status_topic");
  if (!require_force || !require_distance || !require_target_pose ||
      !force_freshness_sec || !distance_freshness_sec ||
      !target_pose_freshness_sec || !status_topic) {
    RCLCPP_ERROR(rclcpp::get_logger("SensorSnapshotNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  auto ros_node = get_ros_node_from_blackboard(config());
  auto sensor_context = get_sensor_context_from_blackboard(config());
  bool ok = true;
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << "Sensor snapshot:";
  if (require_force.value()) {
    if (!sensor_context->isForceFresh(force_freshness_sec.value())) {
      ss << " FTS=STALE";
      ok = false;
    } 
    else {
      double force_z = 0.0;
      rclcpp::Time stamp;
      if (sensor_context->getLatestForceZ(force_z, stamp)) {
        ss << " Fz=" << force_z << " N";
      } 
      else {
        ss << " FTS=NO_SAMPLE";
        ok = false;
      }
    }
  }
  if (require_distance.value()) {
    if (!sensor_context->isDistanceFresh(distance_freshness_sec.value())) {
      ss << " UC4=STALE";
      ok = false;
    } 
    else {
      double distance_m = 0.0;
      rclcpp::Time stamp;
      if (sensor_context->getLatestDistance(distance_m, stamp)) {
        ss << " D=" << distance_m * 1000.0 << " mm";
      } 
      else {
        ss << " UC4=NO_SAMPLE";
        ok = false;
      }
    }
  }
  if (require_target_pose.value()) {
    if (!sensor_context->isTcpTargetPoseFresh(target_pose_freshness_sec.value())) {
      ss << " Pose=STALE";
      ok = false;
    } 
    else {
      geometry_msgs::msg::PoseStamped pose;
      rclcpp::Time stamp;
      if (sensor_context->getLatestTcpTargetPose(pose, stamp)) {
        const double age_sec = (ros_node->now() - stamp).seconds();
        ss << " PoseFrame=" << pose.header.frame_id
           << " PoseAge=" << age_sec << "s"
           << " PoseXYZ=["
           << pose.pose.position.x << ", "
           << pose.pose.position.y << ", "
           << pose.pose.position.z << "]";
      } 
      else {
        ss << " Pose=NO_SAMPLE";
        ok = false;
      }
    }
  }
  const auto summary = ss.str();
  RCLCPP_INFO(rclcpp::get_logger("SensorSnapshotNode"), "%s", summary.c_str());
  auto pub = ros_node->create_publisher<std_msgs::msg::String>(status_topic.value(), 10);
  std_msgs::msg::String msg;
  msg.data = summary;
  pub->publish(msg);
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}