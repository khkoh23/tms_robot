#include "tms_robot_control/bt_nodes/move_to_tcp_target_pose_offset.hpp"

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

MoveToTcpTargetPoseOffsetNode::MoveToTcpTargetPoseOffsetNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList MoveToTcpTargetPoseOffsetNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP/end-effector link to command"),
    BT::InputPort<std::string>("pose_key", std::string("tcp_target_pose"), "Blackboard key containing target PoseStamped"),
    BT::InputPort<std::string>("offset_z_key", std::string("tcp_offset_z_m"), "Blackboard key containing TCP Z offset in meters"),
    BT::InputPort<std::string>("marker_topic", std::string("/debug/tcp_target_offset_marker"), "RViz marker topic for calculated TCP offset pose"),
    BT::InputPort<double>("velocity_scale", 0.05, "MoveIt max velocity scaling factor"), 
    BT::InputPort<double>("acceleration_scale", 0.05, "MoveIt max acceleration scaling factor"),
    BT::InputPort<double>("planning_time", 10.0, "MoveIt planning time in seconds"),
    BT::InputPort<int>("planning_attempts", 5, "MoveIt number of planning attempts"),
    BT::InputPort<double>("position_tolerance", 0.01, "Goal position tolerance in meters"),
    BT::InputPort<double>("orientation_tolerance", 0.10, "Goal orientation tolerance in radians")
  };
}

geometry_msgs::msg::PoseStamped MoveToTcpTargetPoseOffsetNode::applyLocalZOffset(const geometry_msgs::msg::PoseStamped & input_pose,
  double offset_z_m) const {
  tf2::Transform target_tf;
  tf2::fromMsg(input_pose.pose, target_tf);
  tf2::Transform offset_tf;
  offset_tf.setIdentity();
  // Reverse direction: apply offset along NEGATIVE local Z-axis.
  offset_tf.setOrigin(tf2::Vector3(0.0, 0.0, -offset_z_m));
  // Apply offset in the target pose's local coordinate frame:
  // T_result = T_target * T_offset
  const tf2::Transform result_tf = target_tf * offset_tf;
  geometry_msgs::msg::PoseStamped output_pose = input_pose;
  output_pose.pose.position.x = result_tf.getOrigin().x();
  output_pose.pose.position.y = result_tf.getOrigin().y();
  output_pose.pose.position.z = result_tf.getOrigin().z();
  output_pose.pose.orientation.x = result_tf.getRotation().x();
  output_pose.pose.orientation.y = result_tf.getRotation().y();
  output_pose.pose.orientation.z = result_tf.getRotation().z();
  output_pose.pose.orientation.w = result_tf.getRotation().w();
  return output_pose;
}

BT::NodeStatus MoveToTcpTargetPoseOffsetNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "Cancel requested before TCP target offset motion start");
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto pose_key = getInput<std::string>("pose_key");
  auto offset_z_key = getInput<std::string>("offset_z_key");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  auto planning_time = getInput<double>("planning_time");
  auto planning_attempts = getInput<int>("planning_attempts");
  auto position_tolerance = getInput<double>("position_tolerance");
  auto orientation_tolerance = getInput<double>("orientation_tolerance");
  auto marker_topic = getInput<std::string>("marker_topic");
  if (!planning_group || !tcp_link || !pose_key || !offset_z_key ||
      !marker_topic ||
      !velocity_scale || !acceleration_scale ||
      !planning_time || !planning_attempts ||
      !position_tolerance || !orientation_tolerance) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  pose_key_ = pose_key.value();
  offset_z_key_ = offset_z_key.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  planning_time_ = planning_time.value();
  planning_attempts_ = planning_attempts.value();
  position_tolerance_ = position_tolerance.value();
  orientation_tolerance_ = orientation_tolerance.value();
  marker_topic_ = marker_topic.value();
  geometry_msgs::msg::PoseStamped target_pose;
  try {
    target_pose = config().blackboard->get<geometry_msgs::msg::PoseStamped>(pose_key_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"),
      "Failed to read target pose from blackboard key '%s': %s",
      pose_key_.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
  try {
    offset_z_m_ = config().blackboard->get<double>(offset_z_key_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"),
      "Failed to read TCP Z offset from blackboard key '%s': %s",
      offset_z_key_.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
  if (target_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "Target pose has empty frame_id");
    return BT::NodeStatus::FAILURE;
  }
  if (!marker_pub_) {
    auto ros_node = get_ros_node_from_blackboard(config());
    marker_pub_ = ros_node->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 
      rclcpp::QoS(1).transient_local());
  }
  const auto offset_pose = applyLocalZOffset(target_pose, offset_z_m_);
  publishOffsetPoseMarker(offset_pose);
  RCLCPP_INFO(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"),
    "Moving to TCP target offset pose. frame='%s', offset_z=%.3f m",
    offset_pose.header.frame_id.c_str(),
    offset_z_m_);
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveToPoseStamped(planning_group_,
    tcp_link_,
    offset_pose,
    velocity_scale_,
    acceleration_scale_,
    planning_time_,
    planning_attempts_,
    position_tolerance_,
    orientation_tolerance_,
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToTcpTargetPoseOffsetNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "Cancel requested. Stopping MoveIt execution.");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    return BT::NodeStatus::FAILURE;
  }
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  const auto status = moveit_context->pollMotionResult(error_msg);
  if (status == MoveItContext::MotionStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (status == MoveItContext::MotionStatus::SUCCESS) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"), "%s", error_msg.c_str());
  return BT::NodeStatus::FAILURE;
}

void MoveToTcpTargetPoseOffsetNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}

void MoveToTcpTargetPoseOffsetNode::publishOffsetPoseMarker(const geometry_msgs::msg::PoseStamped & offset_pose) {
  if (!marker_pub_) {
    return;
  }
  auto ros_node = get_ros_node_from_blackboard(config());
  tf2::Transform pose_tf;
  tf2::fromMsg(offset_pose.pose, pose_tf);
  const double axis_length = 0.05;  // 10 cm visual axes
  const tf2::Vector3 origin = pose_tf.getOrigin();
  const tf2::Vector3 x_axis = origin + pose_tf.getBasis() * tf2::Vector3(axis_length, 0.0, 0.0);
  const tf2::Vector3 y_axis = origin + pose_tf.getBasis() * tf2::Vector3(0.0, axis_length, 0.0);
  const tf2::Vector3 z_axis = origin + pose_tf.getBasis() * tf2::Vector3(0.0, 0.0, axis_length);
  auto make_point = [](const tf2::Vector3 & v) {
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
  };
  auto make_color = [](float r, float g, float b) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = 1.0f;
    return c;
  };
  const auto red = make_color(1.0f, 0.0f, 0.0f);
  const auto green = make_color(0.0f, 1.0f, 0.0f);
  const auto blue = make_color(0.0f, 0.3f, 1.0f);
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = offset_pose.header.frame_id;
  marker.header.stamp = ros_node->now();
  marker.ns = "tcp_target_offset_axes";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.008;
  // X axis: red
  marker.points.push_back(make_point(origin));
  marker.points.push_back(make_point(x_axis));
  marker.colors.push_back(red);
  marker.colors.push_back(red);
  // Y axis: green
  marker.points.push_back(make_point(origin));
  marker.points.push_back(make_point(y_axis));
  marker.colors.push_back(green);
  marker.colors.push_back(green);
  // Z axis: blue
  marker.points.push_back(make_point(origin));
  marker.points.push_back(make_point(z_axis));
  marker.colors.push_back(blue);
  marker.colors.push_back(blue);
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  marker_pub_->publish(marker);
  RCLCPP_INFO(rclcpp::get_logger("MoveToTcpTargetPoseOffsetNode"),
    "Published TCP target offset axes marker in frame '%s' at [%.3f, %.3f, %.3f]",
    offset_pose.header.frame_id.c_str(),
    offset_pose.pose.position.x,
    offset_pose.pose.position.y,
    offset_pose.pose.position.z);
}