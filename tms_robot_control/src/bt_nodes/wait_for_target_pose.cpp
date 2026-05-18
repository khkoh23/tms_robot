#include "tms_robot_control/bt_nodes/wait_for_target_pose.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

WaitForTargetPoseNode::WaitForTargetPoseNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList WaitForTargetPoseNode::providedPorts() {
  return {
    BT::InputPort<double>("timeout_sec", 10.0, "Maximum time to wait for target pose"),
    BT::InputPort<double>("freshness_sec", 1.0, "Maximum allowed age of received target pose"),
    BT::InputPort<double>("transform_timeout_sec", 0.2, "TF transform timeout"),
    BT::InputPort<std::string>("target_frame", std::string("ur10e_base_link"), "Frame to transform target pose into"),
    BT::InputPort<std::string>("blackboard_key", std::string("tcp_target_pose"), "Blackboard key used to store transformed pose")
  };
}

BT::NodeStatus WaitForTargetPoseNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto timeout = getInput<double>("timeout_sec");
  auto freshness = getInput<double>("freshness_sec");
  auto transform_timeout = getInput<double>("transform_timeout_sec");
  auto target_frame = getInput<std::string>("target_frame");
  auto blackboard_key = getInput<std::string>("blackboard_key");
  if (!timeout || !freshness || !transform_timeout || !target_frame || !blackboard_key) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitForTargetPoseNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  timeout_sec_ = timeout.value();
  freshness_sec_ = freshness.value();
  transform_timeout_sec_ = transform_timeout.value();
  target_frame_ = target_frame.value();
  blackboard_key_ = blackboard_key.value();
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("WaitForTargetPoseNode"),
    "Waiting for TCP target pose. target_frame='%s', timeout=%.2f sec, freshness=%.2f sec",
    target_frame_.c_str(),
    timeout_sec_,
    freshness_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForTargetPoseNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed > timeout_sec_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitForTargetPoseNode"),
      "Timed out waiting for TCP target pose after %.2f sec",
      timeout_sec_);
    return BT::NodeStatus::FAILURE;
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  if (!sensor_context->isTcpTargetPoseFresh(freshness_sec_)) {
    return BT::NodeStatus::RUNNING;
  }
  geometry_msgs::msg::PoseStamped transformed_pose;
  rclcpp::Time receive_stamp;
  std::string error_msg;
  if (!sensor_context->getLatestTcpTargetPoseInFrame(
      target_frame_,
      transformed_pose,
      receive_stamp,
      transform_timeout_sec_,
      error_msg)) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("WaitForTargetPoseNode"),
      *get_ros_node_from_blackboard(config())->get_clock(),
      1000,
      "%s",
      error_msg.c_str());
    return BT::NodeStatus::RUNNING;
  }
  config().blackboard->set<geometry_msgs::msg::PoseStamped>(blackboard_key_, transformed_pose);
  RCLCPP_INFO(rclcpp::get_logger("WaitForTargetPoseNode"),
    "TCP target pose received and stored in blackboard key '%s' in frame '%s'",
    blackboard_key_.c_str(),
    transformed_pose.header.frame_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

void WaitForTargetPoseNode::onHalted() {
}