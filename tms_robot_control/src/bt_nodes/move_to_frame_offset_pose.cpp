#include "tms_robot_control/bt_nodes/move_to_frame_offset_pose.hpp"

#include "tms_robot_control/bt_nodes/bt_utils.hpp"

MoveToFrameOffsetPoseNode::MoveToFrameOffsetPoseNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList MoveToFrameOffsetPoseNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP/end-effector link to command"),
    BT::InputPort<std::string>("reference_frame", std::string("dummy_head"), "Frame in which the target pose is expressed"),
    BT::InputPort<double>("x", 0.0, "Target x position in reference frame"), 
    BT::InputPort<double>("y", 0.0, "Target y position in reference frame"), 
    BT::InputPort<double>("z", 0.5, "Target z position in reference frame"), 
    BT::InputPort<double>("roll", 0.0, "Target roll in radians"), 
    BT::InputPort<double>("pitch", 0.0, "Target pitch in radians"),
    BT::InputPort<double>("yaw", 0.0, "Target yaw in radians"),
    BT::InputPort<double>("velocity_scale", 0.10, "MoveIt max velocity scaling factor"),
    BT::InputPort<double>("acceleration_scale", 0.10, "MoveIt max acceleration scaling factor")
  };
}

BT::NodeStatus MoveToFrameOffsetPoseNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveToFrameOffsetPoseNode"), "Cancel requested before pose motion start");
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto reference_frame = getInput<std::string>("reference_frame");
  auto x = getInput<double>("x");
  auto y = getInput<double>("y");
  auto z = getInput<double>("z");
  auto roll = getInput<double>("roll");
  auto pitch = getInput<double>("pitch");
  auto yaw = getInput<double>("yaw");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  if (!planning_group || !tcp_link || !reference_frame ||
      !x || !y || !z ||
      !roll || !pitch || !yaw ||
      !velocity_scale || !acceleration_scale)
  {RCLCPP_ERROR(rclcpp::get_logger("MoveToFrameOffsetPoseNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  reference_frame_ = reference_frame.value();
  x_ = x.value();
  y_ = y.value();
  z_ = z.value();
  roll_ = roll.value();
  pitch_ = pitch.value();
  yaw_ = yaw.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveToFrameOffsetPose(
      planning_group_,
      tcp_link_,
      reference_frame_,
      x_,
      y_,
      z_,
      roll_,
      pitch_,
      yaw_,
      velocity_scale_,
      acceleration_scale_,
      error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToFrameOffsetPoseNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToFrameOffsetPoseNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveToFrameOffsetPoseNode"), "Cancel requested. Stopping MoveIt execution.");
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
  RCLCPP_ERROR(rclcpp::get_logger("MoveToFrameOffsetPoseNode"), "%s", error_msg.c_str());
  return BT::NodeStatus::FAILURE;
}

void MoveToFrameOffsetPoseNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}