#include "tms_robot_control/bt_nodes/move_tcp_relative_z.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

MoveTcpRelativeZNode::MoveTcpRelativeZNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList MoveTcpRelativeZNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP/end-effector link to move"),
    BT::InputPort<double>("distance", 0.0, "Relative TCP Z distance in meters. Positive means +TCP Z"),
    BT::InputPort<double>("velocity_scale", 0.02, "MoveIt max velocity scaling factor"),
    BT::InputPort<double>("acceleration_scale", 0.02, "MoveIt max acceleration scaling factor"), 
    BT::InputPort<double>("eef_step", 0.0001, "Cartesian path interpolation step in meters"),
    BT::InputPort<double>("min_fraction", 0.90, "Minimum required Cartesian path fraction"),
    BT::InputPort<double>("max_abs_distance", 0.05, "Reject relative motion larger than this absolute distance in meters")
  };
}

BT::NodeStatus MoveTcpRelativeZNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveTcpRelativeZNode"), "Cancel requested before TCP relative Z motion start");
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto distance = getInput<double>("distance");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  auto eef_step = getInput<double>("eef_step");
  auto min_fraction = getInput<double>("min_fraction");
  auto max_abs_distance = getInput<double>("max_abs_distance");
  if (!planning_group || !tcp_link || !distance || 
    !velocity_scale || !acceleration_scale ||
    !eef_step || !min_fraction || !max_abs_distance) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveTcpRelativeZNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  distance_m_ = distance.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  eef_step_ = eef_step.value();
  min_fraction_ = min_fraction.value();
  max_abs_distance_m_ = max_abs_distance.value();
  if (std::abs(distance_m_) > max_abs_distance_m_) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveTcpRelativeZNode"), 
      "Requested TCP Z distance %.6f m exceeds max_abs_distance %.6f m",
      distance_m_,
      max_abs_distance_m_);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("MoveTcpRelativeZNode"),
    "Starting TCP relative Z motion: distance=%.6f m",
    distance_m_);
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_, 
    tcp_link_,
    distance_m_,
    velocity_scale_,
    acceleration_scale_,
    eef_step_,
    min_fraction_,
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveTcpRelativeZNode"), "%s", error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTcpRelativeZNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("MoveTcpRelativeZNode"), "Cancel requested. Stopping TCP relative Z motion.");
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
  RCLCPP_ERROR(rclcpp::get_logger("MoveTcpRelativeZNode"), "%s", error_msg.c_str());
  return BT::NodeStatus::FAILURE;
}

void MoveTcpRelativeZNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}