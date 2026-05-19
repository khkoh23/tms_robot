#include "tms_robot_control/bt_nodes/approach_tcp_z_force_band.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

ApproachTcpZForceBandNode::ApproachTcpZForceBandNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList ApproachTcpZForceBandNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP/end-effector link to move"),
    BT::InputPort<double>("min_force_z", -8.0, "Lower force band bound in N"), 
    BT::InputPort<double>("max_force_z", -3.0, "Upper force band bound in N"), 
    BT::InputPort<double>("hard_min_force_z", -10.0, "Hard minimum force safety limit in N"),
    BT::InputPort<double>("step_distance", 0.0005, "TCP Z step distance in meters"), 
    BT::InputPort<double>("max_total_advance", 0.050, "Maximum accumulated absolute TCP Z travel in meters"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Maximum allowed age of force sample in seconds"),
    BT::InputPort<double>("force_wait_timeout_sec", 5.0, "Maximum time to wait for fresh force data"),
    BT::InputPort<double>("velocity_scale", 0.01, "MoveIt max velocity scaling factor"),
    BT::InputPort<double>("acceleration_scale", 0.01, "MoveIt max acceleration scaling factor"),
    BT::InputPort<double>("eef_step", 0.0001, "Cartesian path interpolation step in meters"),
    BT::InputPort<double>("min_fraction", 0.90, "Minimum Cartesian path fraction")
  };
}

BT::NodeStatus ApproachTcpZForceBandNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto min_force_z = getInput<double>("min_force_z");
  auto max_force_z = getInput<double>("max_force_z");
  auto hard_min_force_z = getInput<double>("hard_min_force_z");
  auto step_distance = getInput<double>("step_distance");
  auto max_total_advance = getInput<double>("max_total_advance");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto force_wait_timeout_sec = getInput<double>("force_wait_timeout_sec");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  auto eef_step = getInput<double>("eef_step");
  auto min_fraction = getInput<double>("min_fraction");
  if (!planning_group || !tcp_link ||
    !min_force_z || !max_force_z || !hard_min_force_z ||
    !step_distance || !max_total_advance ||
    !force_freshness_sec || !force_wait_timeout_sec ||
    !velocity_scale || !acceleration_scale ||
    !eef_step || !min_fraction) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  min_force_z_ = min_force_z.value();
  max_force_z_ = max_force_z.value();
  hard_min_force_z_ = hard_min_force_z.value();
  step_distance_ = step_distance.value();
  max_total_advance_ = max_total_advance.value();
  force_freshness_sec_ = force_freshness_sec.value();
  force_wait_timeout_sec_ = force_wait_timeout_sec.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  eef_step_ = eef_step.value();
  min_fraction_ = min_fraction.value();
  total_motion_abs_ = 0.0;
  step_count_ = 0;
  step_state_ = StepState::IDLE;
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("ApproachTcpZForceBandNode"),
    "Starting force-band TCP Z approach. Band=[%.2f, %.2f] N, hard_min=%.2f N, step=%.6f m, max_total=%.3f m",
    min_force_z_,
    max_force_z_,
    hard_min_force_z_,
    step_distance_,
    max_total_advance_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachTcpZForceBandNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("ApproachTcpZForceBandNode"), "Cancel requested. Stopping force-band approach.");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    return BT::NodeStatus::FAILURE;
  }
  auto moveit_context = get_moveit_context_from_blackboard(config());
  // If a previous TCP step is still executing, poll it first.
  if (step_state_ == StepState::MOVING) {
    std::string error_msg;
    const auto motion_status = moveit_context->pollMotionResult(error_msg);
    if (motion_status == MoveItContext::MotionStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    if (motion_status == MoveItContext::MotionStatus::FAILURE) {
      RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
        "TCP Z force approach step failed: %s",
        error_msg.c_str());
      step_state_ = StepState::IDLE;
      return BT::NodeStatus::FAILURE;
    }
    step_state_ = StepState::IDLE;
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  const auto elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (!sensor_context->isForceFresh(force_freshness_sec_)) {
    if (elapsed_sec > force_wait_timeout_sec_) {
      RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
        "Timed out waiting for fresh force data after %.2f sec",
        force_wait_timeout_sec_);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      *get_ros_node_from_blackboard(config())->get_clock(),
      1000,
      "Waiting for fresh force data...");
    return BT::NodeStatus::RUNNING;
  }
  double force_z = 0.0;
  rclcpp::Time stamp;
  if (!sensor_context->getLatestForceZ(force_z, stamp)) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      *get_ros_node_from_blackboard(config())->get_clock(),
      1000,
      "No force sample available yet");
    return BT::NodeStatus::RUNNING;
  }
  if (force_z < hard_min_force_z_) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Hard force limit exceeded: force_z=%.3f N < %.3f N",
      force_z,
      hard_min_force_z_);
    return BT::NodeStatus::FAILURE;
  }
  if (force_z >= min_force_z_ && force_z <= max_force_z_) {
    RCLCPP_INFO(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Force band reached: force_z=%.3f N within [%.3f, %.3f] N after %d steps, total_abs_motion=%.6f m",
      force_z,
      min_force_z_,
      max_force_z_,
      step_count_,
      total_motion_abs_);
    return BT::NodeStatus::SUCCESS;
  }
  double step = 0.0;
  if (force_z > max_force_z_) {
    // Too light / insufficient contact.
    // Convention: +TCP Z increases contact force.
    step = std::abs(step_distance_);
  } 
  else {
    // Too much contact but not beyond hard limit.
    step = -std::abs(step_distance_);
  }
  if (total_motion_abs_ + std::abs(step) > max_total_advance_) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Maximum TCP Z approach travel exceeded. total=%.6f m, next=%.6f m, max=%.6f m, force_z=%.3f N",
      total_motion_abs_,
      std::abs(step),
      max_total_advance_,
      force_z);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("ApproachTcpZForceBandNode"),
    "Force_z=%.3f N outside band [%.3f, %.3f]. Commanding TCP Z step %.6f m",
    force_z,
    min_force_z_,
    max_force_z_,
    step);
  std::string error_msg;
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_,
    tcp_link_,
    step,
    velocity_scale_,
    acceleration_scale_,
    eef_step_,
    min_fraction_,
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Failed to start TCP Z step: %s",
      error_msg.c_str());
    return BT::NodeStatus::FAILURE;
  }
  total_motion_abs_ += std::abs(step);
  ++step_count_;
  step_state_ = StepState::MOVING;
  return BT::NodeStatus::RUNNING;
}

void ApproachTcpZForceBandNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}