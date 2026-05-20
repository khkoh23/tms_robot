#include "tms_robot_control/bt_nodes/treatment_force_band_hold.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

TreatmentForceBandHoldNode::TreatmentForceBandHoldNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList TreatmentForceBandHoldNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP link to control"),
    BT::InputPort<std::string>("duration_key", std::string("treatment_duration_sec"), "Blackboard key containing treatment duration in seconds"),
    BT::InputPort<double>("min_force_z", -8.0, "Lower desired force band bound in N"),
    BT::InputPort<double>("max_force_z", -3.0, "Upper desired force band bound in N"),
    BT::InputPort<double>("hard_min_force_z", -10.0, "Hard lower force safety limit in N"),
    BT::InputPort<double>("step_distance", 0.0005, "TCP Z adjustment step in meters"),
    BT::InputPort<double>("retract_distance", -0.050, "TCP Z recovery retract distance in meters"),
    BT::InputPort<double>("max_total_adjustment", 0.050, "Maximum accumulated force-hold adjustment travel in meters"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Maximum allowed force sample age"),
    BT::InputPort<double>("velocity_scale", 0.01, "MoveIt max velocity scaling factor"),
    BT::InputPort<double>("acceleration_scale", 0.01, "MoveIt max acceleration scaling factor"),
    BT::InputPort<double>("eef_step", 0.0001, "Cartesian interpolation step in meters"),
    BT::InputPort<double>("min_fraction", 0.90, "Minimum Cartesian path fraction")
  };
}

BT::NodeStatus TreatmentForceBandHoldNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto duration_key = getInput<std::string>("duration_key");
  auto min_force_z = getInput<double>("min_force_z");
  auto max_force_z = getInput<double>("max_force_z");
  auto hard_min_force_z = getInput<double>("hard_min_force_z");
  auto step_distance = getInput<double>("step_distance");
  auto retract_distance = getInput<double>("retract_distance");
  auto max_total_adjustment = getInput<double>("max_total_adjustment");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  auto eef_step = getInput<double>("eef_step");
  auto min_fraction = getInput<double>("min_fraction");
  if (!planning_group || !tcp_link || !duration_key ||
      !min_force_z || !max_force_z || !hard_min_force_z ||
      !step_distance || !retract_distance || !max_total_adjustment ||
      !force_freshness_sec || !velocity_scale || !acceleration_scale ||
      !eef_step || !min_fraction) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  duration_key_ = duration_key.value();
  min_force_z_ = min_force_z.value();
  max_force_z_ = max_force_z.value();
  hard_min_force_z_ = hard_min_force_z.value();
  step_distance_ = step_distance.value();
  retract_distance_ = retract_distance.value();
  max_total_adjustment_ = max_total_adjustment.value();
  force_freshness_sec_ = force_freshness_sec.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  eef_step_ = eef_step.value();
  min_fraction_ = min_fraction.value();
  try {
    duration_sec_ = config().blackboard->get<double>(duration_key_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), 
      "Failed to read treatment duration from blackboard key '%s': %s",
      duration_key_.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
  duration_sec_ = std::clamp(duration_sec_, 10.0, 300.0);
  state_ = InternalState::MONITORING;
  retract_outcome_ = RetractOutcome::SUCCESS_AFTER_DURATION;
  total_adjustment_abs_ = 0.0;
  step_count_ = 0;
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("TreatmentForceBandHoldNode"),
    "Starting active treatment force hold for %.1f sec. Band=[%.2f, %.2f] N, hard_min=%.2f N",
    duration_sec_,
    min_force_z_,
    max_force_z_,
    hard_min_force_z_);
  return BT::NodeStatus::RUNNING;
}

bool TreatmentForceBandHoldNode::startTcpStep(double step) {
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_, 
    tcp_link_, 
    step, 
    velocity_scale_, 
    acceleration_scale_, 
    eef_step_, 
    min_fraction_, 
    true, 
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Failed to start treatment force adjustment step: %s",
      error_msg.c_str());
    return false;
  }
  total_adjustment_abs_ += std::abs(step);
  ++step_count_;
  state_ = InternalState::MOVING_STEP;
  return true;
}

bool TreatmentForceBandHoldNode::startRecoveryRetract(RetractOutcome outcome) {
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  retract_outcome_ = outcome;
  RCLCPP_WARN(rclcpp::get_logger("TreatmentForceBandHoldNode"),
    "Starting contact recovery retract: distance=%.6f m",
    retract_distance_);
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_,
    tcp_link_, 
    retract_distance_, 
    velocity_scale_, 
    acceleration_scale_, 
    eef_step_, 
    min_fraction_, 
    true, 
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Failed to start treatment recovery retract: %s",
      error_msg.c_str());
    return false;
  }
  state_ = InternalState::RETRACTING;
  return true;
}

BT::NodeStatus TreatmentForceBandHoldNode::pollActiveMotion() {
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  const auto motion_status = moveit_context->pollMotionResult(error_msg);
  if (motion_status == MoveItContext::MotionStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (motion_status == MoveItContext::MotionStatus::FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Motion during treatment hold failed: %s",
      error_msg.c_str());
    state_ = InternalState::MONITORING;
    return BT::NodeStatus::FAILURE;
  }
  if (state_ == InternalState::RETRACTING) {
    state_ = InternalState::MONITORING;
    if (retract_outcome_ == RetractOutcome::SUCCESS_AFTER_DURATION) {
      RCLCPP_INFO(rclcpp::get_logger("TreatmentForceBandHoldNode"),
        "Treatment recovery retract completed after duration reached");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Treatment recovery retract completed after safety failure");
    return BT::NodeStatus::FAILURE;
  }
  state_ = InternalState::MONITORING;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TreatmentForceBandHoldNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Cancel requested during treatment force hold");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    return BT::NodeStatus::FAILURE;
  }
  if (state_ == InternalState::MOVING_STEP || state_ == InternalState::RETRACTING) {
    return pollActiveMotion();
  }
  const double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed_sec >= duration_sec_) {
    return startRecoveryRetract(RetractOutcome::SUCCESS_AFTER_DURATION)
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  if (!sensor_context->isForceFresh(force_freshness_sec_)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Force data stale during treatment hold");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
  }
  double force_z = 0.0;
  rclcpp::Time stamp;
  if (!sensor_context->getLatestForceZ(force_z, stamp)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "No force sample during treatment hold");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
  }
  if (force_z < hard_min_force_z_) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Hard force limit exceeded during treatment hold: force_z=%.3f N < %.3f N",
      force_z,
      hard_min_force_z_);
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_HARD_LIMIT)
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
  }
  double step = 0.0;
  if (force_z < min_force_z_) {
    // Too much pressure, retract.
    step = -std::abs(step_distance_);
  } 
  else if (force_z > max_force_z_) {
    // Too little pressure, advance.
    step = std::abs(step_distance_);
  } 
  else {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      *get_ros_node_from_blackboard(config())->get_clock(),
      2000,
      "Treatment hold force within band: force_z=%.3f N, elapsed=%.1f / %.1f sec",
      force_z,
      elapsed_sec,
      duration_sec_);
    return BT::NodeStatus::RUNNING;
  }
  if (total_adjustment_abs_ + std::abs(step) > max_total_adjustment_) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "Maximum treatment adjustment travel exceeded. total=%.6f, next=%.6f, max=%.6f, force_z=%.3f N",
      total_adjustment_abs_,
      std::abs(step),
      max_total_adjustment_,
      force_z);
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("TreatmentForceBandHoldNode"),
    "Treatment force adjustment: force_z=%.3f N outside band [%.3f, %.3f], step=%.6f m",
    force_z,
    min_force_z_,
    max_force_z_,
    step);
  return startTcpStep(step) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

void TreatmentForceBandHoldNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
  RCLCPP_WARN(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Treatment force hold halted");
}