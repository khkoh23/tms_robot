#include "tms_robot_control/bt_nodes/treatment_force_band_hold.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

TreatmentForceBandHoldNode::TreatmentForceBandHoldNode(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config) {
}

BT::PortsList TreatmentForceBandHoldNode::providedPorts() {
  return {
    BT::InputPort<std::string>("planning_group", std::string("ur_arm"), "MoveIt planning group"),
    BT::InputPort<std::string>("tcp_link", std::string("ur10e_tcp"), "TCP link"),
    BT::InputPort<std::string>("duration_key", std::string("treatment_duration_sec"), "Duration blackboard key"),
    BT::InputPort<std::string>("min_distance_key", std::string("min_distance_m"), "Min distance blackboard key"),
    BT::InputPort<double>("min_force_z", -8.0, "Lower desired force band bound"),
    BT::InputPort<double>("max_force_z", -3.0, "Upper desired force band bound"),
    BT::InputPort<double>("hard_min_force_z", -10.0, "Hard lower force safety limit"),
    BT::InputPort<double>("step_distance", 0.0005, "TCP Z adjustment step"),
    BT::InputPort<double>("retract_distance", -0.050, "Recovery retract distance"),
    BT::InputPort<double>("max_total_adjustment", 0.050, "Maximum total adjustment travel"),
    BT::InputPort<bool>("enable_distance_guard", true, "Enable UC4 distance guard"),
    BT::InputPort<double>("distance_freshness_sec", 0.20, "Distance freshness timeout"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Force freshness timeout"),
    BT::InputPort<double>("velocity_scale", 0.01, "Velocity scaling"),
    BT::InputPort<double>("acceleration_scale", 0.01, "Acceleration scaling"),
    BT::InputPort<double>("eef_step", 0.0001, "Cartesian interpolation step"),
    BT::InputPort<double>("min_fraction", 0.90, "Minimum Cartesian path fraction")
  };
}

void TreatmentForceBandHoldNode::publishTreatmentStatus(const std::string & status) {
  if (!treatment_status_pub_) {
    auto ros_node = get_ros_node_from_blackboard(config());
    treatment_status_pub_ = ros_node->create_publisher<std_msgs::msg::String>("treatment_status", 10);
  }
  std_msgs::msg::String msg;
  msg.data = status;
  treatment_status_pub_->publish(msg);
}

BT::NodeStatus TreatmentForceBandHoldNode::onStart() {
  if (is_cancel_requested_from_blackboard(config())) {
    return BT::NodeStatus::FAILURE;
  }
  auto planning_group = getInput<std::string>("planning_group");
  auto tcp_link = getInput<std::string>("tcp_link");
  auto duration_key = getInput<std::string>("duration_key");
  auto min_distance_key = getInput<std::string>("min_distance_key");
  auto min_force_z = getInput<double>("min_force_z");
  auto max_force_z = getInput<double>("max_force_z");
  auto hard_min_force_z = getInput<double>("hard_min_force_z");
  auto step_distance = getInput<double>("step_distance");
  auto retract_distance = getInput<double>("retract_distance");
  auto max_total_adjustment = getInput<double>("max_total_adjustment");
  auto enable_distance_guard = getInput<bool>("enable_distance_guard");
  auto distance_freshness_sec = getInput<double>("distance_freshness_sec");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto velocity_scale = getInput<double>("velocity_scale");
  auto acceleration_scale = getInput<double>("acceleration_scale");
  auto eef_step = getInput<double>("eef_step");
  auto min_fraction = getInput<double>("min_fraction");
  if (!planning_group || !tcp_link || !duration_key || !min_distance_key || 
    !min_force_z || !max_force_z || !hard_min_force_z || 
    !step_distance || !retract_distance || !max_total_adjustment || 
    !enable_distance_guard || !distance_freshness_sec || 
    !force_freshness_sec || !velocity_scale || !acceleration_scale || 
    !eef_step || !min_fraction) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  planning_group_ = planning_group.value();
  tcp_link_ = tcp_link.value();
  duration_key_ = duration_key.value();
  min_distance_key_ = min_distance_key.value();
  min_force_z_ = min_force_z.value();
  max_force_z_ = max_force_z.value();
  hard_min_force_z_ = hard_min_force_z.value();
  step_distance_ = step_distance.value();
  retract_distance_ = retract_distance.value();
  max_total_adjustment_ = max_total_adjustment.value();
  enable_distance_guard_ = enable_distance_guard.value();
  distance_freshness_sec_ = distance_freshness_sec.value();
  force_freshness_sec_ = force_freshness_sec.value();
  velocity_scale_ = velocity_scale.value();
  acceleration_scale_ = acceleration_scale.value();
  eef_step_ = eef_step.value();
  min_fraction_ = min_fraction.value();
  try {
    duration_sec_ = config().blackboard->get<double>(duration_key_);
    min_distance_m_ = config().blackboard->get<double>(min_distance_key_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Failed to read blackboard parameter: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  duration_sec_ = std::clamp(duration_sec_, 10.0, 300.0);
  min_distance_m_ = std::clamp(min_distance_m_, 0.025, 0.150);
  state_ = InternalState::MONITORING;
  retract_outcome_ = RetractOutcome::SUCCESS_AFTER_DURATION;
  total_adjustment_abs_ = 0.0;
  step_count_ = 0;
  start_time_ = std::chrono::steady_clock::now();
  std::ostringstream ss;
  ss << "START duration=" << duration_sec_
     << "s distance_guard=" << (enable_distance_guard_ ? "ON" : "OFF")
     << " min_distance=" << min_distance_m_ * 1000.0 << "mm";
  publishTreatmentStatus(ss.str());
  RCLCPP_INFO(rclcpp::get_logger("TreatmentForceBandHoldNode"),
    "Starting active treatment force hold for %.1f sec. Band=[%.2f, %.2f] N, hard_min=%.2f N, distance_guard=%s, min_distance=%.1f mm",
    duration_sec_,
    min_force_z_,
    max_force_z_,
    hard_min_force_z_,
    enable_distance_guard_ ? "true" : "false",
    min_distance_m_ * 1000.0);
  return BT::NodeStatus::RUNNING;
}

bool TreatmentForceBandHoldNode::checkDistanceGuard(bool & violated) {
  violated = false;
  if (!enable_distance_guard_) {
    return true;
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  if (!sensor_context->isDistanceFresh(distance_freshness_sec_)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Distance data stale during treatment hold");
    violated = true;
    return false;
  }
  double distance_m = 0.0;
  rclcpp::Time stamp;
  if (!sensor_context->getLatestDistance(distance_m, stamp)) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "No distance sample during treatment hold");
    violated = true;
    return false;
  }
  if (distance_m < min_distance_m_) {
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"),
      "UC4 distance guard triggered during treatment hold: distance=%.1f mm < threshold=%.1f mm",
      distance_m * 1000.0,
      min_distance_m_ * 1000.0);
    violated = true;
    return false;
  }
  return true;
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
    RCLCPP_ERROR(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Failed to start treatment force adjustment step: %s", error_msg.c_str());
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
  publishTreatmentStatus("RECOVERY retracting from contact");
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
      publishTreatmentStatus("DONE treatment duration reached and retracted");
      return BT::NodeStatus::SUCCESS;
    }
    publishTreatmentStatus("FAILED safety recovery retract completed");
    return BT::NodeStatus::FAILURE;
  }
  state_ = InternalState::MONITORING;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TreatmentForceBandHoldNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Cancel requested during treatment force hold");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    publishTreatmentStatus("CANCEL requested");
    return BT::NodeStatus::FAILURE;
  }
  if (state_ == InternalState::MOVING_STEP ||
      state_ == InternalState::RETRACTING) {
    return pollActiveMotion();
  }
  const double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed_sec >= duration_sec_) {
    return startRecoveryRetract(RetractOutcome::SUCCESS_AFTER_DURATION) 
    ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  bool distance_violated = false;
  if (!checkDistanceGuard(distance_violated)) {
    if (distance_violated) {
      publishTreatmentStatus("DISTANCE VIOLATION recovery retract");
      return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_DISTANCE) 
      ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    }
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  if (!sensor_context->isForceFresh(force_freshness_sec_)) {
    publishTreatmentStatus("FORCE STALE recovery retract");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
    ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  double force_z = 0.0;
  rclcpp::Time stamp;
  if (!sensor_context->getLatestForceZ(force_z, stamp)) {
    publishTreatmentStatus("NO FORCE recovery retract");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
    ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  if (force_z < hard_min_force_z_) {
    publishTreatmentStatus("HARD FORCE LIMIT recovery retract");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_HARD_LIMIT)
    ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  double step = 0.0;
  std::string action = "HOLD";
  if (force_z < min_force_z_) {
    step = -std::abs(step_distance_);
    action = "RETRACT";
  } 
  else if (force_z > max_force_z_) {
    step = std::abs(step_distance_);
    action = "ADVANCE";
  }
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(1);
  ss << action
     << " | " << elapsed_sec << "/" << duration_sec_ << " sec"
     << " | Fz=" << force_z << " N";
  if (enable_distance_guard_) {
    double distance_m = 0.0;
    rclcpp::Time distance_stamp;
    if (sensor_context->getLatestDistance(distance_m, distance_stamp)) {
      ss << " | D=" << distance_m * 1000.0 << " mm";
    }
  }
  publishTreatmentStatus(ss.str());
  if (step == 0.0) {
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
    publishTreatmentStatus("MAX ADJUSTMENT recovery retract");
    return startRecoveryRetract(RetractOutcome::FAILURE_AFTER_ERROR)
    ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
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
  publishTreatmentStatus("HALTED");
  RCLCPP_WARN(rclcpp::get_logger("TreatmentForceBandHoldNode"), "Treatment force hold halted");
}