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
    BT::InputPort<double>("min_fraction", 0.90, "Minimum Cartesian path fraction"),
    BT::InputPort<bool>("avoid_collisions", true, "Whether Cartesian path computation should avoid collisions"), 
    BT::InputPort<bool>("enable_distance_guard", true, "Enable UC4 distance guard"), 
    BT::InputPort<std::string>("min_distance_key", std::string("min_distance_m"), "Blackboard key containing UC4 minimum distance in meters"), 
    BT::InputPort<double>("distance_freshness_sec", 0.20, "Maximum allowed age of UC4 distance sample in seconds"), 
    BT::InputPort<double>("recovery_retract_distance", -0.050, "TCP Z recovery retract distance after distance guard violation")
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
  auto avoid_collisions = getInput<bool>("avoid_collisions");
  auto enable_distance_guard = getInput<bool>("enable_distance_guard");
  auto min_distance_key = getInput<std::string>("min_distance_key");
  auto distance_freshness_sec = getInput<double>("distance_freshness_sec");
  auto recovery_retract_distance = getInput<double>("recovery_retract_distance");
  if (!planning_group || !tcp_link ||
    !min_force_z || !max_force_z || !hard_min_force_z ||
    !step_distance || !max_total_advance ||
    !force_freshness_sec || !force_wait_timeout_sec ||
    !velocity_scale || !acceleration_scale ||
    !eef_step || !min_fraction || !avoid_collisions ||
    !enable_distance_guard || !min_distance_key ||
    !distance_freshness_sec || !recovery_retract_distance) {
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
  avoid_collisions_ = avoid_collisions.value();
  enable_distance_guard_ = enable_distance_guard.value();
  min_distance_key_ = min_distance_key.value();
  distance_freshness_sec_ = distance_freshness_sec.value();
  recovery_retract_distance_ = recovery_retract_distance.value();
  try {
    min_distance_m_ = config().blackboard->get<double>(min_distance_key_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Failed to read min distance from blackboard key '%s': %s",
      min_distance_key_.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
  total_motion_abs_ = 0.0;
  step_count_ = 0;
  step_state_ = StepState::IDLE;
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("ApproachTcpZForceBandNode"),
    "Starting force-band TCP Z approach. Band=[%.2f, %.2f] N, hard_min=%.2f N, step=%.6f m, max_total=%.3f m, avoid_collisions=%s, distance_guard=%s, min_distance=%.1f mm",
    min_force_z_,
    max_force_z_,
    hard_min_force_z_,
    step_distance_,
    max_total_advance_,
    avoid_collisions_ ? "true" : "false",
    enable_distance_guard_ ? "true" : "false",
    min_distance_m_ * 1000.0);
  return BT::NodeStatus::RUNNING;
}

bool ApproachTcpZForceBandNode::startTcpStep(double step) {
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_,
    tcp_link_,
    step,
    velocity_scale_,
    acceleration_scale_,
    eef_step_,
    min_fraction_,
    avoid_collisions_,
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Failed to start TCP Z step: %s",
      error_msg.c_str());
    return false;
  }
  total_motion_abs_ += std::abs(step);
  ++step_count_;
  step_state_ = StepState::MOVING;
  return true;
}

bool ApproachTcpZForceBandNode::startRecoveryRetract(const std::string & reason) {
  RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
    "Starting recovery retract during approach. Reason: %s",
    reason.c_str());
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  if (!moveit_context->startMoveTcpRelativeZ(planning_group_,
    tcp_link_,
    recovery_retract_distance_,
    velocity_scale_,
    acceleration_scale_,
    eef_step_,
    min_fraction_,
    true,
    error_msg)) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Failed to start approach recovery retract: %s",
      error_msg.c_str());
    return false;
  }
  step_state_ = StepState::RECOVERY_RETRACTING;
  return true;
}

BT::NodeStatus ApproachTcpZForceBandNode::pollActiveMotion() {
  std::string error_msg;
  auto moveit_context = get_moveit_context_from_blackboard(config());
  const auto motion_status = moveit_context->pollMotionResult(error_msg);
  if (motion_status == MoveItContext::MotionStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (motion_status == MoveItContext::MotionStatus::FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "TCP Z approach motion failed: %s",
      error_msg.c_str());
    step_state_ = StepState::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  if (step_state_ == StepState::RECOVERY_RETRACTING) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Approach recovery retract completed. Returning FAILURE.");
    step_state_ = StepState::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  step_state_ = StepState::IDLE;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachTcpZForceBandNode::onRunning() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Cancel requested. Stopping force-band approach.");
    auto moveit_context = get_moveit_context_from_blackboard(config());
    moveit_context->stopMotion();
    return BT::NodeStatus::FAILURE;
  }
  if (step_state_ == StepState::MOVING ||
      step_state_ == StepState::RECOVERY_RETRACTING) {
    return pollActiveMotion();
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  if (enable_distance_guard_) {
    if (!sensor_context->isDistanceFresh(distance_freshness_sec_)) {
      return startRecoveryRetract("UC4 distance data stale")
        ? BT::NodeStatus::RUNNING
        : BT::NodeStatus::FAILURE;
    }
    double distance_m = 0.0;
    rclcpp::Time distance_stamp;
    if (!sensor_context->getLatestDistance(distance_m, distance_stamp)) {
      return startRecoveryRetract("No UC4 distance sample")
        ? BT::NodeStatus::RUNNING
        : BT::NodeStatus::FAILURE;
    }
    if (distance_m < min_distance_m_) {
      RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
        "UC4 distance guard triggered during approach: distance=%.1f mm < threshold=%.1f mm",
        distance_m * 1000.0,
        min_distance_m_ * 1000.0);
      return startRecoveryRetract("UC4 distance below threshold")
        ? BT::NodeStatus::RUNNING
        : BT::NodeStatus::FAILURE;
    }
  }
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
  rclcpp::Time force_stamp;
  if (!sensor_context->getLatestForceZ(force_z, force_stamp)) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      *get_ros_node_from_blackboard(config())->get_clock(),
      1000,
      "No force sample available yet");
    return BT::NodeStatus::RUNNING;
  }
  if (force_z < hard_min_force_z_) {
    RCLCPP_ERROR(rclcpp::get_logger("ApproachTcpZForceBandNode"),
      "Hard force limit exceeded during approach: force_z=%.3f N < %.3f N",
      force_z,
      hard_min_force_z_);
    return startRecoveryRetract("Hard force limit exceeded during approach")
      ? BT::NodeStatus::RUNNING
      : BT::NodeStatus::FAILURE;
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
    // Too little pressure. Advance along +TCP Z.
    step = std::abs(step_distance_);
  } 
  else {
    // Too much pressure, but not beyond hard limit. Retract along -TCP Z.
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
  return startTcpStep(step)
    ? BT::NodeStatus::RUNNING
    : BT::NodeStatus::FAILURE;
}

void ApproachTcpZForceBandNode::onHalted() {
  auto moveit_context = get_moveit_context_from_blackboard(config());
  moveit_context->stopMotion();
}