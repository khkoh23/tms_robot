#include "tms_robot_control/bt_nodes/check_treatment_inputs_fresh.hpp"

#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/bt_nodes/bt_utils.hpp"

CheckTreatmentInputsFreshNode::CheckTreatmentInputsFreshNode(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config) {
}

BT::PortsList CheckTreatmentInputsFreshNode::providedPorts() {
  return {
    BT::InputPort<bool>("require_force", true, "Require fresh FTS force data"),
    BT::InputPort<bool>("require_distance", true, "Require fresh UC4 distance data"),
    BT::InputPort<bool>("require_target_pose", true, "Require fresh camera TCP target pose"),
    BT::InputPort<double>("force_freshness_sec", 0.20, "Maximum allowed age of FTS force data"),
    BT::InputPort<double>("distance_freshness_sec", 0.20, "Maximum allowed age of UC4 distance data"),
    BT::InputPort<double>("target_pose_freshness_sec", 1.00, "Maximum allowed age of camera TCP target pose") 
  };
}

BT::NodeStatus CheckTreatmentInputsFreshNode::tick() {
  if (is_cancel_requested_from_blackboard(config())) {
    RCLCPP_WARN(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "Cancel requested before treatment input freshness check");
    return BT::NodeStatus::FAILURE;
  }
  auto require_force = getInput<bool>("require_force");
  auto require_distance = getInput<bool>("require_distance");
  auto require_target_pose = getInput<bool>("require_target_pose");
  auto force_freshness_sec = getInput<double>("force_freshness_sec");
  auto distance_freshness_sec = getInput<double>("distance_freshness_sec");
  auto target_pose_freshness_sec = getInput<double>("target_pose_freshness_sec");
  if (!require_force ||
      !require_distance ||
      !require_target_pose ||
      !force_freshness_sec ||
      !distance_freshness_sec ||
      !target_pose_freshness_sec) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "Missing required input port");
    return BT::NodeStatus::FAILURE;
  }
  auto sensor_context = get_sensor_context_from_blackboard(config());
  bool ok = true;
  if (require_force.value()) {
    if (!sensor_context->isForceFresh(force_freshness_sec.value())) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckTreatmentInputsFreshNode"),
        "FTS force data is not fresh. Required freshness <= %.3f sec",
        force_freshness_sec.value());
      ok = false;
    } 
    else {
      RCLCPP_INFO(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "FTS force data is fresh");
    }
  }
  if (require_distance.value()) {
    if (!sensor_context->isDistanceFresh(distance_freshness_sec.value())) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckTreatmentInputsFreshNode"),
        "UC4 distance data is not fresh. Required freshness <= %.3f sec",
        distance_freshness_sec.value());
      ok = false;
    } 
    else {
      RCLCPP_INFO(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "UC4 distance data is fresh");
    }
  }
  if (require_target_pose.value()) {
    if (!sensor_context->isTcpTargetPoseFresh(target_pose_freshness_sec.value())) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckTreatmentInputsFreshNode"),
        "Camera TCP target pose is not fresh. Required freshness <= %.3f sec",
        target_pose_freshness_sec.value());
      ok = false;
    } 
    else {
      RCLCPP_INFO(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "Camera TCP target pose is fresh");
    }
  }
  if (!ok) {
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("CheckTreatmentInputsFreshNode"), "Treatment hardware inputs are fresh");
  return BT::NodeStatus::SUCCESS;
}