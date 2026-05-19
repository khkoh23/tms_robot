#include "tms_robot_control/moveit/moveit_context.hpp"

#include <cmath>
#include <exception>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool MoveItContext::checkSystemReady(const std::string & planning_group, const std::string & target, std::string & error_msg) {
  if (!rclcpp::ok()) {
    error_msg = "ROS is not OK";
    return false;
  }
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  const auto joint_names = move_group->getJointNames();
  if (joint_names.empty()) {
    error_msg = "Planning group '" + planning_group + "' has no joints";
    return false;
  }
  if (!target.empty()) {
    const auto target_values = move_group->getNamedTargetValues(target);
    if (target_values.empty()) {
      error_msg = "Named target '" + target + "' not found for planning group '" + planning_group + "'";
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "System ready for planning group '%s'", planning_group.c_str());
  return true;
}

bool MoveItContext::startMoveToNamedTarget(const std::string & planning_group, const std::string & target, std::string & error_msg) {
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Planning move to named target '%s' using planning group '%s'", target.c_str(), planning_group.c_str());
  move_group->setNamedTarget(target);
  const auto plan_result = move_group->plan(active_plan_);
  if (plan_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    error_msg = "Planning failed for target '" + target + "'. MoveIt error code: " + std::to_string(plan_result.val);
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str()); 
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Planning succeeded. Starting execution.");
  active_planning_group_ = planning_group; 
  auto plan_copy = active_plan_; 
  execution_future_ = std::async(std::launch::async, [move_group, plan_copy]() mutable {
    return move_group->execute(plan_copy);
  });
  return true;
}

bool MoveItContext::startMoveToFrameOffsetPose(const std::string & planning_group, const std::string & tcp_link, const std::string & reference_frame, double x, double y, double z, double roll, double pitch, double yaw, double velocity_scale, double acceleration_scale, std::string & error_msg) { 
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Planning TCP pose target for link '%s' in frame '%s': xyz=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
    tcp_link.c_str(), reference_frame.c_str(), x, y, z, roll, pitch, yaw);
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  try {
    move_group->clearPoseTargets();
    move_group->setPoseReferenceFrame(reference_frame);
    if (!tcp_link.empty()) {
      move_group->setEndEffectorLink(tcp_link);
    }
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setMaxAccelerationScalingFactor(acceleration_scale);
    move_group->setPoseTarget(target_pose, tcp_link);
    const auto plan_result = move_group->plan(active_plan_);
    if (plan_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_msg = "Planning failed for frame-offset pose target. MoveIt error code: " + std::to_string(plan_result.val);
      RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Planning succeeded for frame-offset pose. Starting execution.");
    active_planning_group_ = planning_group;
    auto plan_copy = active_plan_;
    execution_future_ = std::async(std::launch::async, [move_group, plan_copy]() mutable {
      return move_group->execute(plan_copy); 
    });
    return true;
  }
  catch (const std::exception & e) {
    error_msg = "Exception while planning frame-offset pose target: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return false;
  }
}

MoveItContext::MotionStatus MoveItContext::pollMotionResult(std::string & error_msg) {
  if (!execution_future_.valid()) {
    error_msg = "Execution future is invalid";
    return MotionStatus::FAILURE;
  }
  const auto future_status = execution_future_.wait_for(std::chrono::milliseconds(0));
  if (future_status != std::future_status::ready) {
    return MotionStatus::RUNNING;
  }
  try {
    const auto execution_result = execution_future_.get();
    if (execution_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "MoveIt execution succeeded");
      return MotionStatus::SUCCESS;
    }
    error_msg = "MoveIt execution failed. Error code: " + std::to_string(execution_result.val);
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return MotionStatus::FAILURE;
  }
  catch (const std::exception & e) {
    error_msg = "Exception while getting execution result: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return MotionStatus::FAILURE;
  }
}

void MoveItContext::stopMotion() {
  if (active_planning_group_.empty()) {
    return;
  }
  std::string error_msg;
  auto * move_group = getMoveGroup(active_planning_group_, error_msg);
  if (!move_group) {
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "Stopping MoveIt execution for planning group '%s'", active_planning_group_.c_str());
  move_group->stop();
  move_group->clearPoseTargets();
}

bool MoveItContext::verifyNamedTargetReached(
  const std::string & planning_group,
  const std::string & target,
  double tolerance,
  double & max_error,
  std::string & worst_joint,
  std::string & error_msg) {
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  const auto target_values = move_group->getNamedTargetValues(target);
  if (target_values.empty()) {
    error_msg = "Named target '" + target + "' not found for planning group '" + planning_group + "'";
    return false;
  }
  auto current_state = move_group->getCurrentState(2.0);
  if (!current_state) {
    error_msg = "Failed to get current robot state";
    return false;
  }
  max_error = 0.0;
  worst_joint.clear();
  for (const auto & [joint_name, target_position] : target_values) {
    const double current_position = current_state->getVariablePosition(joint_name);
    const double error = std::abs(current_position - target_position);
    if (error > max_error) {
      max_error = error;
      worst_joint = joint_name;
    }
    if (error > tolerance) {
      error_msg = "Target verification failed. Joint '" + joint_name + "' error " + std::to_string(error) + " exceeds tolerance " + std::to_string(tolerance);
      return false;
    }
  }
  return true;
}

bool MoveItContext::startMoveToPoseStamped(const std::string & planning_group,
  const std::string & tcp_link,
  const geometry_msgs::msg::PoseStamped & target_pose,
  double velocity_scale,
  double acceleration_scale,
  double planning_time,
  int planning_attempts,
  double position_tolerance,
  double orientation_tolerance,
  std::string & error_msg) {
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  if (target_pose.header.frame_id.empty()) {
    error_msg = "Target pose has empty frame_id";
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
    "Planning TCP pose for link '%s' in frame '%s': xyz=[%.3f, %.3f, %.3f]",
    tcp_link.c_str(),
    target_pose.header.frame_id.c_str(),
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);
  try {
    move_group->clearPoseTargets();
    move_group->setPoseReferenceFrame(target_pose.header.frame_id);
    if (!tcp_link.empty()) {
      move_group->setEndEffectorLink(tcp_link);
    }
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setMaxAccelerationScalingFactor(acceleration_scale);
    move_group->setPlanningTime(planning_time);
    move_group->setNumPlanningAttempts(planning_attempts);
    move_group->setGoalPositionTolerance(position_tolerance);
    move_group->setGoalOrientationTolerance(orientation_tolerance);
    move_group->setPoseTarget(target_pose.pose, tcp_link);
    const auto plan_result = move_group->plan(active_plan_);
    if (plan_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_msg = "Planning failed for TCP target offset pose. MoveIt error code: " +
        std::to_string(plan_result.val);
      RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Planning succeeded for TCP target offset pose. Starting execution.");
    active_planning_group_ = planning_group;
    auto plan_copy = active_plan_;
    execution_future_ = std::async(std::launch::async, [move_group, plan_copy]() mutable { 
      return move_group->execute(plan_copy);
    });
    return true;
  }
  catch (const std::exception & e) {
    error_msg = "Exception while planning TCP target offset pose: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return false;
  }
}

bool MoveItContext::startMoveTcpRelativeZ(const std::string & planning_group,
  const std::string & tcp_link,
  double distance_m,
  double velocity_scale,
  double acceleration_scale,
  double eef_step,
  double min_fraction,
  std::string & error_msg) {
  auto * move_group = getMoveGroup(planning_group, error_msg);
  if (!move_group) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Planning TCP relative Z motion for link '%s': distance=%.6f m",
    tcp_link.c_str(),
    distance_m);
  try {
    move_group->clearPoseTargets();
    if (!tcp_link.empty()) {
      move_group->setEndEffectorLink(tcp_link);
    }
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setMaxAccelerationScalingFactor(acceleration_scale);
    move_group->setStartStateToCurrentState();
    const std::string planning_frame = move_group->getPlanningFrame();
    // Important:
    // Shared MoveGroupInterface may retain a previous pose reference frame.
    // Force Cartesian waypoint interpretation into the planning frame.
    move_group->setPoseReferenceFrame(planning_frame);
    auto current_state = move_group->getCurrentState(2.0);
    if (!current_state) {
      error_msg = "Failed to get current robot state for TCP relative Z motion";
      return false;
    }
    const auto * link_model = current_state->getRobotModel()->getLinkModel(tcp_link);
    if (!link_model) {
      error_msg = "TCP link '" + tcp_link + "' not found in robot model";
      return false;
    }
    const Eigen::Isometry3d current_eigen = current_state->getGlobalLinkTransform(tcp_link);
    Eigen::Isometry3d relative_move = Eigen::Isometry3d::Identity();
    // Positive distance means +local TCP Z.
    relative_move.translation() = Eigen::Vector3d(0.0, 0.0, distance_m);
    // Apply relative move in current TCP local frame.
    const Eigen::Isometry3d target_eigen = current_eigen * relative_move;
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_eigen.translation().x();
    target_pose.position.y = target_eigen.translation().y();
    target_pose.position.z = target_eigen.translation().z();
    Eigen::Quaterniond q(target_eigen.rotation());
    q.normalize();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    const Eigen::Vector3d delta = target_eigen.translation() - current_eigen.translation();
    RCLCPP_INFO(node_->get_logger(), "TCP relative Z target in planning frame '%s': current=[%.6f, %.6f, %.6f], target=[%.6f, %.6f, %.6f], delta_norm=%.6f m",
      planning_frame.c_str(),
      current_eigen.translation().x(),
      current_eigen.translation().y(),
      current_eigen.translation().z(),
      target_eigen.translation().x(),
      target_eigen.translation().y(),
      target_eigen.translation().z(),
      delta.norm());
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group->computeCartesianPath(waypoints, eef_step, trajectory);
    if (fraction < min_fraction) {
      error_msg = "Cartesian TCP Z path fraction too low: " +
        std::to_string(fraction) +
        " < " +
        std::to_string(min_fraction);
      RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Cartesian TCP Z path computed successfully: %.2f%% covered", fraction * 100.0);
    active_plan_ = moveit::planning_interface::MoveGroupInterface::Plan();
    active_plan_.trajectory = trajectory;
    active_planning_group_ = planning_group;
    auto plan_copy = active_plan_;
    execution_future_ = std::async(std::launch::async, [move_group, plan_copy]() mutable {
      return move_group->execute(plan_copy); });
    return true;
  }
  catch (const std::exception & e) {
    error_msg = "Exception while planning TCP relative Z motion: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return false;
  }
}

MoveItContext::MoveItContext(rclcpp::Node::SharedPtr node) : node_(node) {
}

moveit::planning_interface::MoveGroupInterface * MoveItContext::getMoveGroup(const std::string & planning_group, std::string & error_msg) { 
  auto it = move_groups_.find(planning_group);
  if (it != move_groups_.end()) {
    return it->second.get();
  }
  try {
    auto move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
    auto * raw_ptr = move_group.get();
    move_groups_[planning_group] = std::move(move_group);
    return raw_ptr;
  }
  catch (const std::exception & e) {
    error_msg = "Failed to create MoveGroupInterface for group '" + planning_group + "': " + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return nullptr;
  }
}