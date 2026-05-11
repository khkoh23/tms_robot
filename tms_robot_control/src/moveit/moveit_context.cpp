#include "tms_robot_control/moveit/moveit_context.hpp"

#include <cmath>
#include <exception>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

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
  execution_future_ = std::async(std::launch::async, [move_group, plan = active_plan_]() { 
    return move_group->execute(plan); 
  });
  return true;
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