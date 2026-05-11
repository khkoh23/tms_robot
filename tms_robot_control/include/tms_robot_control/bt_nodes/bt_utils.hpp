#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_control/moveit/moveit_context.hpp"

inline bool is_cancel_requested_from_blackboard(const BT::NodeConfig & config) {
  if (!config.blackboard) {
    return false;
  }
  try {
    auto * flag = config.blackboard->get<std::atomic<bool> *>("cancel_requested");
    return flag && flag->load();
  }
  catch (...) {
    return false;
  }
}

inline rclcpp::Node::SharedPtr get_ros_node_from_blackboard(const BT::NodeConfig & config) {
  if (!config.blackboard) {
    throw std::runtime_error("BT blackboard is null");
  }
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

inline std::shared_ptr<MoveItContext> get_moveit_context_from_blackboard(const BT::NodeConfig & config) {
  if (!config.blackboard) {
    throw std::runtime_error("BT blackboard is null");
  }
  return config.blackboard->get<std::shared_ptr<MoveItContext>>("moveit_context");
}