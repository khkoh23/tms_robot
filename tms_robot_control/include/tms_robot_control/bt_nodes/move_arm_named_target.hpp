#pragma once

#include <future>
#include <memory>
#include <string>
#include <behaviortree_cpp/action_node.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveArmNamedTargetNode : public BT::StatefulActionNode {
public:
  MoveArmNamedTargetNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool initialized_{false};
  std::string target_name_;
  std::string planning_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};