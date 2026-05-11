#pragma once

#include <string>
#include <memory>
#include <behaviortree_cpp/condition_node.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

class VerifyNamedTargetReachedNode : public BT::ConditionNode {
public:
  VerifyNamedTargetReachedNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  std::string target_name_;
  std::string planning_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool initializeMoveGroup();
};