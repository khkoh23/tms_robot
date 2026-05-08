#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include "tms_robot_interfaces/action/execute_task.hpp"
#include "tms_robot_interfaces/msg/bt_node_status.hpp"
#include "tms_robot_interfaces/msg/bt_state.hpp"

class TaskExecutorNode : public rclcpp::Node {
public:
  using ExecuteTask = tms_robot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  TaskExecutorNode();
private:
  void register_bt_nodes();
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void execute_goal(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  bool load_tree_for_task(const std::string & task_name);
  void publish_tree_status();
  std::string node_type_to_string(BT::NodeType type) const;
  std::string status_to_string(BT::NodeStatus status) const;
  std::string task_xml_path(const std::string & task_name) const;
  void publish_task_state(
    const std::string & task_name,
    const std::string & overall_state,
    const std::string & active_node,
    const std::string & message);
  void publish_log(const std::string & message);
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Publisher<tms_robot_interfaces::msg::BtNodeStatus>::SharedPtr bt_node_pub_;
  rclcpp::Publisher<tms_robot_interfaces::msg::BtState>::SharedPtr bt_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_log_pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unordered_map<std::string, std::string> last_status_;
  std::atomic<bool> cancel_requested_;
};