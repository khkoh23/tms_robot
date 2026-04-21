#pragma once

#include <QObject>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "tms_robot_interfaces/action/execute_task.hpp"
#include "tms_robot_interfaces/msg/bt_node_status.hpp"
#include "tms_robot_interfaces/msg/bt_state.hpp"

class RosBridge : public QObject {
  Q_OBJECT

public:
  using ExecuteTask = tms_robot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ClientGoalHandle<ExecuteTask>;
  explicit RosBridge(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);
  ~RosBridge();
  rclcpp::Node::SharedPtr node() const { return node_; }
  void startTask(const QString & task_name);
  void cancelTask();

signals:
  void taskStateUpdated(const QString & task_name,
                        const QString & overall_state,
                        const QString & active_node,
                        const QString & message);

  void btNodeStatusUpdated(const QString & node_name,
                           const QString & node_type,
                           const QString & status);

  void logMessage(const QString & msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;
  GoalHandleExecuteTask::SharedPtr current_goal_handle_;
  rclcpp::Subscription<tms_robot_interfaces::msg::BtNodeStatus>::SharedPtr bt_node_sub_;
  rclcpp::Subscription<tms_robot_interfaces::msg::BtState>::SharedPtr bt_state_sub_;
};
