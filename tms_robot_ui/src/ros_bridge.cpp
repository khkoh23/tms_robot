#include "tms_robot_ui/ros_bridge.hpp"

RosBridge::RosBridge(rclcpp::Node::SharedPtr node, QObject * parent) : QObject(parent), node_(node) {
  action_client_ = rclcpp_action::create_client<ExecuteTask>(node_, "execute_task");
  bt_node_sub_ = node_->create_subscription<tms_robot_interfaces::msg::BtNodeStatus>(
    "bt_node_status", 10,
    [this](tms_robot_interfaces::msg::BtNodeStatus::SharedPtr msg) {
      emit btNodeStatusUpdated(
        QString::fromStdString(msg->node_name),
        QString::fromStdString(msg->node_type),
        QString::fromStdString(msg->status));
    });
  bt_state_sub_ = node_->create_subscription<tms_robot_interfaces::msg::BtState>(
    "bt_state", 10,
    [this](tms_robot_interfaces::msg::BtState::SharedPtr msg) {
      emit taskStateUpdated(
        QString::fromStdString(msg->task_name),
        QString::fromStdString(msg->overall_state),
        QString::fromStdString(msg->active_node),
        QString::fromStdString(msg->message));
    });
}

RosBridge::~RosBridge() = default;

void RosBridge::startTask(const QString & task_name) {
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    emit logMessage("ExecuteTask action server not available");
    return;
  }
  ExecuteTask::Goal goal;
  goal.task_name = task_name.toStdString();
  auto options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  options.goal_response_callback =
    [this](GoalHandleExecuteTask::SharedPtr handle) {
      current_goal_handle_ = handle;
      if (handle) {
        emit logMessage("Task goal accepted");
      } else {
        emit logMessage("Task goal rejected");
      }
    };
  options.feedback_callback =
    [this](GoalHandleExecuteTask::SharedPtr,
           const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
      emit taskStateUpdated(
        "",
        QString::fromStdString(feedback->current_state),
        QString::fromStdString(feedback->active_node),
        QString::fromStdString(feedback->message));
    };
  options.result_callback =
    [this](const GoalHandleExecuteTask::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          emit logMessage("Task finished: SUCCEEDED");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          emit logMessage("Task finished: ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          emit logMessage("Task finished: CANCELED");
          break;
        default:
          emit logMessage("Task finished: UNKNOWN");
          break;
      }
    };
  action_client_->async_send_goal(goal, options);
}

void RosBridge::cancelTask() {
  if (current_goal_handle_) {
    action_client_->async_cancel_goal(current_goal_handle_);
    emit logMessage("Cancel requested");
  } 
  else {
    emit logMessage("No active goal handle to cancel");
  }
}
