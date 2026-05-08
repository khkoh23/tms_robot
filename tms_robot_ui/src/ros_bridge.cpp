#include "tms_robot_ui/ros_bridge.hpp"

RosBridge::RosBridge(rclcpp::Node::SharedPtr node, QObject * parent) : QObject(parent), node_(node) {
  action_client_ = rclcpp_action::create_client<ExecuteTask>(node_, "execute_task");
  zero_force_client_ = node_->create_client<ZeroForce>("robotiq_ft_sensor_acc");
  zero_force_client_->configure_introspection(
    node->get_clock(), 
    rclcpp::SystemDefaultsQoS(), 
    RCL_SERVICE_INTROSPECTION_CONTENTS);
  bt_node_sub_ = node_->create_subscription<tms_robot_interfaces::msg::BtNodeStatus>(
    "bt_node_status", 10, [this](tms_robot_interfaces::msg::BtNodeStatus::SharedPtr msg) {
      emit btNodeStatusUpdated(
        QString::fromStdString(msg->node_name),
        QString::fromStdString(msg->node_type),
        QString::fromStdString(msg->status));
    });
  bt_state_sub_ = node_->create_subscription<tms_robot_interfaces::msg::BtState>(
    "bt_state", 10, [this](tms_robot_interfaces::msg::BtState::SharedPtr msg) {
      emit taskStateUpdated(
        QString::fromStdString(msg->task_name),
        QString::fromStdString(msg->overall_state),
        QString::fromStdString(msg->active_node),
        QString::fromStdString(msg->message));
    });
  bt_log_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "bt_log", 10, [this](std_msgs::msg::String::SharedPtr msg) {
      emit logMessage(QString::fromStdString(msg->data));
    });
  force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "robotiq_force_torque_sensor_broadcaster/wrench", 10, [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      emit forceUpdated(
        QString::number(msg->wrench.force.z, 'f', 2));
    });
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "distance", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
      emit distanceUpdated(
        QString::number(msg->data*1000, 'f', 1)); // Convert m to mm
    });
}

RosBridge::~RosBridge() = default;

void RosBridge::zeroFTS() {
  if (!zero_force_client_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(), "FTS Service not ready.");
      emit logMessage("Error: FTS Server is offline.");
      return;
  }
  auto request = std::make_shared<ZeroForce::Request>();
  request->command_id = ZeroForce::Request::COMMAND_SET_ZERO;
  auto result = zero_force_client_->async_send_request(
    request,
    [this](rclcpp::Client<ZeroForce>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(node_->get_logger(), "FTS Zeroing successful: %s", response->res.c_str());
          emit logMessage("FTS Success: " + QString::fromStdString(response->res));
        } 
        else {
          RCLCPP_ERROR(node_->get_logger(), "FTS Zeroing failed: %s", response->res.c_str());
          emit logMessage("FTS Failed: " + QString::fromStdString(response->res));
        }
      } 
      catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
        emit logMessage("FTS Error: System exception during call");
      }
    }
  );
  emit logMessage("Zeroing FTS300-S ...");
}

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