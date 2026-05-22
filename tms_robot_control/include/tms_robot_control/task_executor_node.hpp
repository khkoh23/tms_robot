#pragma once

#include <atomic>
#include <memory>
#include <mutex>
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

class MoveItContext;

class SensorContext;

class TaskExecutorNode : public rclcpp::Node {
public:
  using ExecuteTask = tms_robot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  TaskExecutorNode();

private:
  enum class TaskLifecycleState {
    IDLE, PRECHECK, RUNNING, CANCELING, CANCELED, SUCCESS, FAILURE, FAULT
  };
  static constexpr double kMinTcpOffsetZ = 0.030;
  static constexpr double kMaxTcpOffsetZ = 0.100;
  static constexpr double kMinTreatmentDurationSec = 10.0;
  static constexpr double kMaxTreatmentDurationSec = 300.0;
  static constexpr double kMinDistanceGuardM = 0.025;
  static constexpr double kMaxDistanceGuardM = 0.150;
  static constexpr double kContactRecoveryRetractDistanceM = -0.050;
  static constexpr double kContactRecoveryVelocityScale = 0.01;
  static constexpr double kContactRecoveryAccelerationScale = 0.01;
  static constexpr double kContactRecoveryEefStep = 0.0001;
  static constexpr double kContactRecoveryMinFraction = 0.90;
  void register_bt_nodes();
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void execute_goal(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  bool load_tree_for_task(const std::string & task_name, 
    double tcp_offset_z_m, 
    double treatment_duration_sec, 
    bool enable_distance_guard, 
    double min_distance_m);
  void publish_tree_status();
  std::string task_xml_path(const std::string & task_name) const;
  std::string lifecycle_state_to_string(TaskLifecycleState state) const;
  void set_lifecycle_state(TaskLifecycleState state, const std::string & task_name, const std::string & active_node, const std::string & message);
  void publish_task_state(const std::string & task_name, const std::string & overall_state, const std::string & active_node, const std::string & message);
  void publish_log(const std::string & message);
  void clear_treatment_status();
  double clamp_double(double value, double min_value, double max_value, const std::string & name);
  void publish_task_parameter_summary(const std::string & task_name, 
    double tcp_offset_z_m,
    double treatment_duration_sec,
    bool enable_distance_guard,
    double min_distance_m);
  bool set_contact_collision_allowed(bool allowed);
  void restore_contact_collision_allowance();
  bool perform_contact_recovery_retract();
  std::string status_to_string(BT::NodeStatus status) const;
  std::string node_type_to_string(BT::NodeType type) const;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Publisher<tms_robot_interfaces::msg::BtNodeStatus>::SharedPtr bt_node_pub_;
  rclcpp::Publisher<tms_robot_interfaces::msg::BtState>::SharedPtr bt_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr treatment_status_pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unordered_map<std::string, std::string> last_status_;
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> task_running_{false};
  std::atomic<bool> contact_recovery_required_{false};
  bool contact_collision_allowed_{false};
  std::mutex contact_collision_mutex_;
  TaskLifecycleState lifecycle_state_{TaskLifecycleState::IDLE};
  std::shared_ptr<MoveItContext> moveit_context_;
  std::shared_ptr<SensorContext> sensor_context_;
};