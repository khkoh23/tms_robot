#include "tms_robot_control/task_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>
#include <thread>
#include "tms_robot_control/bt_nodes/move_arm_named_target.hpp"
#include "tms_robot_control/bt_nodes/check_system_ready.hpp"
#include "tms_robot_control/bt_nodes/report_status.hpp"
#include "tms_robot_control/bt_nodes/verify_named_target_reached.hpp"

using namespace std::chrono_literals;

namespace {

bool is_cancel_requested_from_blackboard(const BT::NodeConfig & config) {
  if (!config.blackboard) {
    return false;
  }
  try {
    auto * cancel_flag = config.blackboard->get<std::atomic<bool> *>("cancel_requested");
    return cancel_flag && cancel_flag->load();
  } 
  catch (const std::exception &) {
    return false;
  }
}

class WaitNode : public BT::StatefulActionNode {
public:
  WaitNode(const std::string & name, const BT::NodeConfig & config) 
  : BT::StatefulActionNode(name, config), duration_ms_(1000) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("msec")
    };
  }
  BT::NodeStatus onStart() override {
    if (is_cancel_requested_from_blackboard(config())) {
      return BT::NodeStatus::FAILURE;
    }
    auto msec = getInput<int>("msec");
    if (msec) {
      duration_ms_ = msec.value();
    }
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (is_cancel_requested_from_blackboard(config())) {
      return BT::NodeStatus::FAILURE;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= duration_ms_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {
  }

private:
  int duration_ms_;
  std::chrono::steady_clock::time_point start_time_;
};

class TaskRunningGuard {
public:
  explicit TaskRunningGuard(std::atomic<bool> & flag) : flag_(flag) {}
  ~TaskRunningGuard() {
    flag_ = false;
  }

private:
  std::atomic<bool> & flag_;
};

}  // namespace

TaskExecutorNode::TaskExecutorNode() : Node("robot_task_executor") {
  register_bt_nodes();
  bt_node_pub_ = create_publisher<tms_robot_interfaces::msg::BtNodeStatus>("bt_node_status", 10);
  bt_state_pub_ = create_publisher<tms_robot_interfaces::msg::BtState>("bt_state", 10);
  bt_log_pub_ = create_publisher<std_msgs::msg::String>("bt_log", 10);
  action_server_ = rclcpp_action::create_server<ExecuteTask>(this, "execute_task", 
    std::bind(&TaskExecutorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
    std::bind(&TaskExecutorNode::handle_cancel, this, std::placeholders::_1), 
    std::bind(&TaskExecutorNode::handle_accepted, this, std::placeholders::_1));
}

void TaskExecutorNode::register_bt_nodes() {
  factory_.registerNodeType<WaitNode>("Wait");
  factory_.registerNodeType<CheckSystemReadyNode>("CheckSystemReady");
  factory_.registerNodeType<MoveArmNamedTargetNode>("MoveArmNamedTarget");
  factory_.registerBuilder<ReportStatusNode>("ReportStatus", [this](const std::string & name, const BT::NodeConfig & config) {
    return std::make_unique<ReportStatusNode>(name, config, [this](const std::string & msg) {
      publish_log(msg);
      RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); 
    });
  });
  factory_.registerNodeType<VerifyNamedTargetReachedNode>("VerifyNamedTargetReached");
}

rclcpp_action::GoalResponse TaskExecutorNode::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal) {
  bool expected = false;
  if (!task_running_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "Rejecting task '%s': another task is already running", goal->task_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Received goal for task: %s", goal->task_name.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskExecutorNode::handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>) {
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
  std::thread{std::bind(&TaskExecutorNode::execute_goal, this, goal_handle) }.detach();
}

void TaskExecutorNode::execute_goal(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
  TaskRunningGuard running_guard(task_running_);
  const auto goal = goal_handle->get_goal();
  cancel_requested_ = false;
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result = std::make_shared<ExecuteTask::Result>();
  if (!load_tree_for_task(goal->task_name)) {
    set_lifecycle_state(TaskLifecycleState::FAILURE, goal->task_name, "", "Failed to load behavior tree");
    result->success = false;
    result->message = "Failed to load tree";
    goal_handle->abort(result);
    return;
  }
  set_lifecycle_state(TaskLifecycleState::PRECHECK, goal->task_name, "", "Checking task prerequisites");
  std::this_thread::sleep_for(50ms);
  set_lifecycle_state(TaskLifecycleState::RUNNING, goal->task_name, "", "Task started");
  publish_log("Task started: " + goal->task_name);
  while (rclcpp::ok()) {
    if (cancel_requested_) {
      tree_.haltTree();
      publish_tree_status();
      set_lifecycle_state(TaskLifecycleState::CANCELED, goal->task_name, "", "Task canceled");
      publish_log("Task canceled: " + goal->task_name);
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }
    const auto status = tree_.tickOnce();
    publish_tree_status();
    std::string active_node;
    BT::applyRecursiveVisitor(tree_.rootNode(), [&](BT::TreeNode * node) {
      const auto node_type = node->type();
      const bool is_leaf = node_type == BT::NodeType::ACTION || node_type == BT::NodeType::CONDITION;
      if (active_node.empty() && node->status() == BT::NodeStatus::RUNNING && is_leaf) {
        active_node = node->name();
      }
    });
    const auto state_string = status_to_string(status);
    feedback->current_state = state_string;
    feedback->active_node = active_node;
    feedback->message = "Executing";
    goal_handle->publish_feedback(feedback);
    publish_task_state(goal->task_name, state_string, active_node, "Executing");
    if (status == BT::NodeStatus::SUCCESS) {
      set_lifecycle_state(TaskLifecycleState::SUCCESS, goal->task_name, "", "Task succeeded");
      publish_log("Task succeeded: " + goal->task_name);
      result->success = true;
      result->message = "Task succeeded";
      goal_handle->succeed(result);
      return;
    }
    if (status == BT::NodeStatus::FAILURE) {
      set_lifecycle_state(TaskLifecycleState::FAILURE, goal->task_name, active_node, "Task failed");
      publish_log("Task failed: " + goal->task_name);
      result->success = false;
      result->message = "Task failed";
      goal_handle->abort(result);
      return;
    }
    std::this_thread::sleep_for(50ms);
  }
  set_lifecycle_state(TaskLifecycleState::FAULT, goal->task_name, "", "ROS shutdown during task execution");
  result->success = false;
  result->message = "ROS shutdown during task execution";
  goal_handle->abort(result);
}

bool TaskExecutorNode::load_tree_for_task(const std::string & task_name) {
  const auto xml_path = task_xml_path(task_name);
  if (xml_path.empty()) {
    RCLCPP_ERROR(get_logger(), "Unknown task name: %s", task_name.c_str());
    return false;
  }
  std::ifstream in(xml_path);
  if (!in.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open tree file: %s", xml_path.c_str());
    return false;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  try {
    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::atomic<bool> *>("cancel_requested", &cancel_requested_);
    blackboard->set<rclcpp::Node::SharedPtr>("ros_node", this->shared_from_this());
    tree_ = factory_.createTreeFromText(buffer.str(), blackboard);
  } 
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create tree: %s", e.what());
    return false;
  }
  last_status_.clear();
  publish_tree_status();
  return true;
}

void TaskExecutorNode::publish_tree_status() {
  BT::applyRecursiveVisitor(tree_.rootNode(), [&](BT::TreeNode * node) {
    const auto name = node->name();
    const auto status = status_to_string(node->status());
    auto it = last_status_.find(name);
    if (it == last_status_.end() || it->second != status) {
      last_status_[name] = status;
      tms_robot_interfaces::msg::BtNodeStatus msg;
      msg.node_name = name;
      msg.node_type = node_type_to_string(node->type());
      msg.status = status;
      bt_node_pub_->publish(msg);
    }
  });
}

std::string TaskExecutorNode::task_xml_path(const std::string & task_name) const {
  const auto share_dir = ament_index_cpp::get_package_share_directory("tms_robot_control");
  if (task_name == "move_arm_ready") {
    return share_dir + "/tree/move_arm_ready.xml";
  }
  if (task_name == "move_arm_up") {
    return share_dir + "/tree/move_arm_up.xml";
  }
  if (task_name == "inspect") {
    return share_dir + "/tree/inspect_tree.xml";
  }
  if (task_name == "patrol") {
    return share_dir + "/tree/patrol_tree.xml";
  }
  return "";
}

std::string TaskExecutorNode::lifecycle_state_to_string(TaskLifecycleState state) const {
  switch (state) {
    case TaskLifecycleState::IDLE:
      return "IDLE";
    case TaskLifecycleState::PRECHECK:
      return "PRECHECK";
    case TaskLifecycleState::RUNNING:
      return "RUNNING";
    case TaskLifecycleState::CANCELING:
      return "CANCELING";
    case TaskLifecycleState::CANCELED:
      return "CANCELED";
    case TaskLifecycleState::SUCCESS:
      return "SUCCESS";
    case TaskLifecycleState::FAILURE:
      return "FAILURE";
    case TaskLifecycleState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

void TaskExecutorNode::set_lifecycle_state(TaskLifecycleState state, const std::string & task_name, const std::string & active_node, const std::string & message) {
  lifecycle_state_ = state;
  publish_task_state(task_name, lifecycle_state_to_string(state), active_node, message);
}

void TaskExecutorNode::publish_task_state(const std::string & task_name, const std::string & overall_state, const std::string & active_node, const std::string & message) {
  tms_robot_interfaces::msg::BtState state_msg;
  state_msg.task_name = task_name;
  state_msg.overall_state = overall_state;
  state_msg.active_node = active_node;
  state_msg.message = message;
  bt_state_pub_->publish(state_msg);
}

void TaskExecutorNode::publish_log(const std::string & message) {
  std_msgs::msg::String msg;
  msg.data = message;
  bt_log_pub_->publish(msg);
}

std::string TaskExecutorNode::status_to_string(BT::NodeStatus status) const {
  switch (status) {
    case BT::NodeStatus::IDLE:
      return "IDLE";
    case BT::NodeStatus::RUNNING:
      return "RUNNING";
    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";
    case BT::NodeStatus::FAILURE:
      return "FAILURE";
    default:
      return "UNKNOWN";
  }
}

std::string TaskExecutorNode::node_type_to_string(BT::NodeType type) const {
  switch (type) {
    case BT::NodeType::ACTION:
      return "ACTION";
    case BT::NodeType::CONDITION:
      return "CONDITION";
    case BT::NodeType::CONTROL:
      return "CONTROL";
    case BT::NodeType::DECORATOR:
      return "DECORATOR";
    case BT::NodeType::SUBTREE:
      return "SUBTREE";
    default:
      return "UNKNOWN";
  }
}