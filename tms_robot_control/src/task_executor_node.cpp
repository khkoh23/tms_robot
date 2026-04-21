#include "tms_robot_control/task_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class WaitNode : public BT::StatefulActionNode
{
public:
  WaitNode(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config), duration_ms_(1000)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("msec")};
  }

  BT::NodeStatus onStart() override
  {
    auto msec = getInput<int>("msec");
    if (msec) {
      duration_ms_ = msec.value();
    }
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= duration_ms_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  int duration_ms_;
  std::chrono::steady_clock::time_point start_time_;
};

class ReportStatusNode : public BT::SyncActionNode
{
public:
  ReportStatusNode(
    const std::string & name,
    const BT::NodeConfig & config,
    std::function<void(const std::string &)> log_fn)
  : BT::SyncActionNode(name, config), log_fn_(log_fn)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (msg) {
      log_fn_(msg.value());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::function<void(const std::string &)> log_fn_;
};

class MoveArmNamedTargetNode : public BT::StatefulActionNode
{
public:
  MoveArmNamedTargetNode(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target")};
  }

  BT::NodeStatus onStart() override
  {
    auto target = getInput<std::string>("target");
    if (target) {
      target_name_ = target.value();
    } else {
      target_name_ = "unknown";
    }

    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Placeholder for future real MoveIt integration.
    // Later this node will call MoveIt and monitor execution.
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed > 1200) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::string target_name_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace

TaskExecutorNode::TaskExecutorNode()
: Node("robot_task_executor"), cancel_requested_(false)
{
  register_bt_nodes();

  bt_node_pub_ = create_publisher<tms_robot_interfaces::msg::BtNodeStatus>(
    "bt_node_status", 10);

  bt_state_pub_ = create_publisher<tms_robot_interfaces::msg::BtState>(
    "bt_state", 10);

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this,
    "execute_task",
    std::bind(&TaskExecutorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskExecutorNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&TaskExecutorNode::handle_accepted, this, std::placeholders::_1));
}

void TaskExecutorNode::register_bt_nodes()
{
  factory_.registerNodeType<WaitNode>("Wait");
  factory_.registerNodeType<MoveArmNamedTargetNode>("MoveArmNamedTarget");

  factory_.registerBuilder<ReportStatusNode>(
    "ReportStatus",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReportStatusNode>(
        name, config,
        [this](const std::string & msg) {
          tms_robot_interfaces::msg::BtState state_msg;
          state_msg.task_name = "";
          state_msg.overall_state = "INFO";
          state_msg.active_node = "";
          state_msg.message = msg;
          bt_state_pub_->publish(state_msg);
          RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        });
    });
}

rclcpp_action::GoalResponse TaskExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteTask::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal for task: %s", goal->task_name.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskExecutorNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteTask>)
{
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
{
  std::thread{std::bind(&TaskExecutorNode::execute_goal, this, goal_handle)}.detach();
}

void TaskExecutorNode::execute_goal(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  cancel_requested_ = false;

  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result = std::make_shared<ExecuteTask::Result>();

  if (!load_tree_for_task(goal->task_name)) {
    result->success = false;
    result->message = "Failed to load tree";
    goal_handle->abort(result);
    return;
  }

  tms_robot_interfaces::msg::BtState state_msg;
  state_msg.task_name = goal->task_name;
  state_msg.overall_state = "RUNNING";
  state_msg.active_node = "";
  state_msg.message = "Task started";
  bt_state_pub_->publish(state_msg);

  while (rclcpp::ok()) {
    if (cancel_requested_) {
      tree_.haltTree();

      state_msg.task_name = goal->task_name;
      state_msg.overall_state = "CANCELED";
      state_msg.active_node = "";
      state_msg.message = "Task canceled";
      bt_state_pub_->publish(state_msg);

      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    const auto status = tree_.tickOnce();
    publish_tree_status();

    std::string active_node;

    BT::applyRecursiveVisitor(tree_.rootNode(), [&](BT::TreeNode * node) {
      if (active_node.empty() && node->status() == BT::NodeStatus::RUNNING) {
        active_node = node->name();
      }
    });

    feedback->current_state = status_to_string(status);
    feedback->active_node = active_node;
    feedback->message = "Executing";
    goal_handle->publish_feedback(feedback);

    state_msg.task_name = goal->task_name;
    state_msg.overall_state = status_to_string(status);
    state_msg.active_node = active_node;
    state_msg.message = "Executing";
    bt_state_pub_->publish(state_msg);

    if (status == BT::NodeStatus::SUCCESS) {
      result->success = true;
      result->message = "Task succeeded";
      goal_handle->succeed(result);
      return;
    }

    if (status == BT::NodeStatus::FAILURE) {
      result->success = false;
      result->message = "Task failed";
      goal_handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(50ms);
  }
}

bool TaskExecutorNode::load_tree_for_task(const std::string & task_name)
{
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
    tree_ = factory_.createTreeFromText(buffer.str(), BT::Blackboard::create());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create tree: %s", e.what());
    return false;
  }

  last_status_.clear();
  publish_tree_status();
  return true;
}

void TaskExecutorNode::publish_tree_status()
{
  BT::applyRecursiveVisitor(tree_.rootNode(), [&](BT::TreeNode * node) {
    const auto name = node->name();
    const auto status = status_to_string(node->status());

    auto it = last_status_.find(name);
    if (it == last_status_.end() || it->second != status) {
      last_status_[name] = status;

      tms_robot_interfaces::msg::BtNodeStatus msg;
      msg.node_name = name;
      msg.node_type = "BTNode";
      msg.status = status;
      bt_node_pub_->publish(msg);
    }
  });
}

std::string TaskExecutorNode::status_to_string(BT::NodeStatus status) const
{
  switch (status) {
    case BT::NodeStatus::IDLE: return "IDLE";
    case BT::NodeStatus::RUNNING: return "RUNNING";
    case BT::NodeStatus::SUCCESS: return "SUCCESS";
    case BT::NodeStatus::FAILURE: return "FAILURE";
    default: return "UNKNOWN";
  }
}

std::string TaskExecutorNode::task_xml_path(const std::string & task_name) const
{
  const auto share_dir = ament_index_cpp::get_package_share_directory("tms_robot_control");

  if (task_name == "inspect") {
    return share_dir + "/tree/inspect_tree.xml";
  }
  if (task_name == "patrol") {
    return share_dir + "/tree/patrol_tree.xml";
  }
  if (task_name == "move_arm_home") {
    return share_dir + "/tree/move_arm_home.xml";
  }
  return "";
}
