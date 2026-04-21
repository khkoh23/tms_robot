#include "tms_robot_control/task_executor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}