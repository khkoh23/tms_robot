#include <rclcpp/rclcpp.hpp>
#include <tms_robot_control/bt_executor.h>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto executor_node = std::make_shared<BTExecutor>();
    executor_node->initialize();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(executor_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}