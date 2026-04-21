#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_ui/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("tms_robot_ui", options);
  MainWindow window(node);
  window.show();
  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
