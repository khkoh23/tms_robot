#include "tms_robot_ui/rviz_widget.hpp"
#include <filesystem>
#include <QVBoxLayout>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

RvizWidget::RvizWidget(QWidget * parent): QWidget(parent) {
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  rviz_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_visualization_node");
  frame_ = new rviz_common::VisualizationFrame(rviz_ros_node_, this);
  frame_->setWindowFlags(frame_->windowFlags() & ~Qt::Window);
  frame_->setWindowFlags(frame_->windowFlags() | Qt::Widget) ;
  frame_->setAttribute(Qt::WA_DeleteOnClose, false);
  frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(frame_);
  initialized_ = false;
}

void RvizWidget::showEvent(QShowEvent * event) {
  QWidget::showEvent(event);
  if (!initialized_) {
    QTimer::singleShot(0, this, &RvizWidget::initialize);
  }
}

void RvizWidget::initialize() {
  if (initialized_ || !frame_) {
    return;
  }
  try {
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("tms_robot_moveit_config");
    const std::string rviz_config = pkg_share + "/config/moveit.rviz";
    const std::string kin_yaml_path = pkg_share + "/config/kinematics.yaml";
    RCLCPP_INFO(rclcpp::get_logger("RvizWidget"), "Loading RViz config: %s", rviz_config.c_str());
    if (!std::filesystem::exists(rviz_config)) {
      RCLCPP_ERROR(rclcpp::get_logger("RvizWidget"), "Config file not found: %s", rviz_config.c_str());
      return;
    } 
    frame_->setSplashPath("");
    frame_->initialize(rviz_ros_node_, QString::fromStdString(rviz_config));
    frame_->show();
    frame_->raise();
    frame_->layout()->activate();
    RCLCPP_INFO(rclcpp::get_logger("RVizWidget"), "VisualizationFrame initialized");
    frame_->updateGeometry();
    frame_->repaint();
    initialized_ = true; 
  } 
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RvizWidget"), "Failed to initialize: %s", e.what());
    initialized_ = false;
  }
}