#include "tms_robot_ui/camera_view_widget.hpp"
#include <QMetaObject>
#include <QApplication>

CameraViewWidget::CameraViewWidget(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QWidget(parent), node_(node), label_(nullptr) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  label_ = new QLabel(this);
  label_->setAlignment(Qt::AlignCenter);
  label_->setStyleSheet("background-color: #2b2b2b; color: #ffffff; border: 1px solid #555;");
  label_->setText("Waiting for AI Stream...");
  label_->setFixedSize(640, 480); // Locks the label to 640x480
  this->setFixedSize(640, 480);  // Optional: Locks the whole widget if it's a standalone window
  layout->addWidget(label_);
  setTopic("/camera/ai_results");
}

CameraViewWidget::~CameraViewWidget() {
}

void CameraViewWidget::setTopic(const QString &topic) {
  std::string topic_str = topic.toStdString();
  if (sub_) {
    sub_.shutdown();
  }
  RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", topic_str.c_str());
  try {
    sub_ = image_transport::create_subscription(
      node_.get(),
      topic_str,
      std::bind(&CameraViewWidget::imageCallback, this, std::placeholders::_1),
      "raw",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
  } 
  catch (const std::exception &e) {
    handleError(std::string("Failed to subscribe: ") + e.what());
  }
}

void CameraViewWidget::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat cv_mat = cv_ptr->image;
    cv::Mat rgb_mat;
    if (cv_mat.channels() == 3) {
      cv::cvtColor(cv_mat, rgb_mat, cv::COLOR_BGR2RGB);
    } 
    else {
      cv::cvtColor(cv_mat, rgb_mat, cv::COLOR_GRAY2RGB);
    }
    QImage q_image(rgb_mat.data, rgb_mat.cols, rgb_mat.rows, rgb_mat.step, QImage::Format_RGB888);
    QImage final_img = q_image.copy();
    QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, final_img));
  } 
  catch (const cv_bridge::Exception &e) {
    handleError("cv_bridge error: " + std::string(e.what()));
  } 
  catch (const std::exception &e) {
    handleError("Image conversion error: " + std::string(e.what()));
  }
}

void CameraViewWidget::updateImage(const QImage &img) {
  label_->setPixmap(QPixmap::fromImage(img));
  if (!label_->text().contains("Waiting")) {
  }
}

void CameraViewWidget::handleError(const std::string &msg) {
  RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str()); 
  label_->setText(QString::fromStdString(msg));
  label_->setStyleSheet("background-color: #500; color: white; border: 1px solid red;");
  emit errorOccurred(QString::fromStdString(msg));
}