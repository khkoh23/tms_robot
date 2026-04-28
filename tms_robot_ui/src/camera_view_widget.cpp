#include "tms_robot_ui/camera_view_widget.hpp"
#include <QMetaObject>
#include <QApplication>

CameraViewWidget::CameraViewWidget(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QWidget(parent), node_(node), label_(nullptr)
{
    // Setup UI
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);
    label_->setStyleSheet("background-color: #2b2b2b; color: #ffffff; border: 1px solid #555;");
    label_->setText("Waiting for AI Stream...");

    label_->setFixedSize(640, 480); // Locks the label to 640x480
    this->setFixedSize(640, 480);  // Optional: Locks the whole widget if it's a standalone window
    layout->addWidget(label_);
    
    // Initialize ROS subscription (Topic will be set later or default)
    // We set a default topic here, but you can change it in MainWindow
    setTopic("/camera/ai_results");
}

CameraViewWidget::~CameraViewWidget()
{
    // Clean up is handled by ROS node shutdown
}

void CameraViewWidget::setTopic(const QString &topic)
{
    std::string topic_str = topic.toStdString();
    
    // Unsubscribe if already active
    if (sub_) {
        sub_.shutdown();
    }

    RCLCPP_INFO(node_->get_logger(), "Subscribing to: %s", topic_str.c_str());
    
    try {
        sub_ = image_transport::create_subscription(
            node_.get(),
            topic_str,
            std::bind(&CameraViewWidget::imageCallback, this, std::placeholders::_1),
            "raw", // Transport hint
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
    } catch (const std::exception &e) {
        handleError(std::string("Failed to subscribe: ") + e.what());
    }
}

void CameraViewWidget::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try {
        // 1. Convert ROS Image to OpenCV Mat
        // We assume the Python side sends "bgr8". If it sends "rgb8", change this string.
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat cv_mat = cv_ptr->image;

        // 2. Convert OpenCV Mat to QImage
        // OpenCV is BGR, QImage expects RGB for Format_RGB888
        cv::Mat rgb_mat;
        if (cv_mat.channels() == 3) {
            cv::cvtColor(cv_mat, rgb_mat, cv::COLOR_BGR2RGB);
        } else {
            // Fallback for grayscale if needed
            cv::cvtColor(cv_mat, rgb_mat, cv::COLOR_GRAY2RGB);
        }

        // Create QImage from the RGB Mat data
        // Note: We do NOT copy the data here to save performance, 
        // but we must ensure the Mat data stays valid. 
        // Since cv_ptr holds the shared pointer, the data is safe as long as cv_ptr is alive.
        // However, QImage constructor takes a pointer. We need to keep cv_ptr alive.
        // A safe way in a callback is to copy the data to QImage or use a shared_ptr wrapper.
        // For simplicity and stability in Qt, we will copy the data to a new QImage.
        
        QImage q_image(rgb_mat.data, rgb_mat.cols, rgb_mat.rows, 
                       rgb_mat.step, QImage::Format_RGB888);
        
        // Copy the image to a new QImage to detach from OpenCV memory management
        QImage final_img = q_image.copy();

        // 3. Update UI in the Main Thread
        // We use a signal/slot connection to ensure thread safety
        QMetaObject::invokeMethod(this, "updateImage", 
            Qt::QueuedConnection, 
            Q_ARG(QImage, final_img));

    } catch (const cv_bridge::Exception &e) {
        handleError("cv_bridge error: " + std::string(e.what()));
    } catch (const std::exception &e) {
        handleError("Image conversion error: " + std::string(e.what()));
    }
}

void CameraViewWidget::updateImage(const QImage &img)
{
    // Scale image to fit label while keeping aspect ratio
    // label_->setPixmap(QPixmap::fromImage(img).scaled(
    //     label_->size(), 
    //     Qt::KeepAspectRatio, 
    //     Qt::FastTransformation
    // ));

    // If the image is already 640x480, this happens instantly
    label_->setPixmap(QPixmap::fromImage(img));

    // Clear the "Waiting" text if we got an image
    if (!label_->text().contains("Waiting")) {
        // Optional: Add a timestamp or status
    }
}

void CameraViewWidget::handleError(const std::string &msg)
{
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    label_->setText(QString::fromStdString(msg));
    label_->setStyleSheet("background-color: #500; color: white; border: 1px solid red;");
    emit errorOccurred(QString::fromStdString(msg));
}
