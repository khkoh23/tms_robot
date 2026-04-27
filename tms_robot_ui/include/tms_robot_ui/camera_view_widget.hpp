#ifndef TMS_ROBOT_UI_CAMERA_VIEW_WIDGET_HPP
#define TMS_ROBOT_UI_CAMERA_VIEW_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraViewWidget(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~CameraViewWidget() override;

    void setTopic(const QString &topic);

signals:
    void errorOccurred(const QString &msg);

private slots:
    void updateImage(const QImage &img);

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void handleError(const std::string &msg);

    rclcpp::Node::SharedPtr node_;
    image_transport::Subscriber sub_;
    QLabel *label_;
    QTimer *timer_; // Optional: for fallback if no image arrives

    // Thread safety: We emit a signal to update the UI in the main thread
    // The slot 'updateImage' will be connected to this signal
};

#endif // TMS_ROBOT_UI_CAMERA_VIEW_WIDGET_HPP
