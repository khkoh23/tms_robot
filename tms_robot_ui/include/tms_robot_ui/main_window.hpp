#ifndef TMS_ROBOT_UI_MAIN_WINDOW_HPP
#define TMS_ROBOT_UI_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QTreeWidget>
#include <QSplitter>
#include <QTimer>
#include <QDomDocument>
#include <QList>
#include <QMap>
#include <QString>
#include <QFile>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "tms_robot_ui/ros_bridge.hpp"
#include "tms_robot_ui/rviz_widget.hpp"
#include "tms_robot_ui/camera_view_widget.hpp"

class QLabel;
class QPushButton;
class QComboBox;
class QTextEdit;
class QTreeWidget;
class QTreeWidgetItem;
class QTimer;
class RosBridge;
class RvizWidget;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);
  ~MainWindow();

private slots:
  void onStartClicked();
  void onCancelClicked();
  void onTaskStateUpdated(const QString & task_name,
                          const QString & overall_state,
                          const QString & active_node,
                          const QString & message);
  void onBtNodeStatusUpdated(const QString & node_name,
                             const QString & node_type,
                             const QString & status);
  void onLogMessage(const QString & msg);

private:
  void buildUi();
  void connectSignals();
  void loadSelectedTree();
  QString selectedTaskName() const;
  QString selectedTreePath() const;
  void loadTreeIntoWidget(const QString & xml_path);
  QTreeWidgetItem * buildTreeItemFromDom(const class QDomElement & element);
  void setItemColor(QTreeWidgetItem * item, const QString & status);
  rclcpp::Node::SharedPtr node_;
  RosBridge * ros_bridge_;
  RvizWidget * rviz_widget_;
  CameraViewWidget *camera_view_widget_;
  QTimer * ros_spin_timer_;
  QLabel * state_label_;
  QLabel * active_node_label_;
  QComboBox * task_selector_;
  QPushButton * start_button_;
  QPushButton * cancel_button_;
  QTextEdit * log_text_;
  QTreeWidget * bt_tree_widget_;
  QMap<QString, QTreeWidgetItem *> node_items_;
};

#endif // TMS_ROBOT_UI_MAIN_WINDOW_HPP