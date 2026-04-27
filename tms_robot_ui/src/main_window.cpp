#include "tms_robot_ui/main_window.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QBrush>
#include <QDateTime>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QMainWindow(parent),
  node_(node),
  ros_bridge_(new RosBridge(node, this)),
  rviz_widget_(nullptr),
  camera_view_widget_(nullptr),
  ros_spin_timer_(nullptr)
{
  buildUi();
  connectSignals();
  loadSelectedTree();
  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_spin_timer_->start(20);
}

MainWindow::~MainWindow() = default;

void MainWindow::buildUi() {
  resize(1500, 900);
  setWindowTitle("Robot Console UI");
  auto * central = new QWidget(this);
  auto * layout = new QHBoxLayout(central);
  auto * splitter = new QSplitter(Qt::Horizontal, central);
  auto * left = new QWidget(splitter);
  auto * left_layout = new QVBoxLayout(left);
  task_selector_ = new QComboBox(left);
  task_selector_->addItem("inspect");
  task_selector_->addItem("patrol");
  task_selector_->addItem("move_arm_home");
  start_button_ = new QPushButton("Start Task", left);
  cancel_button_ = new QPushButton("Cancel Task", left);
  state_label_ = new QLabel("State: IDLE", left);
  active_node_label_ = new QLabel("Active Node: -", left);
  bt_tree_widget_ = new QTreeWidget(left);
  bt_tree_widget_->setColumnCount(2);
  bt_tree_widget_->setHeaderLabels({"BT Node", "Status"});
  bt_tree_widget_->header()->setSectionResizeMode(QHeaderView::Stretch);
  log_text_ = new QTextEdit(left);
  log_text_->setReadOnly(true);
  left_layout->addWidget(new QLabel("<b>Tasks</b>", left));
  left_layout->addWidget(task_selector_);
  left_layout->addWidget(start_button_);
  left_layout->addWidget(cancel_button_);
  left_layout->addWidget(state_label_);
  left_layout->addWidget(active_node_label_);
  left_layout->addWidget(new QLabel("<b>Behavior Tree</b>", left));
  left_layout->addWidget(bt_tree_widget_, 1);
  left_layout->addWidget(new QLabel("<b>Log</b>", left));
  left_layout->addWidget(log_text_, 1);
  auto * right_splitter = new QSplitter(Qt::Vertical, splitter);
  rviz_widget_ = new RvizWidget(right_splitter);
  camera_view_widget_ = new CameraViewWidget(node_, right_splitter);
  camera_view_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  right_splitter->setStretchFactor(0, 5); //50%
  right_splitter->setStretchFactor(1, 5); //50%
  splitter->addWidget(left);
  splitter->addWidget(right_splitter);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  // splitter->setSizes({600, 100});
  layout->addWidget(splitter);
  setCentralWidget(central);
}

void MainWindow::connectSignals() {
  connect(start_button_, &QPushButton::clicked, this, &MainWindow::onStartClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &MainWindow::onCancelClicked);
  connect(task_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { loadSelectedTree(); });
  connect(ros_bridge_, &RosBridge::taskStateUpdated, this, &MainWindow::onTaskStateUpdated); 
  connect(ros_bridge_, &RosBridge::btNodeStatusUpdated, this, &MainWindow::onBtNodeStatusUpdated);
  connect(ros_bridge_, &RosBridge::logMessage, this, &MainWindow::onLogMessage);
}

QString MainWindow::selectedTaskName() const {
  return task_selector_->currentText();
}

QString MainWindow::selectedTreePath() const {
  const auto share_dir = QString::fromStdString(ament_index_cpp::get_package_share_directory("tms_robot_control"));
  if (selectedTaskName() == "inspect") {
    return share_dir + "/tree/inspect_tree.xml";
  }
  if (selectedTaskName() == "patrol") {
    return share_dir + "/tree/patrol_tree.xml";
  }
  return share_dir + "/tree/move_arm_home.xml";
}

void MainWindow::loadSelectedTree() {
  loadTreeIntoWidget(selectedTreePath());
}

void MainWindow::onStartClicked() {
  ros_bridge_->startTask(selectedTaskName());
}

void MainWindow::onCancelClicked() {
  ros_bridge_->cancelTask();
}

void MainWindow::onTaskStateUpdated(const QString &,
                                    const QString & overall_state,
                                    const QString & active_node,
                                    const QString & message) {
  state_label_->setText("State: " + overall_state);
  active_node_label_->setText("Active Node: " + active_node);
  if (!message.isEmpty()) {
    onLogMessage(message);
  }
}

void MainWindow::onBtNodeStatusUpdated(const QString & node_name,
                                       const QString &,
                                       const QString & status) {
  if (!node_items_.contains(node_name)) {
    return;
  }
  auto * item = node_items_[node_name];
  item->setText(1, status);
  setItemColor(item, status);
}

void MainWindow::onLogMessage(const QString & msg) {
  const auto now = QDateTime::currentDateTime().toString("hh:mm:ss");
  log_text_->append(QString("[%1] %2").arg(now, msg));
}

void MainWindow::loadTreeIntoWidget(const QString & xml_path) {
  bt_tree_widget_->clear();
  node_items_.clear();
  QFile file(xml_path);
  if (!file.open(QIODevice::ReadOnly)) {
    return;
  }
  QDomDocument doc;
  if (!doc.setContent(&file)) {
    return;
  }
  const auto root = doc.documentElement();
  QDomElement tree_elem;
  auto child = root.firstChild();
  while (!child.isNull()) {
    if (child.isElement() && child.toElement().tagName() == "BehaviorTree") {
      tree_elem = child.toElement();
      break;
    }
    child = child.nextSibling();
  }
  if (tree_elem.isNull()) {
    return;
  }
  auto node = tree_elem.firstChild();
  while (!node.isNull()) {
    if (node.isElement()) {
      auto * top = buildTreeItemFromDom(node.toElement());
      if (top) {
        bt_tree_widget_->addTopLevelItem(top);
      }
    }
    node = node.nextSibling();
  }
  bt_tree_widget_->expandAll();
}

QTreeWidgetItem * MainWindow::buildTreeItemFromDom(const QDomElement & element) {
  QString node_name = element.hasAttribute("name") ? element.attribute("name") : element.tagName();
  auto * item = new QTreeWidgetItem({node_name, "IDLE"});
  node_items_[node_name] = item;
  auto child = element.firstChild();
  while (!child.isNull()) {
    if (child.isElement()) {
      auto * sub = buildTreeItemFromDom(child.toElement());
      if (sub) {
        item->addChild(sub);
      }
    }
    child = child.nextSibling();
  }
  return item;
}

void MainWindow::setItemColor(QTreeWidgetItem * item, const QString & status) {
  QColor color = Qt::white;
  if (status == "RUNNING") color = QColor(255, 235, 130);
  else if (status == "SUCCESS") color = QColor(170, 255, 170);
  else if (status == "FAILURE") color = QColor(255, 170, 170);
  item->setBackground(0, QBrush(color));
  item->setBackground(1, QBrush(color));
}
