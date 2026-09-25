#include "stretch_custom_kinematics/ik_status_panel.hpp"

#include <QFont>
#include <QScrollArea>
#include <QVBoxLayout>
#include <iomanip>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
namespace stretch_custom_kinematics {
namespace {
QString position(const geometry_msgs::msg::PoseStamped& p) {
  return QString("[%1, %2, %3] m")
      .arg(p.pose.position.x, 0, 'f', 5)
      .arg(p.pose.position.y, 0, 'f', 5)
      .arg(p.pose.position.z, 0, 'f', 5);
}
}  // namespace
IkStatusPanel::IkStatusPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout;
  summary_ = new QLabel("Waiting for IK preview");
  summary_->setWordWrap(true);
  layout->addWidget(summary_);
  auto task = new QLabel(
      "Move the TCP marker, then press Move to goal.\nTask: TCP XYZ + local Z "
      "direction\nSpin about target Z is unconstrained.");
  task->setWordWrap(true);
  layout->addWidget(task);
  target_label_ = new QLabel("Target TCP: waiting");
  target_label_->setWordWrap(true);
  layout->addWidget(target_label_);
  solved_label_ = new QLabel("Solved TCP: waiting");
  solved_label_->setWordWrap(true);
  layout->addWidget(solved_label_);
  details_ = new QLabel;
  details_->setWordWrap(true);
  details_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(details_);
  auto btn_execute = new QPushButton("Move to goal");
  execute_button_ = btn_execute;
  layout->addWidget(btn_execute);
  auto btn_stop = new QPushButton("DISARM / STOP");
  layout->addWidget(btn_stop);

  layout->addStretch();
  auto content = new QWidget;
  content->setLayout(layout);
  auto scroll = new QScrollArea;
  scroll->setWidget(content);
  scroll->setWidgetResizable(true);
  auto outer = new QVBoxLayout;
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(scroll);
  setLayout(outer);
  node_ = std::make_shared<rclcpp::Node>(
      "ik_status_panel", "/ik_preview",
      rclcpp::NodeOptions().use_global_arguments(false));
      
  arm_client_ = node_->create_client<std_srvs::srv::SetBool>("/guarded_executor/arm");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/guarded_executor/stop");
  validate_client_ = node_->create_client<std_srvs::srv::Trigger>("/guarded_executor/validate_preview");
  execute_client_ = node_->create_client<std_srvs::srv::Trigger>("/guarded_executor/move_to_preview");
  
  connect(btn_execute, &QPushButton::clicked, [this]() {
    callTriggerService(execute_client_);
  });
  connect(btn_stop, &QPushButton::clicked, [this]() {
    callTriggerService(stop_client_);
  });
  execution_label_ = new QLabel("Executor: waiting");
  execution_label_->setWordWrap(true);
  layout->insertWidget(layout->count() - 1, execution_label_);
  command_label_ = new QLabel;
  command_label_->setWordWrap(true);
  layout->insertWidget(layout->count() - 1, command_label_);
  execution_status_ = node_->create_subscription<std_msgs::msg::String>(
      "/guarded_executor/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const auto reason = msg->data.find(" reason=");
        const auto end = msg->data.find(" start=[", reason);
        execution_label_->setText(QString::fromStdString(
            msg->data.substr(0, msg->data.find(" hardware_client=")) + "\n" +
            (reason == std::string::npos ? "" : msg->data.substr(reason + 1, end - reason - 1))));
        execute_button_->setEnabled(msg->data.find("sequence_active=1") == std::string::npos &&
                                    msg->data.find("pending_goal=1") == std::string::npos);
      });
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  status_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/ik_preview/status", qos,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
        if (m->status.empty()) return;
        auto& s = m->status.front();
        summary_->setText(QString::fromStdString(s.message));
        summary_->setStyleSheet(
            s.level == 0 ? "font-weight:bold;color:#16833a;font-size:18px"
                         : "font-weight:bold;color:#bb3030;font-size:18px");
        std::map<std::string, std::string> v;
        for (auto p : s.values) v[p.key] = p.value;
        QString text;
        for (auto key : {"position_error_m", "direction_error_rad",
                         "full_orientation_error_rad", "solve_time_ms",
                         "iterations", "attempts"})
          text += QString::fromStdString(key) + ": " +
                  QString::fromStdString(v[key]) + "\n";
        text += "\n";
        text += s.level == 0 ? "Joint solution"
                             : "LAST VALID ghost (target unsolved)";
        std::istringstream values(v["last_valid_joint_solution"]);
        double lift, arm, yaw, pitch, roll;
        if (values >> lift >> arm >> yaw >> pitch >> roll) {
          text +=
              QString("\nlift=%1 m, arm=%2 m\nyaw=%3, pitch=%4, roll=%5 rad")
                  .arg(lift, 0, 'f', 5)
                  .arg(arm, 0, 'f', 5)
                  .arg(yaw, 0, 'f', 5)
                  .arg(pitch, 0, 'f', 5)
                  .arg(roll, 0, 'f', 5);
        }
        details_->setText(text);
      });
  target_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_preview/target_pose", qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
        target_label_->setText("Target TCP: " + position(*p));
      });
  solved_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_preview/solved_pose", qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
        solved_label_->setText("Solved TCP: " + position(*p));
      });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, [this]() {
    if (rclcpp::ok()) rclcpp::spin_some(node_);
  });
  timer_->start(50);
}

void IkStatusPanel::callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
  if (!client->wait_for_service(std::chrono::milliseconds(100))) {
    execution_label_->setText("Executor service unavailable");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
    const auto response = f.get();
    command_label_->setText(QString::fromStdString(response->message));
  });
}

void IkStatusPanel::callBoolService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value) {
  if (!client->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = value;
  client->async_send_request(req);
}

}  // namespace stretch_custom_kinematics
PLUGINLIB_EXPORT_CLASS(stretch_custom_kinematics::IkStatusPanel,
                       rviz_common::Panel)
