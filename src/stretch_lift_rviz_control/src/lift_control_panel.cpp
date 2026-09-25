#include "stretch_lift_rviz_control/lift_control_panel.hpp"

#include <cmath>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace stretch_lift_rviz_control {

LiftControlPanel::LiftControlPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout;
  auto title = new QLabel("REAL ROBOT — LIFT ONLY");
  title->setStyleSheet("font-weight:bold;color:#bb3030;font-size:18px");
  layout->addWidget(title);
  current_label_ = new QLabel("Current Lift: waiting");
  selected_label_ = new QLabel("Selected Target: waiting");
  state_label_ = new QLabel("State: waiting");
  progress_label_ = new QLabel("No execution active");
  for (auto* label : {current_label_, selected_label_, state_label_,
                      progress_label_}) {
    label->setWordWrap(true);
    layout->addWidget(label);
  }
  execute_ = new QPushButton("Execute Lift Target");
  cancel_ = new QPushButton("Cancel Lift Motion");
  reset_button_ = new QPushButton("Reset Target to Current");
  layout->addWidget(execute_);
  layout->addWidget(cancel_);
  layout->addWidget(reset_button_);
  layout->addStretch();
  setLayout(layout);
  cancel_->setEnabled(false);

  connect(execute_, &QPushButton::clicked, this,
          &LiftControlPanel::executeTarget);
  connect(cancel_, &QPushButton::clicked, this,
          &LiftControlPanel::cancelExecution);
  connect(reset_button_, &QPushButton::clicked, this,
          &LiftControlPanel::resetTarget);

  node_ = std::make_shared<rclcpp::Node>(
      "lift_control_panel", "/lift_control",
      rclcpp::NodeOptions().use_global_arguments(false));
  auto retained = rclcpp::QoS(1).reliable().transient_local();
  current_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/lift_control/current_position", retained,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        current_label_->setText(
            QString("Current Lift: %1 m").arg(msg->data, 0, 'f', 5));
      });
  selected_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/lift_control/selected_target", retained,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        selected_value_ = msg->data;
        have_selected_ = std::isfinite(selected_value_);
        selected_label_->setText(
            QString("Selected Target: %1 m").arg(msg->data, 0, 'f', 5));
        execute_->setEnabled(have_selected_ && !goal_);
      });
  state_ = node_->create_subscription<std_msgs::msg::String>(
      "/guarded_executor/lift_execution_status", retained,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        state_label_->setText("State: " + QString::fromStdString(msg->data));
      });
  action_ = rclcpp_action::create_client<Action>(
      node_, "/guarded_executor/execute_lift_target");
  reset_ = node_->create_client<std_srvs::srv::Trigger>(
      "/lift_control/reset_target");
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, [this]() {
    if (rclcpp::ok()) rclcpp::spin_some(node_);
  });
  timer_->start(30);
}

void LiftControlPanel::setBusy(bool busy) {
  execute_->setEnabled(!busy && have_selected_);
  cancel_->setEnabled(busy);
  reset_button_->setEnabled(!busy);
}

void LiftControlPanel::executeTarget() {
  if (!have_selected_ || goal_) return;
  if (!action_->action_server_is_ready()) {
    progress_label_->setText("BLOCKED: lift action server unavailable");
    return;
  }
  Action::Goal request;
  request.target_position = selected_value_;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this](GoalHandle::SharedPtr goal) {
    goal_ = goal;
    if (!goal_) {
      setBusy(false);
      progress_label_->setText("BLOCKED: target rejected");
    }
  };
  options.feedback_callback = [this](
                                  GoalHandle::SharedPtr,
                                  const std::shared_ptr<const Action::Feedback>
                                      feedback) {
    progress_label_->setText(
        QString("%1 — segment %2/%3 — measured %4 m → target %5 m")
            .arg(QString::fromStdString(feedback->state))
            .arg(feedback->current_segment)
            .arg(feedback->total_segments)
            .arg(feedback->current_position, 0, 'f', 5)
            .arg(feedback->target_position, 0, 'f', 5));
  };
  options.result_callback = [this](const GoalHandle::WrappedResult& wrapped) {
    const bool success =
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped.result && wrapped.result->success;
    progress_label_->setText(
        QString("%1: %2 — final %3 m")
            .arg(success ? "COMPLETE" : "ABORTED")
            .arg(wrapped.result
                     ? QString::fromStdString(wrapped.result->message)
                     : QString("no result"))
            .arg(wrapped.result ? wrapped.result->final_position : 0., 0, 'f',
                 5));
    goal_.reset();
    setBusy(false);
    if (success && reset_->service_is_ready())
      reset_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
  };
  setBusy(true);
  progress_label_->setText("Submitting explicit lift target…");
  action_->async_send_goal(request, options);
}

void LiftControlPanel::cancelExecution() {
  if (!goal_) return;
  progress_label_->setText("Cancel requested; waiting for physical stop…");
  action_->async_cancel_goal(goal_);
}

void LiftControlPanel::resetTarget() {
  if (!reset_->service_is_ready()) {
    progress_label_->setText("Marker reset service unavailable");
    return;
  }
  reset_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

}  // namespace stretch_lift_rviz_control

PLUGINLIB_EXPORT_CLASS(stretch_lift_rviz_control::LiftControlPanel,
                       rviz_common::Panel)
