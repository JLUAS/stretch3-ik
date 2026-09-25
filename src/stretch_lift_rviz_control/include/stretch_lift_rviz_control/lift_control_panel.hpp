#pragma once

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "stretch_guarded_executor/action/execute_lift_target.hpp"

namespace stretch_lift_rviz_control {

class LiftControlPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit LiftControlPanel(QWidget* parent = nullptr);

 private Q_SLOTS:
  void executeTarget();
  void cancelExecution();
  void resetTarget();

 private:
  using Action = stretch_guarded_executor::action::ExecuteLiftTarget;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  void setBusy(bool busy);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr action_;
  GoalHandle::SharedPtr goal_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_, selected_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  QLabel *current_label_, *selected_label_, *state_label_, *progress_label_;
  QPushButton *execute_, *cancel_, *reset_button_;
  QTimer* timer_;
  double selected_value_ = 0;
  bool have_selected_ = false;
};

}  // namespace stretch_lift_rviz_control
