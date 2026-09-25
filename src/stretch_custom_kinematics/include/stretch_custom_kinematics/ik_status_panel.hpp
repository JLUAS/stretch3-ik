#pragma once
#include <QLabel>
#include <QTimer>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QPushButton>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
namespace stretch_custom_kinematics {
// One-click execution with a separate emergency stop control.
class IkStatusPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit IkStatusPanel(QWidget* parent = nullptr);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_,
      solved_;
  QLabel *summary_, *details_, *target_label_, *solved_label_;
  QTimer* timer_;
  QLabel* execution_label_;
  QLabel* command_label_;
  QPushButton* execute_button_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_status_;
  
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_, validate_client_, execute_client_;
  
  void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);
  void callBoolService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
};
}  // namespace stretch_custom_kinematics
