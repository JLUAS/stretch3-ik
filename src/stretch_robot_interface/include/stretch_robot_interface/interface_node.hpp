#pragma once
#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "stretch_robot_interface/model.hpp"
namespace stretch_robot_interface {
// Spin in a single-threaded executor. No command publishers or service clients.
class InterfaceNode : public rclcpp::Node {
 public:
  explicit InterfaceNode(const std::string& name = "interface_validator");
  std::optional<RobotState> state() const;
  std::string validationReport() const;
  bool ready() const;

 protected:
  std::shared_ptr<RobotModel> model_;
  virtual void onState(const RobotState&) {}

 private:
  using Steady = std::chrono::steady_clock;
  static double age(Steady::time_point t);
  double timeout_, future_tolerance_;
  std::string base_, tcp_, model_xml_, error_ = "Waiting for joint state";
  std::optional<RobotState> state_;
  std::optional<bool> homed_, runstopped_;
  Steady::time_point received_{}, homed_received_{}, runstop_received_{},
      tf_received_{};
  bool tf_seen_ = false, static_seen_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homed_sub_, runstop_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_,
      static_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
}  // namespace stretch_robot_interface
