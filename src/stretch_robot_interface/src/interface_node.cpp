#include "stretch_robot_interface/interface_node.hpp"

#include <sstream>
namespace stretch_robot_interface {
double InterfaceNode::age(Steady::time_point t) {
  return std::chrono::duration<double>(Steady::now() - t).count();
}
InterfaceNode::InterfaceNode(const std::string& name) : Node(name) {
  timeout_ = declare_parameter("state_timeout", 1.0);
  if (!std::isfinite(timeout_) || timeout_ <= 0)
    throw std::invalid_argument("state_timeout must be positive");
  future_tolerance_ = declare_parameter("future_stamp_tolerance", 0.05);
  if (!std::isfinite(future_tolerance_) || future_tolerance_ < 0 ||
      future_tolerance_ > .2)
    throw std::invalid_argument(
        "future_stamp_tolerance must be finite and in [0, 0.2]");
  base_ = declare_parameter("base_frame", std::string("base_link"));
  tcp_ = declare_parameter("tcp_frame", std::string("link_grasp_center"));
  auto topic = declare_parameter("joint_state_topic",
                                 std::string("/stretch/joint_states"));
  description_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        if (m->data == model_xml_ && model_) return;
        model_.reset();
        state_.reset();
        try {
          model_ = std::make_shared<RobotModel>(m->data, base_, tcp_);
          model_xml_ = m->data;
          RCLCPP_INFO(get_logger(), "URDF accepted: %s -> %s", base_.c_str(),
                      tcp_.c_str());
        } catch (const std::exception& e) {
          error_ = e.what();
          RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
      });
  joints_ = create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        received_ = Steady::now();
        state_.reset();
        if (!model_) {
          error_ = "No valid robot_description";
          return;
        }
        try {
          state_ = model_->decode(*m);
          error_.clear();
          auto q = state();
          if (q && q->valid) onState(*q);
        } catch (const std::exception& e) {
          error_ = e.what();
        }
      });
  homed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/is_homed", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        homed_ = m->data;
        homed_received_ = Steady::now();
      });
  runstop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/is_runstopped", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        runstopped_ = m->data;
        runstop_received_ = Steady::now();
      });
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        if (!m->transforms.empty()) {
          tf_seen_ = true;
          tf_received_ = Steady::now();
        }
      });
  static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        static_seen_ = static_seen_ || !m->transforms.empty();
      });
  report_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
    RCLCPP_INFO(get_logger(), "%s", validationReport().c_str());
  });
}
std::optional<RobotState> InterfaceNode::state() const {
  if (!state_) return std::nullopt;
  auto q = *state_;
  q.homed = homed_.value_or(false);
  q.runstopped = runstopped_.value_or(true);
  double stamp_age =
      (now() - rclcpp::Time(q.timestamp, get_clock()->get_clock_type()))
          .seconds();
  q.valid = fresh(age(received_), stamp_age, timeout_, future_tolerance_) &&
            homed_.has_value() &&
            runstopped_.has_value() && age(homed_received_) <= timeout_ &&
            age(runstop_received_) <= timeout_;
  return q;
}
bool InterfaceNode::ready() const {
  auto q = state();
  return model_ && q && q->valid && tf_seen_ && age(tf_received_) <= timeout_ &&
         static_seen_;
}
std::string InterfaceNode::validationReport() const {
  auto q = state();
  std::ostringstream s;
  s << "READ-ONLY interface " << (ready() ? "PASS" : "WAIT/FAIL")
    << " model=" << bool(model_) << " joints=" << bool(state_)
    << " fresh_valid=" << (q && q->valid)
    << " tf=" << (tf_seen_ && age(tf_received_) <= timeout_)
    << " tf_static=" << static_seen_ << " homed_read=" << homed_.has_value()
    << " runstop_read=" << runstopped_.has_value();
  if (q)
    s << " homed=" << q->homed << " runstopped=" << q->runstopped << " q=["
      << q->lift << ", " << q->arm_extension << ", " << q->wrist_yaw << ", "
      << q->wrist_pitch << ", " << q->wrist_roll << "]";
  if (!error_.empty()) s << " reason=" << error_;
  return s.str();
}
}  // namespace stretch_robot_interface
