#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <iomanip>
#include <interactive_markers/interactive_marker_server.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "stretch_custom_kinematics/forward_kinematics.hpp"
#include "stretch_custom_kinematics/inverse_kinematics.hpp"
#include "stretch_robot_interface/interface_node.hpp"

namespace {
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using Marker = visualization_msgs::msg::Marker;
using stretch_custom_kinematics::ForwardKinematics;
using stretch_custom_kinematics::JointBounds;
using stretch_robot_interface::RobotState;

class LiftMarker final : public stretch_robot_interface::InterfaceNode {
 public:
  LiftMarker() : InterfaceNode("lift_marker") {
    if (std::string(get_namespace()) != "/lift_control")
      throw std::invalid_argument("Lift marker requires /lift_control namespace");
    if (get_parameter("joint_state_topic").as_string() !=
        "/stretch/joint_states")
      throw std::invalid_argument(
          "Lift marker requires canonical /stretch/joint_states");

    const auto share = ament_index_cpp::get_package_share_directory(
        "stretch_custom_kinematics");
    hardware_ = JointBounds::load(declare_parameter(
        "hardware_limits_file", share + "/config/hardware_limits.csv"));
    auto retained = rclcpp::QoS(1).reliable().transient_local();
    current_pub_ = create_publisher<std_msgs::msg::Float64>(
        "current_position", retained);
    selected_pub_ = create_publisher<std_msgs::msg::Float64>(
        "selected_target", retained);
    state_pub_ = create_publisher<std_msgs::msg::String>("marker_status",
                                                         retained);
    visual_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization", retained);
    execution_sub_ = create_subscription<std_msgs::msg::String>(
        "/guarded_executor/lift_execution_status", retained,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          execution_status_ = message->data;
          executing_ = execution_status_.find("active=1") != std::string::npos;
          if (execution_status_.find("state=COMPLETE") != std::string::npos)
            reset_pending_ = true;
          publishStatus();
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_target",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (!measured_) {
            response->message = "fresh measured lift unavailable";
            return;
          }
          selectMeasured();
          response->success = true;
          response->message = "selected target reset to measured lift";
        });

    for (const auto* topic : {"/lift_control/target/update",
                              "/lift_control/target/feedback"})
      if (get_node_topics_interface()->resolve_topic_name(topic).rfind(
              "/lift_control/", 0) != 0)
        throw std::invalid_argument("Unsafe lift marker remap");
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "/lift_control/target", this);
  }

 protected:
  void onState(const RobotState& measured) override {
    measured_ = measured;
    if (!model_) return;
    if (model_identity_ != model_.get()) {
      model_identity_ = model_.get();
      bounds_ = hardware_.intersectModel(*model_);
      bounds_ready_ = true;
      marker_created_ = false;
      server_->clear();
      server_->applyChanges();
      RCLCPP_INFO(get_logger(),
                  "Runtime lift selection range [%.9f, %.9f] m",
                  bounds_.lower[0], bounds_.upper[0]);
    }
    std_msgs::msg::Float64 current;
    current.data = measured.lift;
    current_pub_->publish(current);

    if (reset_pending_ || !selected_) {
      reset_pending_ = false;
      selected_ = measured.lift;
    }
    selected_ = std::clamp(*selected_, bounds_.lower[0], bounds_.upper[0]);
    if (!dragging_) updateInteractivePose();
    publishSelected();
    publishVisuals();
  }

 private:
  std::string baseFrame() const { return model_ ? model_->base() : "base_link"; }

  geometry_msgs::msg::Pose poseAt(double lift) const {
    auto target = *measured_;
    target.lift = lift;
    return ForwardKinematics(model_).computeTcpPose(target);
  }

  void createMarker(const geometry_msgs::msg::Pose& pose) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = baseFrame();
    marker.name = "lift_target";
    marker.description = "Selected lift target — Z only";
    marker.scale = .24;
    marker.pose = pose;

    visualization_msgs::msg::InteractiveMarkerControl visual;
    visual.always_visible = true;
    Marker sphere;
    sphere.type = Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = .045;
    sphere.color.r = .95;
    sphere.color.g = .65;
    sphere.color.b = .05;
    sphere.color.a = .95;
    sphere.pose.orientation.w = 1.;
    visual.markers.push_back(sphere);
    marker.controls.push_back(visual);

    visualization_msgs::msg::InteractiveMarkerControl z;
    z.name = "move_lift_z_only";
    z.orientation.w = 1.;
    z.orientation.y = 1.;
    z.orientation_mode = z.FIXED;
    z.interaction_mode = z.MOVE_AXIS;
    marker.controls.push_back(z);

    server_->insert(marker, [this](Feedback::ConstSharedPtr feedback) {
      processFeedback(feedback);
    });
    server_->applyChanges();
    marker_created_ = true;
  }

  void updateInteractivePose() {
    if (!measured_ || !selected_ || !bounds_ready_) return;
    const auto pose = poseAt(*selected_);
    if (!marker_created_)
      createMarker(pose);
    else {
      server_->setPose("lift_target", pose);
      server_->applyChanges();
    }
  }

  void processFeedback(const Feedback::ConstSharedPtr& feedback) {
    if (!feedback || !measured_ || !bounds_ready_ || executing_) {
      if (feedback && executing_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Target editing blocked during lift execution");
      updateInteractivePose();
      return;
    }
    if (feedback->header.frame_id != baseFrame()) {
      RCLCPP_ERROR(get_logger(), "Rejected marker feedback in frame '%s'",
                   feedback->header.frame_id.c_str());
      updateInteractivePose();
      return;
    }
    if (feedback->event_type == Feedback::MOUSE_DOWN) {
      dragging_ = true;
      drag_start_lift_ = *selected_;
      drag_start_z_ = poseAt(*selected_).position.z;
      return;
    }
    if (feedback->event_type == Feedback::POSE_UPDATE) {
      if (!dragging_) {
        dragging_ = true;
        drag_start_lift_ = *selected_;
        drag_start_z_ = poseAt(*selected_).position.z;
      }
      const double requested =
          drag_start_lift_ + feedback->pose.position.z - drag_start_z_;
      const double clamped =
          std::clamp(requested, bounds_.lower[0], bounds_.upper[0]);
      if (std::abs(clamped - requested) > 1e-12)
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Lift target clamped to runtime range [%.6f, %.6f] m",
            bounds_.lower[0], bounds_.upper[0]);
      selected_ = clamped;
      publishSelected();
      publishVisuals();
      server_->setPose("lift_target", poseAt(*selected_));
      server_->applyChanges();
      return;
    }
    if (feedback->event_type == Feedback::MOUSE_UP) {
      dragging_ = false;
      updateInteractivePose();
      publishStatus();
    }
  }

  void selectMeasured() {
    if (!measured_ || !bounds_ready_) return;
    dragging_ = false;
    selected_ = std::clamp(measured_->lift, bounds_.lower[0], bounds_.upper[0]);
    updateInteractivePose();
    publishSelected();
    publishVisuals();
    publishStatus();
  }

  void publishSelected() {
    if (!selected_) return;
    std_msgs::msg::Float64 selected;
    selected.data = *selected_;
    selected_pub_->publish(selected);
  }

  Marker marker(int id, int type) const {
    Marker result;
    result.header.frame_id = baseFrame();
    result.header.stamp = now();
    result.ns = "lift_control";
    result.id = id;
    result.type = type;
    result.action = Marker::ADD;
    result.pose.orientation.w = 1.;
    result.color.a = 1.;
    return result;
  }

  void publishVisuals() {
    if (!measured_ || !selected_) return;
    visualization_msgs::msg::MarkerArray array;
    auto current = marker(0, Marker::SPHERE);
    current.pose = poseAt(measured_->lift);
    current.scale.x = current.scale.y = current.scale.z = .03;
    current.color.r = .05;
    current.color.g = .8;
    current.color.b = 1.;
    array.markers.push_back(current);

    auto line = marker(1, Marker::LINE_LIST);
    line.scale.x = .008;
    line.color.r = .9;
    line.color.g = .7;
    line.color.b = .1;
    line.points = {current.pose.position, poseAt(*selected_).position};
    array.markers.push_back(line);

    auto text = marker(2, Marker::TEXT_VIEW_FACING);
    text.pose = poseAt(*selected_);
    text.pose.position.z += .09;
    text.scale.z = .032;
    text.color.r = 1.;
    text.color.g = 1.;
    text.color.b = 1.;
    std::ostringstream label;
    label << std::fixed << std::setprecision(3) << "Lift target " << *selected_
          << " m\nMeasured " << measured_->lift << " m";
    text.text = label.str();
    array.markers.push_back(text);
    visual_pub_->publish(array);
  }

  void publishStatus() {
    std_msgs::msg::String status;
    std::ostringstream text;
    text << (executing_ ? "EXECUTING" : "TARGET_SELECTION")
         << " current=" << (measured_ ? measured_->lift : NAN)
         << " selected=" << (selected_ ? *selected_ : NAN);
    if (!execution_status_.empty()) text << " executor={" << execution_status_ << "}";
    status.data = text.str();
    state_pub_->publish(status);
  }

  JointBounds hardware_, bounds_;
  const void* model_identity_ = nullptr;
  bool bounds_ready_ = false, marker_created_ = false, dragging_ = false,
       executing_ = false, reset_pending_ = false;
  double drag_start_lift_ = 0., drag_start_z_ = 0.;
  std::optional<RobotState> measured_;
  std::optional<double> selected_;
  std::string execution_status_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_,
      selected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LiftMarker>());
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
