#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "stretch_custom_kinematics/inverse_kinematics.hpp"
#include "stretch_robot_interface/interface_node.hpp"
using namespace stretch_custom_kinematics;
using Marker = visualization_msgs::msg::Marker;
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
class IkPreview : public stretch_robot_interface::InterfaceNode {
 public:
  IkPreview() : InterfaceNode("ik_preview") {
    if (std::string(get_namespace()) != "/ik_preview")
      throw std::invalid_argument(
          "Preview node must run in namespace /ik_preview");
    auto share = ament_index_cpp::get_package_share_directory(
        "stretch_custom_kinematics");
    hardware_ = JointBounds::load(declare_parameter(
        "hardware_limits_file", share + "/config/hardware_limits.csv"));
    meshes_ = declare_parameter("mesh_package_path", std::string(""));
    grasp_approach_tolerance_ =
        declare_parameter("grasp_approach_tolerance", 0.12);
    max_base_offset_ = declare_parameter("max_base_offset", 0.6);
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    joints_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    tf_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    static_tf_ =
        create_publisher<tf2_msgs::msg::TFMessage>("tf_static", latched);
    description_ =
        create_publisher<std_msgs::msg::String>("robot_description", latched);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers",
                                                                      latched);
    status_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status",
                                                                      latched);
    target_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose",
                                                                latched);
    solved_ = create_publisher<geometry_msgs::msg::PoseStamped>("solved_pose",
                                                                latched);
    // Check resolved topics, so accidental command-line remapping cannot put
    // preview states or TF on robot topics, even outside the supported launch.
    for (auto p : {std::string(joints_->get_topic_name()),
                   std::string(description_->get_topic_name()),
                   std::string(markers_->get_topic_name()),
                   std::string(status_->get_topic_name()),
                   std::string(target_->get_topic_name()),
                   std::string(solved_->get_topic_name())})
      if (p.rfind("/ik_preview/", 0) != 0)
        throw std::invalid_argument("Unsafe preview publisher remap: " + p);
    for (const auto* name :
         {"/ik_preview/target/update", "/ik_preview/target/feedback"}) {
      if (get_node_topics_interface()->resolve_topic_name(name).rfind(
              "/ik_preview/", 0) != 0)
        throw std::invalid_argument("Unsafe interactive marker remap");
    }
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "/ik_preview/target", this);
    model_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", latched,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          if (xml_ != m->data) {
            xml_ = m->data;
            ik_.reset();
            have_target_ = false;
            last_solution_.reset();
            server_->clear();
            server_->applyChanges();
          }
        });
    raw_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_state_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          if (m->name.size() == m->position.size())
            for (size_t i = 0; i < m->name.size(); ++i)
              if (std::isfinite(m->position[i]))
                raw_[m->name[i]] = m->position[i];
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          setExternalGoal(*message, false);
        });
    grasp_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "grasp_goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          setExternalGoal(*message, true);
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });

    arm_client_ = create_client<std_srvs::srv::SetBool>("/guarded_executor/arm");
    stop_client_ = create_client<std_srvs::srv::Trigger>("/guarded_executor/stop");
    validate_client_ = create_client<std_srvs::srv::Trigger>("/guarded_executor/validate_preview");
    execute_client_ = create_client<std_srvs::srv::Trigger>("/guarded_executor/move_to_preview");

    menu_handler_.insert("Move to goal", [this](const interactive_markers::MenuHandler::FeedbackConstPtr&) {
      callTriggerService(execute_client_);
    });
    menu_handler_.insert("DISARM / STOP", [this](const interactive_markers::MenuHandler::FeedbackConstPtr&) {
      callTriggerService(stop_client_);
    });
  }

 private:
  // Grasp goals constrain position, approach (local X) and finger axis
  // (local Y); other goals keep the XYZ + local-Z task.
  void setExternalGoal(const geometry_msgs::msg::PoseStamped& message,
                       bool grasp) {
    if (!model_ || !have_target_) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring external goal until IK preview is ready");
      return;
    }
    const auto& p = message.pose.position;
    const auto& q = message.pose.orientation;
    const double q_norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    // The preview base mirrors the robot base, so both names are the same frame.
    const bool known_frame = message.header.frame_id == frame() ||
                             message.header.frame_id == model_->base();
    if (!known_frame || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z) || !std::isfinite(q_norm) || q_norm < 1e-6) {
      RCLCPP_ERROR(get_logger(),
                   "Rejected external IK goal: expected finite pose in %s",
                   frame().c_str());
      return;
    }
    target_pose_ = message.pose;
    target_pose_.orientation.x /= q_norm;
    target_pose_.orientation.y /= q_norm;
    target_pose_.orientation.z /= q_norm;
    target_pose_.orientation.w /= q_norm;
    grasp_mode_ = grasp;
    goal_stamp_ns_ = int64_t(message.header.stamp.sec) * 1000000000 +
                     message.header.stamp.nanosec;
    bad_frame_ = false;
    dirty_ = true;
    server_->setPose("target_tcp", target_pose_);
    server_->applyChanges();
  }
  void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service %s is not available (is the executor running?)", client->get_service_name());
      return;
    }
    RCLCPP_INFO(get_logger(), "Calling service: %s", client->get_service_name());
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto res = future.get();
        RCLCPP_INFO(get_logger(), "Response: %s (%s)", res->success ? "SUCCESS" : "FAIL", res->message.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
      }
    });
  }

  void callBoolService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service %s is not available", client->get_service_name());
      return;
    }
    RCLCPP_INFO(get_logger(), "Calling service: %s (%s)", client->get_service_name(), value ? "true" : "false");
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = value;
    client->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      try {
        auto res = future.get();
        RCLCPP_INFO(get_logger(), "Response: %s (%s)", res->success ? "SUCCESS" : "FAIL", res->message.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
      }
    });
  }

  static geometry_msgs::msg::Pose poseMsg(const Eigen::Isometry3d& t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();
    Eigen::Quaterniond q(t.linear());
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
  }
  std::string frame() const { return "ik_preview/" + model_->base(); }
  void initialize() {
    IkOptions options;
    options.state_timeout = get_parameter("state_timeout").as_double();
    options.approach_tolerance = grasp_approach_tolerance_;
    options.max_base_offset = max_base_offset_;
    ik_ = std::make_unique<InverseKinematics>(model_, hardware_, options);
    if (!display_model_.initString(xml_))
      throw std::invalid_argument("Invalid display model");
    auto description = xml_;
    if (!meshes_.empty()) {
      const std::string from = "package://stretch_description/",
                        to = "file://" +
                             std::filesystem::absolute(meshes_).string() + "/";
      size_t at = 0;
      while ((at = description.find(from, at)) != std::string::npos) {
        description.replace(at, from.size(), to);
        at += to.size();
      }
    }
    std_msgs::msg::String msg;
    msg.data = description;
    description_->publish(msg);
    const auto q = state();
    if (!q || !q->valid) return;
    target_pose_ = ForwardKinematics(model_).computeTcpPose(*q);
    createInteractiveTarget();
    have_target_ = true;
    dirty_ = true;
  }
  void createInteractiveTarget() {
    visualization_msgs::msg::InteractiveMarker m;
    m.header.frame_id = frame();
    m.name = "target_tcp";
    m.description = "Target TCP";
    m.scale = .25;
    m.pose = target_pose_;
    visualization_msgs::msg::InteractiveMarkerControl visual;
    visual.always_visible = true;
    Marker sphere;
    sphere.type = Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = .035;
    sphere.color.r = 1;
    sphere.color.g = .6;
    sphere.color.a = .9;
    sphere.pose.orientation.w = 1;
    visual.markers.push_back(sphere);
    m.controls.push_back(visual);
    for (int axis = 0; axis < 3; ++axis)
      for (bool rotate : {false, true}) {
        visualization_msgs::msg::InteractiveMarkerControl c;
        c.name =
            std::string(rotate ? "rotate_" : "move_") + std::to_string(axis);
        c.orientation.w = 1;
        if (axis == 0)
          c.orientation.x = 1;
        else if (axis == 1)
          c.orientation.z = 1;
        else
          c.orientation.y = 1;
        c.orientation_mode = c.FIXED;
        c.interaction_mode = rotate ? c.ROTATE_AXIS : c.MOVE_AXIS;
        m.controls.push_back(c);
      }
    server_->insert(m, [this](Feedback::ConstSharedPtr f) {
      if (f->event_type != Feedback::POSE_UPDATE) return;
      target_pose_ = f->pose;
      grasp_mode_ = false;
      goal_stamp_ns_ = 0;
      bad_frame_ = f->header.frame_id != frame();
      dirty_ = true;
      server_->setPose("target_tcp", target_pose_);
      server_->applyChanges();
    });
    menu_handler_.apply(*server_, "target_tcp");
    server_->applyChanges();
  }
  void tick() {
    if (!model_ || xml_.empty()) {
      publishWaiting("Waiting for live model and current state");
      return;
    }
    auto current = state();
    if (!ik_ || !have_target_) {
      if (!current || !current->valid) {
        publishWaiting("stale state: waiting for current logical seed");
        return;
      }
      try {
        initialize();
      } catch (const std::exception& e) {
        publishWaiting(e.what());
        return;
      }
    }
    if (!current || !current->valid) {
      result_.success = false;
      result_.reason = IkFailure::StaleState;
      result_.position_error = result_.direction_error =
          result_.orientation_error = INFINITY;
      result_.solve_time_ms = 0;
      result_.iterations = 0;
      base_dx_ = NAN;  // a correction from an earlier solve is not an answer
      dirty_ = true;
      publishDisplay();
      return;
    }
    // The executor requires the measured start to match the IK seed (1 mm,
    // 5 mrad); re-solve the same target when the arm drifts from the seed
    // (after a stop, a mode switch, gravity) so the candidate stays usable.
    if (!dirty_ && last_seed_ && last_solution_) {
      const JointVector drift =
          (coordinates(*current) - coordinates(*last_seed_)).cwiseAbs();
      const JointVector limit =
          (JointVector() << .0005, .0005, .0025, .0025, .0025).finished();
      if ((drift.array() > limit.array()).any()) dirty_ = true;
    }
    if (dirty_) {
      last_seed_ = *current;
      if (bad_frame_) {
        result_ = IkResult();
        result_.reason = IkFailure::InvalidTarget;
      } else {
        const double age = (now() - rclcpp::Time(current->timestamp,
                                                 get_clock()->get_clock_type()))
                               .seconds();
        base_dx_ = NAN;
        if (!grasp_mode_) {
          result_ = ik_->solve(target_pose_, *current, age);
        } else {
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          const auto& o = target_pose_.orientation;
          t.linear() = Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
          t.translation() = Eigen::Vector3d(target_pose_.position.x,
                                            target_pose_.position.y,
                                            target_pose_.position.z);
          result_ = ik_->solveGrasp(t, *current, age, false);
          base_dx_ = 0;
          if (!result_.success) {
            // Only reported: the executor never runs a moved-base solution.
            auto moved = ik_->solveGrasp(t, *current, age, true);
            base_dx_ = moved.success ? moved.base_dx : NAN;
          }
        }
      }
      if (result_.success) {
        last_solution_ = result_.solution;
        solved_pose_ =
            ForwardKinematics(model_).computeTcpPose(result_.solution);
      }
      const auto start = coordinates(*current);
      const auto goal = result_.success ? coordinates(result_.solution)
                                        : JointVector::Constant(NAN);
      RCLCPP_INFO(
          get_logger(),
          "[IK_TRACE] target_xyz=[%.6f %.6f %.6f] target_frame=%s "
          "q_start=[%.6f %.6f %.6f %.6f %.6f] "
          "q_goal=[%.6f %.6f %.6f %.6f %.6f] status=%s",
          target_pose_.position.x, target_pose_.position.y,
          target_pose_.position.z, frame().c_str(), start[0], start[1],
          start[2], start[3], start[4], goal[0], goal[1], goal[2], goal[3],
          goal[4], failureName(result_.reason));
      RCLCPP_INFO(get_logger(),
                  "[LIFT_TRACE] stage=ik target_z=%.6f target_frame=%s "
                  "transformed_z=%.6f ik_lift=%.6f status=%s",
                  target_pose_.position.z, frame().c_str(),
                  target_pose_.position.z, goal[0], failureName(result_.reason));
      dirty_ = false;
    }
    if (last_solution_) publishGhost(*last_solution_);
    publishDisplay();
  }
  void publishGhost(const RobotState& q) {
    auto values = raw_;
    for (auto [name, v] : model_->physicalJoints(q)) values[name] = v;
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    joints.header.frame_id = frame();
    tf2_msgs::msg::TFMessage dynamic, statics;
    for (const auto& [name, j] : display_model_.joints_) {
      double v = values.count(name) ? values.at(name) : 0;
      auto t = stretch_robot_interface::RobotModel::origin(
          j->parent_to_joint_origin_transform);
      if (j->type != urdf::Joint::FIXED) {
        joints.name.push_back(name);
        joints.position.push_back(v);
        Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
        Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
        if (j->type == urdf::Joint::PRISMATIC)
          motion.translation() = axis * v;
        else if (j->type == urdf::Joint::REVOLUTE ||
                 j->type == urdf::Joint::CONTINUOUS)
          motion.linear() = Eigen::AngleAxisd(v, axis).toRotationMatrix();
        t = t * motion;
      }
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now();
      tf.header.frame_id = "ik_preview/" + j->parent_link_name;
      tf.child_frame_id = "ik_preview/" + j->child_link_name;
      auto p = poseMsg(t);
      tf.transform.translation.x = p.position.x;
      tf.transform.translation.y = p.position.y;
      tf.transform.translation.z = p.position.z;
      tf.transform.rotation = p.orientation;
      (j->type == urdf::Joint::FIXED ? statics : dynamic)
          .transforms.push_back(tf);
    }
    joints_->publish(joints);
    tf_->publish(dynamic);
    static_tf_->publish(statics);
  }
  static std::string number(double value) {
    std::ostringstream s;
    s << std::setprecision(9) << value;
    return s.str();
  }
  Marker marker(int id, int type, const geometry_msgs::msg::Pose& p) const {
    Marker m;
    m.header.frame_id = frame();
    m.header.stamp = now();
    m.ns = "ik_preview";
    m.id = id;
    m.type = type;
    m.pose = p;
    m.color.a = 1;
    return m;
  }
  void publishWaiting(const std::string& message) {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "ik_preview";
    s.level = s.WARN;
    s.message = message;
    a.status.push_back(s);
    status_->publish(a);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                         message.c_str());
  }
  // A grasp the fixed base cannot reach but a base move can is not an
  // error: say so instead of the fixed-base failure (often "joint limit").
  std::string statusText() const {
    if (grasp_mode_ && !result_.success && std::isfinite(base_dx_)) {
      std::ostringstream s;
      s << "base move needed: dx=" << std::showpos << std::fixed
        << std::setprecision(3) << base_dx_ << " m";
      return s.str();
    }
    return failureName(result_.reason);
  }
  void publishDisplay() {
    diagnostic_msgs::msg::DiagnosticArray status;
    status.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus entry;
    entry.name = "ik_preview";
    entry.level = result_.success ? entry.OK : entry.ERROR;
    entry.message = statusText();
    auto add = [&](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue p;
      p.key = key;
      p.value = value;
      entry.values.push_back(p);
    };
    add("success", result_.success ? "true" : "false");
    add("position_error_m", number(result_.position_error));
    add("direction_error_rad", number(result_.direction_error));
    add("full_orientation_error_rad", number(result_.orientation_error));
    add("solve_time_ms", std::to_string(result_.solve_time_ms));
    add("iterations", std::to_string(result_.iterations));
    add("attempts", std::to_string(result_.attempts));
    add("target_frame", frame());
    add("target_z", number(target_pose_.position.z));
    // Lets clients match an answer to the goal they sent.
    add("target_xyz", number(target_pose_.position.x) + " " +
                          number(target_pose_.position.y) + " " +
                          number(target_pose_.position.z));
    add("transformed_z", number(target_pose_.position.z));
    add("ik_goal_lift",
        result_.success ? number(result_.solution.lift) : "nan");
    if (last_seed_) {
      std::ostringstream seed;
      seed << std::setprecision(17) << coordinates(*last_seed_).transpose();
      add("seed_joint_state", seed.str());
      add("seed_topic", get_parameter("joint_state_topic").as_string());
      add("seed_stamp_nanoseconds",
          std::to_string(rclcpp::Time(last_seed_->timestamp).nanoseconds()));
    }
    add("task", grasp_mode_
                    ? "grasp: XYZ + approach (local X) + finger axis (local Y)"
                    : "XYZ + TCP local Z unit direction; spin about Z unconstrained");
    add("approach_error_rad", number(result_.approach_error));
    // Forward base translation that makes a failed grasp goal reachable.
    add("base_dx", number(base_dx_));
    // Header stamp of the external goal this answer belongs to (0: marker).
    add("goal_stamp_ns", std::to_string(goal_stamp_ns_));
    if (result_.success) {
      const auto& p = solved_pose_.position;
      const auto& o = solved_pose_.orientation;
      add("solved_pose", number(p.x) + " " + number(p.y) + " " + number(p.z) +
                             " " + number(o.x) + " " + number(o.y) + " " +
                             number(o.z) + " " + number(o.w));
    }
    if (last_solution_) {
      std::ostringstream q;
      q << std::setprecision(17) << coordinates(*last_solution_).transpose();
      add("last_valid_joint_solution", q.str());
    }
    status.status.push_back(entry);
    status_->publish(status);
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = frame();
    target.header.stamp = now();
    target.pose = target_pose_;
    target_->publish(target);
    if (last_solution_) {
      auto solved = target;
      solved.pose = solved_pose_;
      solved_->publish(solved);
    }
    visualization_msgs::msg::MarkerArray list;
    // Target and solved TCP triads expose the free orientation component.
    for (int kind = 0; kind < 2; ++kind) {
      if (kind == 1 && !last_solution_) continue;
      auto pose = kind == 0 ? target_pose_ : solved_pose_;
      Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z);
      if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) continue;
      for (int axis = 0; axis < 3; ++axis) {
        auto m =
            marker(kind * 3 + axis, Marker::ARROW, geometry_msgs::msg::Pose());
        m.pose.orientation.w = 1;
        m.scale.x = .008;
        m.scale.y = .014;
        m.scale.z = .02;
        m.color.r = axis == 0 ? 1 : 0;
        m.color.g = axis == 1 ? 1 : 0;
        m.color.b = axis == 2 ? 1 : 0;
        m.color.a = kind == 0 ? .45 : 1.;
        geometry_msgs::msg::Point start = pose.position, end = start;
        Eigen::Vector3d v =
            rotation.normalized().toRotationMatrix().col(axis) * .12;
        end.x += v.x();
        end.y += v.y();
        end.z += v.z();
        m.points = {start, end};
        list.markers.push_back(m);
      }
    }
    geometry_msgs::msg::Pose label;
    label.orientation.w = 1;
    label.position.x = .35;
    label.position.z = 1.5;
    auto text = marker(20, Marker::TEXT_VIEW_FACING, label);
    text.scale.z = .035;
    text.color.r = result_.success ? .3 : 1;
    text.color.g = result_.success ? 1 : .3;
    text.color.b = .8;
    text.text = statusText();
    list.markers.push_back(text);
    markers_->publish(list);
  }
  JointBounds hardware_;
  std::unique_ptr<InverseKinematics> ik_;
  urdf::Model display_model_;
  std::string xml_, meshes_;
  std::map<std::string, double> raw_;
  bool have_target_ = false, dirty_ = false, bad_frame_ = false,
       grasp_mode_ = false;
  double base_dx_ = NAN, grasp_approach_tolerance_ = .12, max_base_offset_ = .6;
  int64_t goal_stamp_ns_ = 0;
  geometry_msgs::msg::Pose target_pose_, solved_pose_;
  std::optional<RobotState> last_solution_, last_seed_;
  IkResult result_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_, static_tf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_,
      solved_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_,
      grasp_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_, validate_client_, execute_client_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IkPreview>());
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
