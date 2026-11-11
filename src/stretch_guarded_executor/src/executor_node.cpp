#include <action_msgs/msg/goal_status_array.hpp>
#include <algorithm>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "stretch_collision_planning/octomap_scene_store.hpp"
#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/rrt_connect.hpp"
#include "stretch_custom_kinematics/forward_kinematics.hpp"
#include "stretch_guarded_executor/controller_state.hpp"
#include "stretch_guarded_executor/diagnostic_policy.hpp"
#include "stretch_guarded_executor/action/execute_lift_target.hpp"
#include "stretch_guarded_executor/guard.hpp"
#include "stretch_guarded_executor/lift_sequence.hpp"
#include "stretch_guarded_executor/planned_trajectory.hpp"
#include "stretch_robot_interface/interface_node.hpp"
using namespace stretch_guarded_executor;
namespace collision = stretch_collision_planning;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
using TrajectoryHandle = rclcpp_action::ClientGoalHandle<TrajectoryAction>;
using LiftAction = stretch_guarded_executor::action::ExecuteLiftTarget;
using LiftHandle = rclcpp_action::ServerGoalHandle<LiftAction>;

struct PlanningTransaction {
  uint64_t request_id = 0;
  Vector q_start = Vector::Zero();
  Vector q_goal = Vector::Zero();
  geometry_msgs::msg::Point desired_tcp;
  std::string target_frame;
  double target_z = std::numeric_limits<double>::quiet_NaN();
  double transformed_z = std::numeric_limits<double>::quiet_NaN();
  collision::CollisionEnvironmentBackend backend =
      collision::CollisionEnvironmentBackend::Primitive;
  uint64_t scene_id = 0;
  collision::OctoMapEnvironment::TimePoint scene_received_at{};
  collision::PlannerOptions options;
};

class Executor : public stretch_robot_interface::InterfaceNode {
 public:
  Executor() : InterfaceNode("executor") {
    if (std::string(get_namespace()) != "/guarded_executor")
      throw std::invalid_argument("Required namespace: /guarded_executor");
    dry_ = immutable<bool>("dry_run", true);
    sim_bypass_hardware_checks_ =
        immutable<bool>("sim_bypass_hardware_checks", false);
    transport_test_mode_ = immutable<bool>("transport_test_mode", false);
    config_.execution_enabled = immutable<bool>("execution_enabled", false);
    config_.lift_only = immutable<bool>("lift_only", true);
    diagnostic_policy_.base_motion_enabled =
        immutable<bool>("base_motion_enabled", false);

    diagnostic_policy_.error_to_warn_allowlist =
        immutable<std::vector<std::string>>(
            "diagnostic_error_to_warn_allowlist", {});

    diagnostic_policy_.pimu_temperature_error_can_warn =
        immutable<bool>(
            "pimu_temperature_error_can_warn",
            false);

    diagnostic_policy_.pimu_temperature_warn_max_c =
        immutable<double>(
            "pimu_temperature_warn_max_c",
            50.0);

    diagnostic_policy_.wacc_error_can_warn =
        immutable<bool>("wacc_error_can_warn", false);
    diagnostic_policy_.wacc_ax_min = immutable<double>("wacc_ax_min", 6.0);
    diagnostic_policy_.wacc_ax_max = immutable<double>("wacc_ax_max", 13.5);

    diagnostic_policy_.cliff_range_min =
        immutable<double>("cliff_range_min", -100.0);

    diagnostic_policy_.cliff_range_max =
        immutable<double>("cliff_range_max", 200.0);

    diagnostic_policy_.validate();
    // This is a capability restriction, not permission to add base commands.
    // The classifier also refuses the cliff exception if a future profile has
    // base motion enabled; this executable refuses such a profile altogether.
    if (diagnostic_policy_.base_motion_enabled)
      throw std::invalid_argument(
          "Base motion is prohibited by the lift-only executor");
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    const char* localhost = std::getenv("ROS_LOCALHOST_ONLY");
    if (sim_bypass_hardware_checks_ &&
        (!domain || std::string(domain) != "20" || !localhost ||
         std::string(localhost) != "1"))
      throw std::invalid_argument("Simulation requires domain 20 and localhost isolation");
    if (transport_test_mode_ &&
        (dry_ || !domain || std::string(domain) != "20" || !localhost ||
         std::string(localhost) != "1"))
      throw std::invalid_argument(
          "Transport test mode requires dry_run=false, ROS_DOMAIN_ID=20 and "
          "ROS_LOCALHOST_ONLY=1");
    if (!dry_ && !transport_test_mode_ && !sim_bypass_hardware_checks_ &&
        (!domain || std::string(domain) != "42" ||
         get_parameter("use_sim_time").as_bool() ||
         get_parameter("joint_state_topic").as_string() !=
             "/stretch/joint_states"))
      throw std::invalid_argument(
          "Physical mode requires domain 42, wall time and canonical measured "
          "state");
    if (!dry_ && transport_test_mode_ &&
        (get_parameter("use_sim_time").as_bool() ||
         get_parameter("joint_state_topic").as_string() !=
             "/stretch/joint_states"))
      throw std::invalid_argument(
          "Transport test mode requires wall time and canonical state");
    duration_ = immutable<double>("trajectory_duration", 5.0);
    lift_delta_ = immutable<double>("lift_test_delta", .01);
    config_.max_duration = immutable<double>("max_duration", config_.max_duration);
    config_.min_duration = immutable<double>("min_duration", config_.min_duration);
    self_ignore_margin_ =
        immutable<double>("collision_planning.self_ignore_margin", 0.08);
    // A full 1.1 m lift at 0.1 m/s takes about 21 s.
    if (!sim_bypass_hardware_checks_ && config_.max_duration > 30.0)
      throw std::invalid_argument(
          "max_duration above 30 s is restricted to isolated simulation");
    collision_planning_enabled_ =
        immutable<bool>("collision_planning.enabled", false);
    collision_backend_ = collision::parseCollisionEnvironmentBackend(
        immutable<std::string>("collision_environment", "primitive"));
    octomap_topic_ = immutable<std::string>(
        "collision_planning.octomap_topic", "/perception/octomap_binary");
    collision::OctoMapEnvironmentOptions octomap_options;
    octomap_options.freshness_timeout = immutable<double>(
        "collision_planning.scene_freshness_timeout", 1.0);
    octomap_options.default_map_frame =
        immutable<std::string>("collision_planning.map_frame", "base_link");
    octomap_options.unknown_policy =
        immutable<bool>("collision_planning.unknown_is_collision", true)
            ? collision::UnknownSpacePolicy::UnknownIsCollision
            : collision::UnknownSpacePolicy::UnknownIsFree;
    octomap_scene_store_ = std::make_unique<collision::OctoMapSceneStore>(
        octomap_options, octomap_options.default_map_frame);
    collision_options_.planning_timeout =
        immutable<double>("collision_planning.timeout", 2.0);
    collision_options_.max_iterations =
        immutable<int>("collision_planning.max_iterations", 2000);
    collision_options_.max_extension_distance =
        immutable<double>("collision_planning.step_size", 0.12);
    collision_options_.edge_resolution =
        immutable<double>("collision_planning.edge_resolution", 0.01);
    collision_options_.goal_bias =
        immutable<double>("collision_planning.goal_bias", 0.05);
    const int collision_seed =
        immutable<int>("collision_planning.random_seed", 7);
    if (collision_seed < 0)
      throw std::invalid_argument("collision_planning.random_seed must be >= 0");
    collision_options_.random_seed = static_cast<unsigned>(collision_seed);
    collision_options_.simplify =
        immutable<bool>("collision_planning.shortcut_enabled", true);
    collision_options_.shortcut_attempts =
        immutable<int>("collision_planning.shortcut_attempts", 120);
    collision_marker_enabled_ =
        immutable<bool>("collision_planning.visualize_path", true);
    for (auto entry :
         {std::pair<const char*, Vector*>{"max_delta", &config_.max_delta},
          {"tracking_error", &config_.tracking_error},
          {"goal_tolerance", &config_.goal_tolerance},
          {"start_drift", &config_.start_drift},
          {"max_velocity", &config_.max_velocity},
          {"max_acceleration", &config_.max_acceleration}}) {
      auto v = immutable<std::vector<double>>(
          entry.first,
          std::vector<double>(entry.second->data(), entry.second->data() + 5));
      if (v.size() != 5)
        throw std::invalid_argument("Guard arrays require five entries");
      for (int i = 0; i < 5; ++i) (*entry.second)[i] = v[i];
    }
    config_.validate();
    if (!std::isfinite(lift_delta_) || std::abs(lift_delta_) > .01 ||
        std::abs(lift_delta_) < config_.minimum_motion)
      throw std::invalid_argument(
          "Lift test delta must be nonzero and <= 0.01 m");
    auto share = ament_index_cpp::get_package_share_directory(
        "stretch_custom_kinematics");
    const auto validated = Bounds::load(share + "/config/hardware_limits.csv");
    hardware_ = Bounds::load(immutable<std::string>(
        "hardware_limits_file", share + "/config/hardware_limits.csv"));
    // An alternative limits file may tighten, never widen the validated bounds.
    hardware_.lower = hardware_.lower.cwiseMax(validated.lower);
    hardware_.upper = hardware_.upper.cwiseMin(validated.upper);
    report_path_ = immutable<std::string>(
        "dry_run_report_path", "/tmp/stretch_guarded_executor_report.json");
    action_name_ = immutable<std::string>(
        "trajectory_action", "/stretch_controller/follow_joint_trajectory");
    const bool supported_sim_action =
        sim_bypass_hardware_checks_ &&
        action_name_ == "/stretch_arm_controller/follow_joint_trajectory";
    if (action_name_ != "/stretch_controller/follow_joint_trajectory" &&
        !supported_sim_action)
      throw std::invalid_argument(
          "Unsupported Stretch trajectory action");
    if (!dry_ && !sim_bypass_hardware_checks_) {
      for (auto name :
           {"/stretch_controller/follow_joint_trajectory",
            "/stretch_controller/follow_joint_trajectory/_action/status"})
        if (get_node_topics_interface()->resolve_topic_name(name) != name)
          throw std::invalid_argument("Physical action remapping prohibited");
      for (auto name :
           {"/stop_the_robot", "/runstop", "/stretch_driver/get_parameters"})
        if (get_node_services_interface()->resolve_service_name(name) != name)
          throw std::invalid_argument("Physical service remapping prohibited");
      for (auto name :
           {"/stretch/joint_states", "/is_homed", "/is_runstopped", "/mode",
            "/is_streaming_position", "/diagnostics", "/robot_description"})
        if (get_node_topics_interface()->resolve_topic_name(name) != name)
          throw std::invalid_argument("Physical input remapping prohibited");
    }
    status_ = create_publisher<std_msgs::msg::String>(
        "status", rclcpp::QoS(1).reliable().transient_local());
    lift_status_ = create_publisher<std_msgs::msg::String>(
        "lift_execution_status",
        rclcpp::QoS(1).reliable().transient_local());
    planned_path_markers_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "planned_path", rclcpp::QoS(1).reliable().transient_local());
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        octomap_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](octomap_msgs::msg::Octomap::ConstSharedPtr message) {
          if (!octomap_scene_store_) return;
          // A map kept in odom is queried through the latest odometry pose.
          Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
          if (message->header.frame_id != "base_link") {
            if (steady() - odom_time_ > .5 ||
                odom_frame_ != message->header.frame_id) {
              RCLCPP_ERROR_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Rejected OctoMap in %s: no fresh odometry in that frame",
                  message->header.frame_id.c_str());
              return;
            }
            map_from_base = odom_base_;
          }
          if (!octomap_scene_store_->update(
                  *message, collision::OctoMapEnvironment::Clock::now(),
                  map_from_base)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "Rejected OctoMap: %s",
                octomap_scene_store_->lastError().c_str());
            return;
          }
          monitorDynamicObstacles();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        immutable<std::string>("odom_topic", "/odom"), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& p = m->pose.pose.position;
          const auto& o = m->pose.pose.orientation;
          const auto& v = m->twist.twist;
          Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
          if (!std::isfinite(p.x + p.y + p.z) || q.norm() < 1e-6) return;
          odom_base_ = Eigen::Isometry3d::Identity();
          odom_base_.linear() = q.normalized().toRotationMatrix();
          odom_base_.translation() = Eigen::Vector3d(p.x, p.y, p.z);
          odom_frame_ = m->header.frame_id;
          odom_still_ = std::hypot(v.linear.x, v.linear.y) < .005 &&
                        std::abs(v.angular.z) < .01;
          odom_time_ = steady();
        });
    if (std::string(status_->get_topic_name()).rfind("/guarded_executor/", 0) !=
        0)
      throw std::invalid_argument("Status must remain namespaced");
    auto sensor = rclcpp::SensorDataQoS();
    mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/mode", sensor, [this](std_msgs::msg::String::ConstSharedPtr m) {
          mode_ = m->data;
          mode_time_ = steady();
        });
    stream_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/is_streaming_position", sensor,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          streaming_ = m->data;
          stream_time_ = steady();
        });
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", sensor,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          // Several nodes publish here (driver, Nav2, perception), each its
          // complete status set per array. Keep the latest array per source,
          // keyed by its first status's "<node>:" prefix: one source's OK must
          // never overwrite another's ERROR. Fail-closed on any ERROR stays.
          if (m->status.empty()) return;
          const auto& first = m->status.front().name;
          const std::string source = first.substr(0, first.find(':'));
          std::string blocking, warnings;
          for (const auto& s : m->status) {
            const auto decision = diagnostic_policy_.classify(s);
            if (decision.blocking()) {
              blocking += "BLOCKING: " + s.name + ": " + s.message + "; ";
              RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "Blocking diagnostic: name='%s' level=%u message='%s'",
                                    s.name.c_str(), static_cast<unsigned>(s.level),
                                    s.message.c_str());
            }
            if (decision.downgraded) {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Diagnostic downgraded: name='%s' original_level=ERROR(%u) "
                  "effective_level=WARN message='%s' policy_reason='%s'",
                  s.name.c_str(),
                  static_cast<unsigned>(decision.original_level),
                  s.message.c_str(), decision.reason.c_str());
              // Keep the driver's raw message in the log above. The operator
              // status should describe the effective, calibrated policy so it
              // does not advertise the driver's legacy limits as active.
              warnings += "WARN: " + s.name + ": " + decision.reason + "; ";
            }
          }
          const double now = steady();
          diag_sources_[source] = {now, blocking, warnings};
          diag_ok_ = true;
          diag_reason_.clear();
          warnings.clear();
          for (auto it = diag_sources_.begin(); it != diag_sources_.end();) {
            // A source silent for 2.5 s no longer speaks for the robot; the
            // driver's own silence is caught by the freshness check below.
            if (now - it->second.time > 2.5) { it = diag_sources_.erase(it); continue; }
            if (!it->second.blocking.empty()) diag_ok_ = false;
            diag_reason_ += it->second.blocking;
            warnings += it->second.warnings;
            ++it;
          }
          // Blocking causes first: status consumers truncate long reasons.
          diag_reason_ += warnings;
          // Fresh only while the driver itself reports.
          if (source == "stretch_driver") diag_time_ = now;
        });
    action_status_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        action_name_ + "/_action/status",
        rclcpp::QoS(10).reliable().transient_local(),
        [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) {
          controller_state_.observe(*m);
        });
    preview_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/ik_preview/joint_states", sensor,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          try {
            if (!model_) return;
            auto s = model_->decode(*m);
            preview_q_ = stretch_custom_kinematics::coordinates(s);
            preview_time_ = steady();
            preview_stamp_ = rclcpp::Time(m->header.stamp).seconds();
          } catch (const std::exception&) {
            preview_time_ = -1;
          }
          updateCandidate();
        });
    preview_status_ =
        create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/ik_preview/status", rclcpp::QoS(1).reliable().transient_local(),
            [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
              preview_ok_ = false;
              preview_status_time_ = steady();
              if (m->status.empty()) return;
              const auto& s = m->status.front();
              if (s.level != s.OK || s.message != "success") return;
              std::map<std::string, std::string> values;
              for (auto v : s.values) values[v.key] = v.value;
              std::istringstream q(values["last_valid_joint_solution"]),
                  seed(values["seed_joint_state"]);
              preview_ok_ = values["success"] == "true" &&
                            values["seed_topic"] == get_parameter("joint_state_topic").as_string();
              for (int i = 0; i < 5; ++i)
                if (!(q >> status_q_[i]) || !(seed >> preview_seed_[i]))
                  preview_ok_ = false;
              std::string extra;
              if (q >> extra || seed >> extra) preview_ok_ = false;
              try {
                if (!values.count("target_frame") ||
                    !values.count("target_z") ||
                    !values.count("transformed_z")) {
                  preview_ok_ = false;
                } else {
                  preview_target_frame_ = values.at("target_frame");
                  preview_target_z_ = std::stod(values.at("target_z"));
                  preview_transformed_z_ =
                      std::stod(values.at("transformed_z"));
                  if (!std::isfinite(preview_target_z_) ||
                      !std::isfinite(preview_transformed_z_))
                    preview_ok_ = false;
                }
              } catch (const std::exception&) {
                preview_ok_ = false;
              }
              updateCandidate();
            });
    // Read-only parameter client is allowed in dry run. No action or stop
    // clients exist there.
    parameters_ = create_client<rcl_interfaces::srv::GetParameters>(
        "/stretch_driver/get_parameters");
    if (!dry_ && config_.execution_enabled) {
      action_ = rclcpp_action::create_client<TrajectoryAction>(this, action_name_);
      stop_client_ = create_client<Trigger>("/stop_the_robot");
      runstop_client_ = create_client<SetBool>("/runstop");
      trajectory_mode_client_ = create_client<Trigger>("/switch_to_trajectory_mode");
    }
    arm_service_ = create_service<SetBool>(
        "arm", [this](SetBool::Request::SharedPtr req,
                      SetBool::Response::SharedPtr res) {
          if (!req->data) {
            stopAndDisarm(false);
            res->success = true;
            res->message = "disarmed";
            return;
          }
          if (!guard_ || pending_ || stop_unconfirmed_ || lift_goal_ || sequence_active_) {
            res->success = false;
            res->message = "model unavailable or previous stop/goal unresolved";
            return;
          }
          auto c = guard_->arm();
          res->success = c.ok;
          res->message = c.reason;
          publish();
        });
    services_.push_back(create_service<Trigger>(
        "disarm",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          stopAndDisarm(false);
          r->success = true;
          r->message = "disarmed (active motion aborted/cancel requested)";
        }));
    services_.push_back(create_service<Trigger>(
        "stop",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          stopAndDisarm(true);
          r->success = true;
          r->message =
              dry_ ? "dry-run STOP: no hardware clients exist"
                   : (!action_
                          ? "execution disabled: no hardware STOP client exists"
                          : "STOP/cancel requested; inspect status for "
                            "confirmation");
        }));
    services_.push_back(create_service<Trigger>(
        "stage_lift_test",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          auto in = input();
          if (!guard_ || lift_goal_ || !guard_->armed() ||
              guard_->state() == State::EXECUTING || !in.fresh) {
            r->message = "must be armed with fresh measured state";
            return;
          }
          manual_ = true;
          candidate_ = {};
          candidate_.q = in.q;
          candidate_.seed = in.q;
          candidate_.q[0] += lift_delta_;
          candidate_.relative_lift = lift_delta_;
          candidate_.received = steady();
          candidate_.valid = true;
          candidate_.revision = ++revision_;
          guard_->invalidate("manual lift test staged");
          r->success = true;
          r->message = "staged relative lift " + std::to_string(lift_delta_) +
                       " m only; no goal sent";
          publish();
        }));
    services_.push_back(create_service<Trigger>(
        "use_ik_preview",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          if (!guard_ || lift_goal_ || guard_->state() == State::EXECUTING) {
            r->message = "unavailable while executing";
            return;
          }
          manual_ = false;
          candidate_.valid = false;
          guard_->invalidate("preview source selected");
          updateCandidate();
          r->success = candidate_.valid;
          r->message = "selected IK preview; validation still required";
        }));
    services_.push_back(create_service<Trigger>(
        "validate_preview",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          if (!guard_) {
            r->message = "model unavailable";
            return;
          }
          refreshCandidate();
          auto in = input();
          auto check = guard_->validate(
              in, candidate_, segmentDuration(candidate_.q - in.q), steady());
          if (dry_) writeDryReport(in, check);
          r->success = check.ok;
          r->message = check.reason +
                       (dry_ ? "; dry-run artifact: " + report_path_ : "");
          publish();
        }));
    services_.push_back(create_service<Trigger>(
        "move_to_preview",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          if (!guard_ || pending_ || stop_unconfirmed_ || lift_goal_ || sequence_active_ ||
              guard_->state() == State::EXECUTING) {
            r->message = "executor unavailable or motion already active";
            return;
          }
          if (collision_planning_enabled_) {
            moveToPreviewWithPlanning(*r);
            return;
          }
          manual_ = false;
          updateCandidate();
          if (!candidate_.valid || config_.lift_only) {
            r->message = "a valid 5-DoF IK target is required";
            return;
          }
          sequence_target_ = candidate_.q;
          if ((sequence_target_.array() < active_bounds_.lower.array()).any() ||
              (sequence_target_.array() > active_bounds_.upper.array()).any()) {
            r->message = "target outside hardware/model limits";
            return;
          }
          guard_->disarm();
          auto check = guard_->arm();
          auto measured = input();
          const bool switch_mode = !dry_ && measured.mode_fresh && !measured.trajectory_mode;
          if (switch_mode) measured.trajectory_mode = true;
          if (check.ok) check = guard_->preflight(measured);
          if (check.ok && (!measured.controller_idle || measured.active_goal_present))
            check = {false, "controller busy"};
          if (!check.ok) {
            guard_->abort(check.reason);
            r->message = check.reason;
            publish();
            return;
          }
          sequence_active_ = true;
          manual_ = true;
          if (switch_mode) {
            if (!trajectory_mode_client_ || !trajectory_mode_client_->service_is_ready()) {
              sequence_active_ = false;
              manual_ = false;
              guard_->abort("trajectory mode service unavailable");
              r->message = guard_->reason();
              publish();
              return;
            }
            preparing_mode_ = true;
            mode_request_sent_ = steady();
            trajectory_mode_client_->async_send_request(std::make_shared<Trigger::Request>(),
                [this](rclcpp::Client<Trigger>::SharedFuture future) {
                  if (!preparing_mode_ || !sequence_active_) return;
                  if (!future.get()->success) {
                    preparing_mode_ = false;
                    sequence_active_ = false;
                    manual_ = false;
                    guard_->abort("driver rejected trajectory mode");
                    publish();
                  }
                });
            r->success = true;
            r->message = "target accepted; preparing trajectory mode";
            publish();
            return;
          }
          startNextSegment(*r);
        }));
    services_.push_back(create_service<Trigger>(
        "execute_preview",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          execute(*r);
        }));
    lift_action_server_ = rclcpp_action::create_server<LiftAction>(
        this, "execute_lift_target",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const LiftAction::Goal> goal) {
          return liftGoal(goal);
        },
        [this](const std::shared_ptr<LiftHandle> goal) {
          return liftCancel(goal);
        },
        [this](const std::shared_ptr<LiftHandle> goal) {
          liftAccepted(goal);
        });
    parameter_lock_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>&) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = false;
          r.reason =
              "Executor configuration is immutable; restart DISARMED to change";
          return r;
        });
    if (get_parameter("state_timeout").as_double() > .5)
      throw std::invalid_argument("State timeout must be <= 0.5 s");
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
                "Startup DISARMED; dry_run=%d execution_enabled=%d "
                "hardware_action_client=%d",
                dry_, config_.execution_enabled, bool(action_));
  }
  void shutdownStop() {
    if (guard_ && (guard_->state() == State::EXECUTING || pending_))
      stopAndDisarm(true);
  }

 private:
  template <class T>
  T immutable(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return declare_parameter<T>(name, value, d);
  }
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool actionServerAvailable() const {
    // Physical execution uses the actual action client's readiness check. In
    // dry run no ActionClient may exist: inspect all five advertised action
    // endpoints from the inspected driver instead, without sending requests.
    try {
      if (action_) return action_->action_server_is_ready();
      const auto services =
          get_service_names_and_types_by_node("stretch_driver", "/");
      for (const auto& entry :
           {std::pair<std::string, std::string>{
                "send_goal",
                "control_msgs/action/FollowJointTrajectory_SendGoal"},
            {"get_result",
             "control_msgs/action/FollowJointTrajectory_GetResult"},
            {"cancel_goal", "action_msgs/srv/CancelGoal"}}) {
        const auto found =
            services.find(action_name_ + "/_action/" + entry.first);
        if (found == services.end() ||
            std::find(found->second.begin(), found->second.end(),
                      entry.second) == found->second.end())
          return false;
      }
      for (const auto& entry :
           {std::pair<std::string, std::string>{
                "status", "action_msgs/msg/GoalStatusArray"},
            {"feedback",
             "control_msgs/action/FollowJointTrajectory_FeedbackMessage"}}) {
        bool found = false;
        for (const auto& endpoint : get_publishers_info_by_topic(
                 action_name_ + "/_action/" + entry.first))
          if (endpoint.node_name() == "stretch_driver" &&
              endpoint.node_namespace() == "/" &&
              endpoint.topic_type() == entry.second)
            found = true;
        if (!found) return false;
      }
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }
  Input input() const {
    Input in;
    auto q = state();
    if (q) {
      in.q = stretch_custom_kinematics::coordinates(*q);
      in.fresh = q->valid;
      in.required_joints = q->valid;
      in.homed = q->homed;
      in.runstopped = q->runstopped;
    }
    double t = steady();
    in.mode_fresh = t - mode_time_ < .5;
    in.trajectory_mode = mode_ == "trajectory";
    in.streaming_known = t - stream_time_ < .5;
    in.streaming = streaming_;
    const auto controller =
        controller_state_.preflight(actionServerAvailable(), pending_);
    in.action_server_available = controller.action_server_available;
    in.status_message_received = controller.status_message_received;
    in.active_goal_present = controller.active_goal_present;
    in.controller_idle = controller.controller_idle;
    in.diagnostics_fresh = t - diag_time_ < 2.5;
    in.diagnostics_ok = diag_ok_;
    in.startpoint_guard_verified = start_guard_ && t - param_time_ < 5;
    // Without odometry (kinematic fixtures) the base cannot move.
    in.base_still = odom_time_ < 0 || (t - odom_time_ < .5 && odom_still_);
    if (get_parameter("sim_bypass_hardware_checks").as_bool()) {
      // The simulated controller owns trajectory timing and state watchdogs.
      // Require a decoded state, but ignore brief /clock publication skew.
      in.fresh = q.has_value();
      in.required_joints = q.has_value();
      in.homed = true;
      in.runstopped = false;
      in.mode_fresh = true;
      in.trajectory_mode = true;
      in.streaming_known = true;
      in.streaming = false;
      if (dry_) {
        in.action_server_available = true;
        in.controller_idle = !in.active_goal_present;
      }
      in.diagnostics_fresh = true;
      in.diagnostics_ok = true;
      in.startpoint_guard_verified = true;
    }
    return in;
  }
  void updateCandidate() {
    if (manual_ || !guard_ || guard_->state() == State::EXECUTING) return;
    double t = steady();
    bool ok = preview_ok_ && t - preview_time_ < .5 &&
              t - preview_status_time_ < .5 &&
              std::abs(now().seconds() - preview_stamp_) < .5 &&
              preview_q_.allFinite() && status_q_.allFinite() &&
              preview_seed_.allFinite() &&
              (preview_q_ - status_q_).cwiseAbs().maxCoeff() < 1e-9;
    if (!ok) {
      candidate_.valid = false;
      guard_->invalidate("preview invalid/failure/stale");
      return;
    }
    if (!candidate_.valid ||
        (candidate_.q - preview_q_).cwiseAbs().maxCoeff() > 1e-9 ||
        (candidate_.seed - preview_seed_).cwiseAbs().maxCoeff() > 1e-9) {
      candidate_.revision = ++revision_;
      guard_->invalidate("preview changed; revalidation required");
    }
    candidate_.q = preview_q_;
    candidate_.seed = preview_seed_;
    candidate_.received = t;
    candidate_.valid = true;
    candidate_.relative_lift.reset();
  }
  void refreshCandidate() {
    if (!manual_) updateCandidate();
  }
  void readParameters() {
    double t = steady();
    if (t - last_param_query_ < 1) return;
    last_param_query_ = t;
    if (param_pending_) {
      if (t - param_sent_ > 2) {
        parameters_->remove_pending_request(param_request_id_);
        param_pending_ = false;
        start_guard_ = false;
      } else
        return;
    }
    if (!parameters_->service_is_ready()) {
      start_guard_ = false;
      return;
    }
    auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    req->names = {"fail_if_motor_initial_point_is_not_trajectory_first_point"};
    param_pending_ = true;
    param_sent_ = t;
    auto f = parameters_->async_send_request(
        req,
        [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture
                   f) {
          param_pending_ = false;
          auto r = f.get();
          start_guard_ =
              r->values.size() == 1 &&
              r->values[0].type ==
                  rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
              r->values[0].bool_value;
          param_time_ = steady();
        });
    param_request_id_ = f.request_id;
  }
  void tick() {
    readParameters();
    if (model_ && model_.get() != model_identity_) {
      if (guard_ && guard_->state() == State::EXECUTING) stopAndDisarm(true);
      if (pending_) return;
      model_identity_ = model_.get();
      active_bounds_ = hardware_.intersectModel(*model_);
      bounds_ready_ = true;
      guard_ = std::make_unique<Guard>(config_, active_bounds_);
      configureCollisionPlanning();
      candidate_.valid = false;
      manual_ = false;
    }
    if (!guard_) {
      publish();
      return;
    }
    if (guard_->state() == State::EXECUTING) {
      const double now = steady();
      const auto measured = input();
      // Gazebo controllers advance trajectories in simulation time. The
      // controller still enforces path tolerances; wall-time tracking here is
      // retained for the physical robot only.
      auto c = tracking_synchronized_ && !sim_bypass_hardware_checks_
                   ? guard_->monitor(measured, now)
                   : guard_->monitorPrerequisites(measured);
      if (!c.ok) {
        sendStop();
        publish();
        return;
      }
      if (tracking_synchronized_ && guard_->plan()) {
        const auto error =
            (measured.q - Guard::reference(*guard_->plan(),
                                            now - guard_->plan()->started_at))
                .cwiseAbs();
        max_tracking_error_ = max_tracking_error_.cwiseMax(error);
        if (lift_goal_)
          lift_max_tracking_error_ =
              std::max(lift_max_tracking_error_, error[0]);
      }
      if (controller_success_pending_) {
        if (now - controller_result_time_ >= .1)
          completeSuccessfulTrajectory();
      } else if (!accepted_ && now - sent_time_ > 1.) {
        guard_->abort("goal acknowledgement timeout");
        sendStop();
      } else if (accepted_ && now - feedback_time_ > 1.) {
        guard_->abort("controller feedback timeout");
        sendStop();
      } else if (action_ && !action_->action_server_is_ready()) {
        guard_->abort("controller unavailable");
        sendStop();
      }
    } else if (guard_->state() == State::READY) {
      refreshCandidate();
      if (guard_->plan() &&
          steady() - guard_->plan()->approved_at > config_.approval_timeout)
        guard_->invalidate("approval expired");
    }
    if (stop_unconfirmed_ && runstop_requested_) {
      const auto measured = input();
      if (measured.fresh && measured.runstopped) stop_unconfirmed_ = false;
    }
    if (stop_unconfirmed_ && steady() - stop_sent_ > .5 && !runstop_requested_)
      requestRunstop();
    if (lift_goal_) {
      if (lift_cancel_requested_ && !pending_ && !stop_unconfirmed_) {
        finishLiftAction(false, true, "lift execution canceled by operator");
      } else if (guard_->state() == State::ABORTED && !pending_ &&
                 !stop_unconfirmed_) {
        failLiftAction(guard_->reason());
      } else if (lift_next_pending_ && !pending_ &&
                 guard_->state() != State::EXECUTING) {
        startNextLiftSegment();
      }
    }
    if (steady() - last_publish_ > .2) publish();
    if (preparing_mode_ && replan_after_mode_) {
      const auto measured = input();
      if (measured.mode_fresh && measured.trajectory_mode) {
        if (mode_ready_since_ < 0) mode_ready_since_ = steady();
        if (steady() - mode_ready_since_ > .5) {  // let the arm settle
          preparing_mode_ = replan_after_mode_ = false;
          sequence_active_ = sequence_planned_ = false;
          Trigger::Response response;
          moveToPreviewWithPlanning(response);
          if (!response.success)
            RCLCPP_ERROR(get_logger(), "Planning after mode switch failed: %s",
                         response.message.c_str());
        }
      } else if (steady() - mode_request_sent_ > 3.) {
        preparing_mode_ = replan_after_mode_ = sequence_active_ = false;
        sequence_planned_ = false;
        guard_->abort("timeout waiting for trajectory mode");
        publish();
      }
    } else if (preparing_mode_ && sequence_active_) {
      const auto measured = input();
      if (measured.mode_fresh && measured.trajectory_mode) {
        preparing_mode_ = false;
        Trigger::Response response;
        if (sequence_planned_)
          executePreparedPlannedTrajectory(response);
        else
          startNextSegment(response);
      } else if (steady() - mode_request_sent_ > 3.) {
        preparing_mode_ = false;
        sequence_active_ = false;
        sequence_planned_ = false;
        prepared_trajectory_ = {};
        manual_ = false;
        guard_->abort("timeout waiting for trajectory mode");
        publish();
      }
    }
    if (sequence_active_ && guard_->state() == State::ABORTED) {
      sequence_active_ = false;
      sequence_planned_ = false;
      prepared_trajectory_ = {};
      manual_ = false;
    }
    if (sequence_active_ && !pending_ && !stop_unconfirmed_ &&
        guard_->state() == State::COMPLETE) {
      if (sequence_planned_) {
        sequence_active_ = false;
        sequence_planned_ = false;
        prepared_trajectory_ = {};
        manual_ = false;
        publish();
      } else {
        Trigger::Response response;
        startNextSegment(response);
      }
    }
  }

  void configureCollisionPlanning() {
    collision_planner_.reset();
    collision_checker_.reset();
    robot_collision_model_.reset();
    if (!collision_planning_enabled_) {
      planning_status_ = "collision_planning=disabled";
      clearPlannedMarkers();
      return;
    }
    try {
      robot_collision_model_ =
          std::make_shared<collision::PrimitiveRobotCollisionModel>(
          collision::PrimitiveRobotCollisionModel::makeConservativeStretchArm());
      if (collision_backend_ ==
          collision::CollisionEnvironmentBackend::Primitive) {
        auto scene = std::make_shared<collision::PrimitiveEnvironment>(
            collision::PrimitiveEnvironment::makeDeskBenchmark());
        collision_checker_ =
            std::make_unique<collision::StateValidityChecker>(
                model_, active_bounds_, robot_collision_model_, scene);
        collision_planner_ =
            std::make_unique<collision::RrtConnectPlanner>(*collision_checker_);
        planning_status_ =
            "collision_planning=ready environment=PRIMITIVE "
            "scene=deterministic_primitive_desk";
      } else {
        planning_status_ =
            "collision_planning=waiting environment=OCTOMAP "
            "scene=SCENE_UNAVAILABLE";
      }
    } catch (const std::exception& e) {
      planning_status_ = std::string("SCENE_UNAVAILABLE: ") + e.what();
      collision_planner_.reset();
      collision_checker_.reset();
      clearPlannedMarkers();
    }
  }

  // Minimum-time quintic for one direct segment, never shorter than the
  // configured nominal duration. The 1% margin keeps the guard's exact peak
  // checks from failing on rounding.
  double segmentDuration(const Vector& delta) const {
    double t = duration_;
    for (int i = 0; i < 5; ++i) {
      if (!std::isfinite(delta[i])) return duration_;
      const double d = std::abs(delta[i]);
      t = std::max({t, 1.01 * 1.875 * d / config_.max_velocity[i],
                    1.01 * std::sqrt((10 / std::sqrt(3.)) * d /
                                     config_.max_acceleration[i])});
    }
    return t;
  }
  TimedTrajectoryOptions timingOptions() const {
    TimedTrajectoryOptions options;
    options.bounds = active_bounds_;
    options.max_delta = config_.max_delta;
    options.max_velocity = config_.max_velocity;
    options.max_acceleration = config_.max_acceleration;
    options.min_duration = config_.min_duration;
    options.max_duration = config_.max_duration;
    options.nominal_duration = duration_;
    return options;
  }

  Check validateTimedTrajectoryCollision(
      const TimedTrajectory& trajectory,
      const collision::StateValidityChecker* checker = nullptr) const {
    if (!checker) checker = collision_checker_.get();
    if (!checker)
      return {false, "SCENE_UNAVAILABLE: collision checker unavailable"};
    if (trajectory.states.empty() ||
        trajectory.states.size() != trajectory.times_from_start.size())
      return {false, "TRAJECTORY_COLLISION: invalid timed trajectory"};
    for (size_t i = 0; i < trajectory.states.size(); ++i) {
      const auto state = checker->isStateValid(trajectory.states[i]);
      if (!state.valid) {
        return {false, "TRAJECTORY_COLLISION: point " + std::to_string(i) +
                           " invalid: " + state.reason};
      }
    }
    for (size_t i = 1; i < trajectory.states.size(); ++i) {
      const auto edge = checker->isEdgeValid(
          trajectory.states[i - 1], trajectory.states[i],
          collision_options_.edge_resolution);
      if (!edge.valid) {
        return {false, "TRAJECTORY_COLLISION: edge " +
                           std::to_string(i - 1) + "->" +
                           std::to_string(i) +
                           " invalid: " + edge.first_invalid.reason};
      }
    }
    return {true, "trajectory collision validation passed"};
  }

  // Copy of a map scene where voxels touching the arm where it is now are
  // ignored: they are the arm itself (camera latency, gripper bulkier than
  // its capsule), never an obstacle it could avoid.
  std::shared_ptr<collision::OctoMapEnvironment> withoutSelf(
      const std::shared_ptr<const collision::OctoMapEnvironment>& scene) const {
    auto environment = std::make_shared<collision::OctoMapEnvironment>(*scene);
    const auto measured = input();
    if (!measured.q.allFinite() || !robot_collision_model_) return environment;
    collision::StateValidityChecker current(
        model_, active_bounds_, robot_collision_model_, environment);
    auto body =
        robot_collision_model_->capsules(current.linkTransforms(measured.q));
    for (auto& c : body) c.radius += self_ignore_margin_;
    environment->setIgnoredCapsules(body);
    return environment;
  }

  // Re-checks the rest of an executing planned trajectory against every new
  // map, so an obstacle that appears on the path stops the arm.
  void monitorDynamicObstacles() {
    if (!guard_ || guard_->state() != State::EXECUTING || !guard_->plan() ||
        guard_->plan()->trajectory.empty() || guard_->plan()->started_at <= 0)
      return;
    const auto latest = octomap_scene_store_->snapshot();
    if (!latest.available()) return;
    const auto& plan = *guard_->plan();
    const double elapsed = steady() - plan.started_at;
    TimedTrajectory rest;
    for (size_t i = 0; i < plan.trajectory.size(); ++i)
      if (plan.times_from_start[i] >= elapsed - .2) {
        rest.states.push_back(plan.trajectory[i]);
        rest.times_from_start.push_back(plan.times_from_start[i]);
      }
    if (rest.states.empty()) return;
    Check check{true, ""};
    try {
      collision::StateValidityChecker checker(
          model_, active_bounds_, robot_collision_model_,
          withoutSelf(latest.environment));
      check = validateTimedTrajectoryCollision(rest, &checker);
    } catch (const std::exception& e) {
      // The motion keeps going on its pre-validated path; say the watch is blind.
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Dynamic obstacle check skipped: %s", e.what());
      return;
    }
    if (check.ok) return;
    const std::string reason = "DYNAMIC_OBSTACLE: " + check.reason;
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    stopAndDisarm(true);
    guard_->abort(reason);
    publish();
  }

  stretch_robot_interface::RobotState stateFromVector(const Vector& q) const {
    stretch_robot_interface::RobotState state;
    state.valid = true;
    state.lift = q[0];
    state.arm_extension = q[1];
    state.wrist_yaw = q[2];
    state.wrist_pitch = q[3];
    state.wrist_roll = q[4];
    return state;
  }

  geometry_msgs::msg::Point tcpPoint(const Vector& q) const {
    const auto pose =
        stretch_custom_kinematics::ForwardKinematics(model_).computeTcpPose(
            stateFromVector(q));
    geometry_msgs::msg::Point point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    return point;
  }

  visualization_msgs::msg::Marker lineMarker(
      int id, const std::string& ns, const std::vector<Vector>& path,
      float r, float g, float b, float a, double width) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = model_ ? model_->base() : "base_link";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    if (model_) {
      for (const auto& q : path) marker.points.push_back(tcpPoint(q));
    }
    return marker;
  }

  geometry_msgs::msg::Point markerPoint(const Eigen::Vector3d& value) const {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
  }

  visualization_msgs::msg::Marker diagnosticMarker(
      int id, const std::string& ns, int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = model_ ? model_->base() : "base_link";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void publishCollisionDiagnostics(
      const Vector& start, const Vector& goal,
      const collision::StateValidityResult& validity,
      const std::string& context) {
    if (!collision_marker_enabled_ || !planned_path_markers_ ||
        !collision_checker_)
      return;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const auto add_workspace_point = [&](int id, const std::string& ns,
                                         const Vector& q, float r, float g,
                                         float b) {
      auto marker = diagnosticMarker(
          id, ns, visualization_msgs::msg::Marker::SPHERE);
      marker.pose.position = tcpPoint(q);
      marker.scale.x = marker.scale.y = marker.scale.z = 0.035;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.95f;
      markers.markers.push_back(std::move(marker));
    };
    add_workspace_point(30, "workspace_start", start, 0.1f, 0.9f, 0.2f);
    add_workspace_point(31, "workspace_goal", goal, 1.0f, 0.55f, 0.05f);

    if (validity.contact) {
      const auto& contact = *validity.contact;
      const bool unknown =
          contact.kind == collision::CollisionContactKind::Unknown;
      if (contact.robot_capsule) {
        const auto& capsule = *contact.robot_capsule;
        auto line = diagnosticMarker(
            10, "rejected_collision_capsule",
            visualization_msgs::msg::Marker::LINE_LIST);
        line.points = {markerPoint(capsule.a), markerPoint(capsule.b)};
        line.scale.x = 2.0 * capsule.radius;
        line.color.r = unknown ? 0.7f : 1.0f;
        line.color.g = unknown ? 0.7f : 0.05f;
        line.color.b = unknown ? 0.7f : 0.05f;
        line.color.a = 0.9f;
        markers.markers.push_back(std::move(line));

        auto ends = diagnosticMarker(
            11, "rejected_collision_capsule_ends",
            visualization_msgs::msg::Marker::SPHERE_LIST);
        ends.points = {markerPoint(capsule.a), markerPoint(capsule.b)};
        ends.scale.x = ends.scale.y = ends.scale.z = 2.0 * capsule.radius;
        ends.color.r = unknown ? 0.7f : 1.0f;
        ends.color.g = unknown ? 0.7f : 0.05f;
        ends.color.b = unknown ? 0.7f : 0.05f;
        ends.color.a = 0.9f;
        markers.markers.push_back(std::move(ends));
      }
      if (contact.environment_box) {
        const auto& box = *contact.environment_box;
        auto voxel = diagnosticMarker(
            12, unknown ? "collision_voxel_unknown"
                        : "collision_voxel_occupied",
            visualization_msgs::msg::Marker::CUBE);
        voxel.pose.position = markerPoint(box.center);
        voxel.scale.x = box.size.x();
        voxel.scale.y = box.size.y();
        voxel.scale.z = box.size.z();
        voxel.color.r = unknown ? 0.55f : 1.0f;
        voxel.color.g = unknown ? 0.55f : 0.0f;
        voxel.color.b = unknown ? 0.55f : 0.0f;
        voxel.color.a = 0.95f;
        markers.markers.push_back(std::move(voxel));

        auto label = diagnosticMarker(
            13, "collision_contact_label",
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        label.pose.position = markerPoint(
            box.center + Eigen::Vector3d(0.0, 0.0, 0.06));
        label.scale.z = 0.035;
        label.color.r = label.color.g = label.color.b = 1.0f;
        label.color.a = 1.0f;
        label.text = context + ": " +
                     (unknown ? "UNKNOWN" : "OCCUPIED") + "\n" +
                     contact.robot_geometry;
        markers.markers.push_back(std::move(label));
      }
    }

    // A 2-D diagnostic slice: lift (vertical) x arm extension (horizontal),
    // with wrist joints fixed at the rejected goal values.
    constexpr int samples = 25;
    constexpr double width = 0.40;
    constexpr double height = 0.55;
    const Eigen::Vector3d origin(0.42, 0.38, 0.20);
    const auto& bounds = collision_checker_->bounds();
    auto grid = diagnosticMarker(
        20, "cspace_slice", visualization_msgs::msg::Marker::CUBE_LIST);
    grid.scale.x = width / samples;
    grid.scale.y = 0.008;
    grid.scale.z = height / samples;
    grid.points.reserve(samples * samples);
    grid.colors.reserve(samples * samples);
    for (int lift_index = 0; lift_index < samples; ++lift_index) {
      for (int arm_index = 0; arm_index < samples; ++arm_index) {
        const double lift_u = static_cast<double>(lift_index) / (samples - 1);
        const double arm_u = static_cast<double>(arm_index) / (samples - 1);
        Vector q = goal;
        q[0] = bounds.lower[0] + lift_u * (bounds.upper[0] - bounds.lower[0]);
        q[1] = bounds.lower[1] + arm_u * (bounds.upper[1] - bounds.lower[1]);
        const auto sample = collision_checker_->isStateValid(q);
        geometry_msgs::msg::Point point;
        point.x = origin.x() + arm_u * width;
        point.y = origin.y();
        point.z = origin.z() + lift_u * height;
        grid.points.push_back(point);
        std_msgs::msg::ColorRGBA color;
        color.a = 0.65f;
        if (sample.valid) {
          color.r = 0.1f;
          color.g = 0.8f;
          color.b = 0.2f;
          color.a = 0.35f;
        } else if (sample.contact &&
                   sample.contact->kind ==
                       collision::CollisionContactKind::Unknown) {
          color.r = color.g = color.b = 0.5f;
        } else if (sample.contact &&
                   sample.contact->kind ==
                       collision::CollisionContactKind::Occupied) {
          color.r = 0.95f;
          color.g = 0.05f;
          color.b = 0.05f;
        } else {
          color.r = 1.0f;
          color.g = 0.55f;
          color.b = 0.0f;
        }
        grid.colors.push_back(color);
      }
    }
    markers.markers.push_back(std::move(grid));

    const auto cspace_point = [&](const Vector& q) {
      geometry_msgs::msg::Point point;
      const double arm_u = std::clamp(
          (q[1] - bounds.lower[1]) / (bounds.upper[1] - bounds.lower[1]),
          0.0, 1.0);
      const double lift_u = std::clamp(
          (q[0] - bounds.lower[0]) / (bounds.upper[0] - bounds.lower[0]),
          0.0, 1.0);
      point.x = origin.x() + arm_u * width;
      point.y = origin.y() - 0.012;
      point.z = origin.z() + lift_u * height;
      return point;
    };
    auto projected_edge = diagnosticMarker(
        21, "cspace_projected_edge",
        visualization_msgs::msg::Marker::LINE_STRIP);
    projected_edge.points = {cspace_point(start), cspace_point(goal)};
    projected_edge.scale.x = 0.008;
    projected_edge.color.r = 1.0f;
    projected_edge.color.g = 0.85f;
    projected_edge.color.b = 0.0f;
    projected_edge.color.a = 1.0f;
    markers.markers.push_back(std::move(projected_edge));

    const auto add_cspace_point = [&](int id, const std::string& ns,
                                      const Vector& q, float r, float g,
                                      float b) {
      auto marker = diagnosticMarker(
          id, ns, visualization_msgs::msg::Marker::SPHERE);
      marker.pose.position = cspace_point(q);
      marker.scale.x = marker.scale.y = marker.scale.z = 0.025;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0f;
      markers.markers.push_back(std::move(marker));
    };
    add_cspace_point(22, "cspace_start", start, 0.1f, 1.0f, 0.2f);
    add_cspace_point(23, "cspace_goal", goal, 1.0f, 0.55f, 0.0f);

    auto cspace_label = diagnosticMarker(
        24, "cspace_label",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    cspace_label.pose.position = markerPoint(
        origin + Eigen::Vector3d(width * 0.5, 0.0, height + 0.04));
    cspace_label.scale.z = 0.035;
    cspace_label.color.r = cspace_label.color.g = cspace_label.color.b = 1.0f;
    cspace_label.color.a = 1.0f;
    cspace_label.text =
        "C-space: lift x arm\ngreen=free red=occupied gray=unknown";
    markers.markers.push_back(std::move(cspace_label));

    planned_path_markers_->publish(markers);
  }

  void clearPlannedMarkers() {
    if (!planned_path_markers_) return;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    planned_path_markers_->publish(markers);
  }

  void publishPlannedMarkers(const std::vector<Vector>& raw,
                             const std::vector<Vector>& simplified,
                             const TimedTrajectory& trajectory) {
    if (!collision_marker_enabled_ || !planned_path_markers_) return;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    if (!raw.empty())
      markers.markers.push_back(
          lineMarker(1, "raw_rrt_tcp_path", raw, .6f, .6f, .6f, .45f, .004));
    if (!simplified.empty())
      markers.markers.push_back(lineMarker(2, "simplified_tcp_path",
                                           simplified, .1f, .45f, 1.f, .8f,
                                           .007));
    if (!trajectory.states.empty())
      markers.markers.push_back(lineMarker(3, "executable_tcp_trajectory",
                                           trajectory.states, .1f, 1.f, .25f,
                                           1.f, .01));
    planned_path_markers_->publish(markers);
  }

  void failPlanning(Trigger::Response& response, const std::string& status,
                    const std::string& reason) {
    logLiftTrace("planning_failed", status + ": " + reason);
    prepared_trajectory_ = {};
    planned_path_raw_.clear();
    planned_path_simplified_.clear();
    sequence_active_ = false;
    sequence_planned_ = false;
    preparing_mode_ = replan_after_mode_ = false;
    manual_ = false;
    candidate_.valid = false;
    active_planning_request_id_ = 0;
    planning_status_ = "REQUEST_ID=" +
                       std::to_string(active_transaction_.request_id) +
                       " ENVIRONMENT=" +
                       collision::collisionEnvironmentBackendName(
                           active_transaction_.backend) +
                       " " + status + ": " + reason;
    clearPlannedMarkers();
    if (guard_) guard_->abort(planning_status_);
    response.success = false;
    response.message = planning_status_;
    RCLCPP_WARN(get_logger(), "%s", planning_status_.c_str());
    lift_trace_active_ = false;
    publish();
  }

  void logLiftTrace(const std::string& stage, const std::string& status,
                    double measured = std::numeric_limits<double>::quiet_NaN()) {
    if (!lift_trace_active_) return;
    const double trajectory_lift =
        prepared_trajectory_.states.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : prepared_trajectory_.states.back()[0];
    RCLCPP_INFO(
        get_logger(),
        "[LIFT_TRACE] stage=%s target_z=%.6f target_frame=%s "
        "transformed_z=%.6f ik_lift=%.6f planner_lift=%.6f "
        "trajectory_lift=%.6f commanded_lift=%.6f measured_lift=%.6f "
        "status=%s",
        stage.c_str(), active_transaction_.target_z,
        active_transaction_.target_frame.c_str(),
        active_transaction_.transformed_z, active_transaction_.q_goal[0],
        active_transaction_.q_goal[0], trajectory_lift, commanded_lift_,
        measured, status.c_str());
  }

  double sceneAgeSeconds(
      const collision::OctoMapSceneSnapshot& snapshot) const {
    return std::chrono::duration<double>(
               collision::OctoMapEnvironment::Clock::now() -
               snapshot.received_at)
        .count();
  }

  bool selectPlanningSnapshot(Trigger::Response& response) {
    if (collision_backend_ ==
        collision::CollisionEnvironmentBackend::Primitive) {
      active_scene_id_ = 0;
      active_octomap_snapshot_ = {};
      return collision_checker_ && collision_planner_;
    }
    const auto snapshot = octomap_scene_store_->snapshot();
    if (!snapshot.available()) {
      failPlanning(response, collision::sceneFreshnessName(snapshot.status),
                   snapshot.reason);
      return false;
    }
    try {
      auto checker = std::make_unique<collision::StateValidityChecker>(
          model_, active_bounds_, robot_collision_model_,
          withoutSelf(snapshot.environment));
      auto planner =
          std::make_unique<collision::RrtConnectPlanner>(*checker);
      collision_checker_ = std::move(checker);
      collision_planner_ = std::move(planner);
      active_scene_id_ = snapshot.id;
      active_octomap_snapshot_ = snapshot;
      return true;
    } catch (const std::exception& e) {
      failPlanning(response, "SCENE_UNAVAILABLE", e.what());
      return false;
    }
  }

  void moveToPreviewWithPlanning(Trigger::Response& response) {
    const uint64_t request_id = ++planning_request_id_;
    active_transaction_.request_id = request_id;
    active_transaction_.backend = collision_backend_;
    active_transaction_.options = collision_options_;
    lift_trace_active_ = false;
    commanded_lift_ = std::numeric_limits<double>::quiet_NaN();
    prepared_trajectory_ = {};
    planned_path_raw_.clear();
    planned_path_simplified_.clear();
    manual_ = false;
    updateCandidate();
    if (!candidate_.valid || config_.lift_only) {
      failPlanning(response, "IK_FAILED",
                   "a valid 5-DoF IK target is required");
      return;
    }
    // Switch to trajectory mode first and plan afterwards: the arm settles a
    // few millimeters in the switch, which would invalidate a plan made before.
    const auto before = input();
    if (!dry_ && before.mode_fresh && !before.trajectory_mode &&
        trajectory_mode_client_ && trajectory_mode_client_->service_is_ready()) {
      preparing_mode_ = true;
      replan_after_mode_ = true;
      mode_ready_since_ = -1;
      sequence_active_ = true;
      sequence_planned_ = true;
      mode_request_sent_ = steady();
      trajectory_mode_client_->async_send_request(
          std::make_shared<Trigger::Request>(),
          [this](rclcpp::Client<Trigger>::SharedFuture future) {
            if (!future.get()->success && replan_after_mode_) {
              replan_after_mode_ = preparing_mode_ = sequence_active_ = false;
              guard_->abort("driver rejected trajectory mode");
              publish();
            }
          });
      response.success = true;
      response.message = "switching to trajectory mode; planning follows";
      publish();
      return;
    }
    const Candidate request_candidate = candidate_;
    lift_trace_active_ = true;
    active_transaction_.q_goal = request_candidate.q;
    active_transaction_.target_frame = preview_target_frame_;
    active_transaction_.target_z = preview_target_z_;
    active_transaction_.transformed_z = preview_transformed_z_;
    sequence_target_ = request_candidate.q;
    if ((sequence_target_.array() < active_bounds_.lower.array()).any() ||
        (sequence_target_.array() > active_bounds_.upper.array()).any()) {
      failPlanning(response, "GOAL_IN_COLLISION",
                   "target outside hardware/model limits");
      return;
    }
    if (!selectPlanningSnapshot(response)) {
      if (!response.message.empty()) return;
      failPlanning(response, "SCENE_UNAVAILABLE",
                   "selected collision environment is not ready");
      return;
    }

    guard_->disarm();
    auto check = guard_->arm();
    auto measured = input();
    const bool switch_mode =
        !dry_ && measured.mode_fresh && !measured.trajectory_mode;
    if (switch_mode) measured.trajectory_mode = true;
    if (check.ok) check = guard_->preflight(measured);
    if (check.ok && (!measured.controller_idle || measured.active_goal_present))
      check = {false, "controller busy"};
    if (!check.ok) {
      failPlanning(response, "EXECUTION_REJECTED", check.reason);
      return;
    }

    const Vector start = measured.q;
    const Vector goal = request_candidate.q;
    active_transaction_.request_id = request_id;
    active_transaction_.q_start = start;
    active_transaction_.q_goal = goal;
    active_transaction_.desired_tcp = tcpPoint(goal);
    active_transaction_.backend = collision_backend_;
    active_transaction_.scene_id = active_scene_id_;
    active_transaction_.scene_received_at =
        active_octomap_snapshot_.received_at;
    active_transaction_.options = collision_options_;
    collision_checker_->resetStatistics();
    const auto start_valid = collision_checker_->isStateValid(start);
    if (!start_valid.valid) {
      failPlanning(response, "START_IN_COLLISION", start_valid.reason);
      publishCollisionDiagnostics(start, goal, start_valid, "START");
      return;
    }
    const auto goal_valid = collision_checker_->isStateValid(goal);
    if (!goal_valid.valid) {
      failPlanning(response, "GOAL_IN_COLLISION", goal_valid.reason);
      publishCollisionDiagnostics(start, goal, goal_valid, "GOAL");
      return;
    }
    const auto direct = collision_checker_->isEdgeValid(
        start, goal, collision_options_.edge_resolution);

    const auto plan =
        collision_planner_->plan(start, goal, collision_options_);
    if (!plan.success) {
      const auto statistics = collision_checker_->statistics();
      std::ostringstream reason;
      reason << "IK=SUCCESS START=FREE GOAL=FREE DIRECT_EDGE="
             << (direct.valid ? "FREE" : "COLLISION")
             << " PLANNER=FAILED failure=" << plan.message
             << " iterations=" << plan.iterations
             << " state_checks=" << statistics.state_checks
             << " edge_checks=" << statistics.edge_checks
             << " environment_queries=" << statistics.environment_queries
             << " collision_checking_time_ms="
             << statistics.collision_checking_time_ms
             << " planning_time_ms=" << plan.planning_time_ms;
      failPlanning(response, "PLANNING_FAILED", reason.str());
      if (!direct.valid)
        publishCollisionDiagnostics(start, goal, direct.first_invalid,
                                    "DIRECT EDGE");
      return;
    }
    const auto timed =
        buildTimedTrajectory(plan.path(), timingOptions());
    if (!timed.success) {
      failPlanning(response, "TRAJECTORY_GENERATION_FAILED", timed.message);
      return;
    }
    const auto final_validation =
        validateTimedTrajectoryCollision(timed.trajectory);
    if (!final_validation.ok) {
      failPlanning(response, "TRAJECTORY_COLLISION",
                   final_validation.reason);
      return;
    }

    candidate_ = request_candidate;
    manual_ = true;
    sequence_active_ = true;
    sequence_planned_ = true;
    active_planning_request_id_ = request_id;
    prepared_trajectory_ = timed.trajectory;
    planned_path_raw_ = plan.raw_path;
    planned_path_simplified_ = plan.path();
    const auto statistics = collision_checker_->statistics();

    std::ostringstream status;
    status << "REQUEST_ID=" << request_id
           << " ENVIRONMENT="
           << collision::collisionEnvironmentBackendName(collision_backend_)
           << " SCENE_ID=" << active_scene_id_;
    if (collision_backend_ ==
        collision::CollisionEnvironmentBackend::OctoMap) {
      status << " SCENE_AGE=" << sceneAgeSeconds(active_octomap_snapshot_)
             << " SCENE_STATUS=AVAILABLE";
    }
    status << " IK=SUCCESS START=FREE GOAL=FREE DIRECT_EDGE="
           << (direct.valid ? "FREE" : "COLLISION")
           << " DIRECT_PATH=" << (direct.valid ? "FREE" : "COLLISION")
           << " PLANNER=" << (plan.direct_path ? "DIRECT" : "RRT_CONNECT")
           << " iterations=" << plan.iterations
           << " state_checks=" << statistics.state_checks
           << " edge_checks=" << statistics.edge_checks
           << " environment_queries=" << statistics.environment_queries
           << " collision_checking_time_ms="
           << statistics.collision_checking_time_ms
           << " planning_time_ms=" << plan.planning_time_ms
           << " raw_waypoints=" << plan.raw_path.size()
           << " simplified_waypoints=" << plan.path().size()
           << " trajectory_points=" << prepared_trajectory_.states.size()
           << " FINAL_SNAPSHOT_VALIDATION=PASS final_validation=PASS";
    planning_status_ = status.str();
    logLiftTrace("planned", "success", measured.q[0]);
    RCLCPP_INFO(get_logger(), "%s", planning_status_.c_str());
    publishPlannedMarkers(plan.raw_path, plan.path(), prepared_trajectory_);

    if (switch_mode) {
      if (!trajectory_mode_client_ ||
          !trajectory_mode_client_->service_is_ready()) {
        failPlanning(response, "EXECUTION_REJECTED",
                     "trajectory mode service unavailable");
        return;
      }
      preparing_mode_ = true;
      mode_request_sent_ = steady();
      trajectory_mode_client_->async_send_request(
          std::make_shared<Trigger::Request>(),
          [this, request_id](rclcpp::Client<Trigger>::SharedFuture future) {
            if (!preparing_mode_ || !sequence_active_ || !sequence_planned_ ||
                active_planning_request_id_ != request_id)
              return;
            if (!future.get()->success) {
              Trigger::Response response;
              failPlanning(response, "EXECUTION_REJECTED",
                           "driver rejected trajectory mode");
            }
          });
      response.success = true;
      response.message = planning_status_ + "; preparing trajectory mode";
      publish();
      return;
    }

    executePreparedPlannedTrajectory(response);
  }

  Check validateLiveSceneBeforeExecution() {
    if (active_planning_request_id_ != planning_request_id_ ||
        active_transaction_.request_id != active_planning_request_id_) {
      return {false, "STALE_PLANNING_RESULT: request superseded"};
    }
    if (collision_backend_ ==
        collision::CollisionEnvironmentBackend::Primitive) {
      return validateTimedTrajectoryCollision(prepared_trajectory_);
    }

    const auto latest = octomap_scene_store_->snapshot();
    if (!latest.available()) {
      return {false, std::string(collision::sceneFreshnessName(latest.status)) +
                         ": " + latest.reason};
    }
    if (latest.id == active_scene_id_) {
      const auto validation =
          validateTimedTrajectoryCollision(prepared_trajectory_);
      if (!validation.ok) return validation;
      return {true, "LIVE_PRE_EXECUTION_VALIDATION=PASS scene_unchanged"};
    }

    try {
      auto latest_checker =
          std::make_unique<collision::StateValidityChecker>(
              model_, active_bounds_, robot_collision_model_,
              withoutSelf(latest.environment));
      const auto validation = validateTimedTrajectoryCollision(
          prepared_trajectory_, latest_checker.get());
      if (!validation.ok) {
        return {false, "LIVE_SCENE_CHANGED: newest scene " +
                           std::to_string(latest.id) + " rejects trajectory: " +
                           validation.reason};
      }
      collision_checker_ = std::move(latest_checker);
      collision_planner_ =
          std::make_unique<collision::RrtConnectPlanner>(*collision_checker_);
      active_scene_id_ = latest.id;
      active_octomap_snapshot_ = latest;
      active_transaction_.scene_id = latest.id;
      active_transaction_.scene_received_at = latest.received_at;
      return {true, "LIVE_PRE_EXECUTION_VALIDATION=PASS revalidated_scene=" +
                        std::to_string(latest.id)};
    } catch (const std::exception& e) {
      return {false, std::string("SCENE_UNAVAILABLE: ") + e.what()};
    }
  }

  void executePreparedPlannedTrajectory(Trigger::Response& response) {
    if (!sequence_active_ || !sequence_planned_ ||
        prepared_trajectory_.states.empty()) {
      failPlanning(response, "PATH_INVALID",
                   "no prepared collision-planned trajectory");
      return;
    }
    const auto live_validation = validateLiveSceneBeforeExecution();
    if (!live_validation.ok) {
      failPlanning(response, "LIVE_PRE_EXECUTION_REJECTED",
                   live_validation.reason);
      return;
    }
    planning_status_ += "; " + live_validation.reason;
    const auto measured = input();
    auto check = guard_->validateTrajectory(
        measured, candidate_, prepared_trajectory_.states,
        prepared_trajectory_.times_from_start, steady());
    if (!check.ok) {
      failPlanning(response, "TRAJECTORY_GENERATION_FAILED", check.reason);
      return;
    }
    execute(response);
    if (response.success) {
      planning_status_ += "; EXECUTION=STARTED";
    } else {
      planning_status_ += "; EXECUTION_REJECTED=" + response.message;
      sequence_active_ = false;
      sequence_planned_ = false;
      prepared_trajectory_ = {};
      manual_ = false;
    }
    if (dry_) {
      sequence_active_ = false;
      sequence_planned_ = false;
      prepared_trajectory_ = {};
      manual_ = false;
    }
    publish();
  }
  void startNextSegment(Trigger::Response& response) {
    const auto measured = input();
    if (!measured.fresh || !measured.required_joints || !measured.q.allFinite()) {
      sequence_active_ = false;
      manual_ = false;
      guard_->abort("measured state unavailable before next segment");
      response.message = guard_->reason();
      publish();
      return;
    }
    const Vector delta = sequence_target_ - measured.q;
    if ((delta.cwiseAbs().array() <= config_.goal_tolerance.array()).all()) {
      sequence_active_ = false;
      manual_ = false;
      if (guard_->armed()) guard_->disarm();
      response.success = true;
      response.message = "5-DoF target reached (measured)";
      publish();
      return;
    }
    guard_->disarm();
    auto check = guard_->arm();
    candidate_ = {};
    candidate_.seed = measured.q;
    double fraction = 1.;
    for (int i = 0; i < 5; ++i)
      if (std::abs(delta[i]) > 0.)
        fraction = std::min(fraction, config_.max_delta[i] / std::abs(delta[i]));
    candidate_.q = measured.q + fraction * delta;
    candidate_.valid = true;
    candidate_.received = steady();
    candidate_.revision = ++revision_;
    if (check.ok)
      check = guard_->validate(measured, candidate_,
                               segmentDuration(candidate_.q - measured.q),
                               steady());
    if (check.ok) execute(response);
    else response.message = check.reason;
    if (!response.success || dry_) {
      sequence_active_ = false;
      manual_ = false;
      if (!response.success) guard_->abort(response.message);
    }
    publish();
  }
  void execute(Trigger::Response& r) {
    if (!guard_ || lift_goal_) {
      r.message = lift_goal_ ? "absolute lift execution already active"
                             : "model unavailable";
      return;
    }
    refreshCandidate();
    if (!dry_ && (!action_ || !action_->action_server_is_ready() ||
                  (!sim_bypass_hardware_checks_ &&
                   (!stop_client_->service_is_ready() ||
                    !runstop_client_->service_is_ready())))) {
      guard_->abort("hardware action/stop/runstop interface unavailable");
      r.message = guard_->reason();
      publish();
      return;
    }
    auto c = guard_->begin(input(), candidate_, steady());
    if (!c.ok) {
      r.message = c.reason;
      publish();
      return;
    }
    if (dry_) {
      writeDryReport(input(), {true, "fully validated dry-run plan"});
      guard_->completeDryRun();
      r.success = true;
      r.message = guard_->reason();
      candidate_.valid = false;
      publish();
      return;
    }
    std::string message;
    r.success = submitTrajectory(message);
    r.message = message;
    publish();
  }

  bool submitTrajectory(std::string& message) {
    if (!guard_ || guard_->state() != State::EXECUTING || !guard_->plan()) {
      message = "no approved executing lift plan";
      return false;
    }
    auto trajectory = Guard::trajectory(*guard_->plan(), config_.lift_only);
    commanded_lift_ = trajectory.points.empty()
                            ? std::numeric_limits<double>::quiet_NaN()
                            : trajectory.points.back().positions.front();
    logLiftTrace("commanded", "submitted", input().q[0]);
    TrajectoryAction::Goal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance =
        rclcpp::Duration::from_seconds(config_.execution_slack);
    auto append_tolerances = [&](const Vector& logical,
                                 std::vector<control_msgs::msg::JointTolerance>& out) {
      out.clear();
      const auto names = Guard::trajectoryJointNames(config_.lift_only);
      for (const auto& name : names) {
        control_msgs::msg::JointTolerance tol;
        tol.name = name;
        if (name == "joint_lift")
          tol.position = logical[0];
        else if (name.rfind("joint_arm_l", 0) == 0)
          tol.position = logical[1] / 4.0;
        else if (name == "joint_wrist_yaw")
          tol.position = logical[2];
        else if (name == "joint_wrist_pitch")
          tol.position = logical[3];
        else if (name == "joint_wrist_roll")
          tol.position = logical[4];
        out.push_back(tol);
      }
    };
    append_tolerances(config_.tracking_error, goal.path_tolerance);
    append_tolerances(config_.goal_tolerance, goal.goal_tolerance);
    pending_ = true;
    accepted_ = false;
    tracking_synchronized_ = false;
    max_tracking_error_.setZero();
    sent_time_ = feedback_time_ = steady();
    stop_unconfirmed_ = false;
    runstop_requested_ = false;
    rclcpp_action::Client<TrajectoryAction>::SendGoalOptions opt;
    opt.goal_response_callback = [this](TrajectoryHandle::SharedPtr h) {
      handle_ = h;
      if (!h) {
        pending_ = false;
        if (guard_->state() == State::EXECUTING)
          guard_->abort("controller rejected goal");
        publish();
        return;
      }
      accepted_ = true;
      feedback_time_ = steady();
      if (guard_->state() != State::EXECUTING) sendStop();
    };
    opt.feedback_callback = [this](
                                TrajectoryHandle::SharedPtr,
                                const std::shared_ptr<const TrajectoryAction::Feedback>) {
      const double now = steady();
      feedback_time_ = now;
      if (!tracking_synchronized_ && guard_ &&
          guard_->state() == State::EXECUTING) {
        auto c = guard_->synchronizeStart(input(), now);
        if (!c.ok) {
          sendStop();
        } else {
          tracking_synchronized_ = true;
          RCLCPP_INFO(get_logger(), "%s", c.reason.c_str());
        }
        publish();
      }
    };
    opt.result_callback = [this](const TrajectoryHandle::WrappedResult& result) {
      onTrajectoryResult(result);
    };
    try {
      action_->async_send_goal(goal, opt);
      message =
          "joint trajectory submitted; monitor status; auto-disarm at "
          "terminal result";
      return true;
    } catch (const std::exception& e) {
      guard_->abort(std::string("send goal failed: ") + e.what());
      sendStop();
      message = guard_->reason();
      return false;
    }
  }

  void onTrajectoryResult(const TrajectoryHandle::WrappedResult& result) {
      pending_ = false;
      handle_.reset();
      if (lift_goal_ && lift_cancel_requested_ &&
          result.code == rclcpp_action::ResultCode::CANCELED) {
        stop_unconfirmed_ = false;
        if (guard_) guard_->abort("lift execution canceled by operator");
        candidate_.valid = false;
        finishLiftAction(false, true,
                         "lift execution canceled; controller confirmed stop");
        publish();
        return;
      }
      if (stop_unconfirmed_ &&
          result.code == rclcpp_action::ResultCode::CANCELED)
        stop_unconfirmed_ = false;
      if (guard_->state() == State::EXECUTING) {
        if (!tracking_synchronized_) {
          guard_->abort("controller result received before first feedback");
          sendStop();
          candidate_.valid = false;
          failLiftAction(guard_->reason());
          publish();
          return;
        }
        bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                       result.result &&
                       result.result->error_code ==
                           TrajectoryAction::Result::SUCCESSFUL;
        if (!success) {
          std::ostringstream reason;
          reason << "controller failure: action_result_code="
                 << static_cast<int>(result.code);
          if (result.result)
            reason << " controller_error_code=" << result.result->error_code
                   << " controller_error='" << result.result->error_string
                   << "'";
          else
            reason << " controller_result_unavailable";
          guard_->abort(reason.str());
          logLiftTrace("execution_failed", reason.str(), input().q[0]);
          lift_trace_active_ = false;
          RCLCPP_ERROR(get_logger(), "%s", reason.str().c_str());
          sendStop();
          failLiftAction(reason.str());
        } else {
          controller_success_pending_ = true;
          controller_result_time_ = steady();
          publish();
          return;
        }
      }
      candidate_.valid = false;
      publish();
  }

  void completeSuccessfulTrajectory() {
    if (!controller_success_pending_ || !guard_ ||
        guard_->state() != State::EXECUTING)
      return;
    controller_success_pending_ = false;
    double finish_time = steady();
    if (sim_bypass_hardware_checks_ && guard_->plan())
      finish_time = guard_->plan()->started_at + guard_->plan()->duration;
    auto c = guard_->finish(input(), true, finish_time);
    if (!c.ok) {
      logLiftTrace("execution_failed", c.reason, input().q[0]);
      lift_trace_active_ = false;
      sendStop();
      failLiftAction(c.reason);
    } else if (lift_goal_) {
      const auto measured = input();
      const double endpoint = measured.q[0];
      RCLCPP_INFO(get_logger(),
                  "Lift segment %u/%u succeeded: target=%.9f "
                  "measured=%.9f error=%.9f joints=[joint_lift]",
                  lift_sequence_.currentSegment(),
                  lift_sequence_.totalSegments(), segment_target_, endpoint,
                  endpoint - segment_target_);
      lift_sequence_.segmentComplete(endpoint);
      publishLiftFeedback(lift_sequence_.active() ? "SEGMENT_COMPLETE"
                                                  : "COMPLETE");
      if (lift_sequence_.active())
        lift_next_pending_ = true;
      else
        finishLiftAction(true, false,
                         "absolute lift target reached and measured");
    }
    if (c.ok && !lift_goal_) {
      logLiftTrace("execution_finished", "success", input().q[0]);
      lift_trace_active_ = false;
    }
    candidate_.valid = false;
    publish();
  }

  rclcpp_action::GoalResponse liftGoal(
      const std::shared_ptr<const LiftAction::Goal> goal) {
    if (!goal || sequence_active_ || !std::isfinite(goal->target_position) || !bounds_ready_ ||
        lift_goal_ || lift_sequence_.active() || pending_ || stop_unconfirmed_)
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->target_position < active_bounds_.lower[0] ||
        goal->target_position > active_bounds_.upper[0])
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse liftCancel(
      const std::shared_ptr<LiftHandle> goal) {
    if (!lift_goal_ || goal != lift_goal_)
      return rclcpp_action::CancelResponse::REJECT;
    lift_cancel_requested_ = true;
    lift_state_ = "CANCELING";
    if (pending_ || (guard_ && guard_->state() == State::EXECUTING)) sendStop();
    publishLiftFeedback(lift_state_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void liftAccepted(const std::shared_ptr<LiftHandle> goal) {
    if (!goal || lift_goal_) {
      if (goal) {
        auto result = std::make_shared<LiftAction::Result>();
        result->message = "another lift execution is active";
        goal->abort(result);
      }
      return;
    }
    const auto measured = input();
    lift_goal_ = goal;
    lift_target_display_ = goal->get_goal()->target_position;
    lift_max_tracking_error_ = 0.;
    if (!guard_) {
      failLiftAction("guard model unavailable");
      return;
    }
    if (guard_->state() != State::DISARMED) guard_->disarm();
    const auto armed = guard_->arm();
    if (!armed.ok) {
      failLiftAction(armed.reason);
      return;
    }
    const auto preflight = guard_->preflight(measured);
    if (!preflight.ok) {
      failLiftAction(preflight.reason);
      return;
    }
    auto check = lift_sequence_.start(
        measured.q[0], goal->get_goal()->target_position,
        active_bounds_.lower[0], active_bounds_.upper[0], config_.max_delta[0],
        config_.goal_tolerance[0]);
    lift_cancel_requested_ = false;
    if (!check.ok) {
      failLiftAction(check.reason);
      return;
    }
    if (!lift_sequence_.active()) {
      finishLiftAction(true, false, check.reason);
      return;
    }
    if (dry_) {
      finishLiftAction(false, false,
                       "DRY RUN: absolute target accepted; no physical "
                       "segments sent");
      return;
    }
    lift_state_ = "READY";
    lift_next_pending_ = true;
    publishLiftFeedback(lift_state_);
    publish();
  }

  void startNextLiftSegment() {
    lift_next_pending_ = false;
    if (!lift_goal_ || lift_cancel_requested_) return;
    const auto measured = input();
    auto next = lift_sequence_.next(measured.q[0]);
    if (!next) {
      if (lift_sequence_.satisfied(measured.q[0]))
        finishLiftAction(true, false, "absolute lift target reached");
      else
        failLiftAction("cannot generate next bounded lift segment");
      return;
    }
    if (!guard_) {
      failLiftAction("guard model unavailable");
      return;
    }
    if (guard_->state() != State::DISARMED) guard_->disarm();
    auto armed = guard_->arm();
    if (!armed.ok) {
      failLiftAction(armed.reason);
      return;
    }
    manual_ = true;
    candidate_ = {};
    candidate_.q = measured.q;
    candidate_.seed = measured.q;
    candidate_.q[0] = *next;
    candidate_.received = steady();
    candidate_.valid = true;
    candidate_.revision = ++revision_;
    segment_target_ = *next;
    auto approved =
        guard_->validate(measured, candidate_, duration_, steady());
    if (!approved.ok) {
      failLiftAction(approved.reason);
      return;
    }
    auto begun = guard_->begin(input(), candidate_, steady());
    if (!begun.ok) {
      failLiftAction(begun.reason);
      return;
    }
    std::string message;
    if (!submitTrajectory(message)) {
      failLiftAction(message);
      return;
    }
    lift_state_ = "EXECUTING";
    publishLiftFeedback(lift_state_);
    publish();
  }

  void publishLiftFeedback(const std::string& state) {
    if (!lift_goal_) return;
    auto feedback = std::make_shared<LiftAction::Feedback>();
    const auto measured = input();
    feedback->current_position = measured.q[0];
    feedback->target_position = lift_sequence_.target();
    feedback->current_segment = lift_sequence_.currentSegment();
    feedback->total_segments = lift_sequence_.totalSegments();
    feedback->state = state;
    lift_goal_->publish_feedback(feedback);
    lift_state_ = state;
    publishLiftStatus();
  }

  void failLiftAction(const std::string& reason) {
    if (!lift_goal_) return;
    lift_sequence_.fail(reason);
    finishLiftAction(false, false, reason);
  }

  void finishLiftAction(bool success, bool canceled,
                        const std::string& message) {
    if (!lift_goal_) return;
    auto result = std::make_shared<LiftAction::Result>();
    const auto measured = input();
    result->success = success;
    result->final_position = measured.q[0];
    result->message = message;
    auto goal = lift_goal_;
    lift_goal_.reset();
    lift_next_pending_ = false;
    if (canceled) {
      lift_sequence_.cancel();
      lift_state_ = "CANCELED";
      goal->canceled(result);
    } else if (success) {
      lift_state_ = "COMPLETE";
      goal->succeed(result);
    } else {
      lift_state_ = "ABORTED";
      goal->abort(result);
    }
    if (guard_ && guard_->state() != State::EXECUTING && guard_->armed())
      guard_->disarm();
    publishLiftStatus();
  }
  void stopAndDisarm(bool stop) {
    ++planning_request_id_;
    active_planning_request_id_ = 0;
    preparing_mode_ = replan_after_mode_ = false;
    sequence_active_ = false;
    sequence_planned_ = false;
    prepared_trajectory_ = {};
    planned_path_raw_.clear();
    planned_path_simplified_.clear();
    planning_status_ = stop ? "STOP: pending collision plan invalidated"
                            : "DISARM: pending collision plan invalidated";
    clearPlannedMarkers();
    bool moving = pending_ || (guard_ && guard_->state() == State::EXECUTING);
    if (guard_) {
      if (stop || moving)
        guard_->abort(stop ? "operator STOP" : "operator disarm");
      else
        guard_->disarm();
    }
    candidate_.valid = false;
    manual_ = false;
    if (lift_goal_) {
      lift_cancel_requested_ = true;
      lift_state_ = "CANCELING";
    }
    if (!dry_ && (stop || moving)) sendStop();
    publish();
  }
  void sendStop() {
    if (dry_ || !action_) return;
    controller_success_pending_ = false;
    if (sim_bypass_hardware_checks_) {
      if (handle_) action_->async_cancel_goal(handle_);
      stop_unconfirmed_ = false;
      return;
    }
    stop_unconfirmed_ = true;
    stop_sent_ = steady();
    if (handle_) action_->async_cancel_goal(handle_);
    // Late goal responses are always canceled too. An unacknowledged goal gets
    // immediate runstop protection against arriving after the first stop
    // request.
    if (pending_ && !handle_) requestRunstop();
    if (stop_client_->service_is_ready())
      stop_client_->async_send_request(
          std::make_shared<Trigger::Request>(),
          [this](rclcpp::Client<Trigger>::SharedFuture f) {
            const auto response = f.get();
            if (response->success) {
              stop_unconfirmed_ = false;
              publish();
            } else {
              requestRunstop();
            }
          });
    else
      requestRunstop();
  }
  void requestRunstop() {
    if (dry_ || !runstop_client_) return;
    runstop_requested_ = true;
    if (runstop_client_->service_is_ready()) {
      auto r = std::make_shared<SetBool::Request>();
      r->data = true;
      runstop_client_->async_send_request(
          r, [this](rclcpp::Client<SetBool>::SharedFuture f) {
            if (!f.get()->success)
              RCLCPP_ERROR(get_logger(),
                           "Runstop request failed: use physical runstop");
          });
    } else
      RCLCPP_ERROR(get_logger(),
                   "STOP NOT CONFIRMED: use physical runstop; never "
                   "automatically rearmed");
  }
  void writeDryReport(const Input& in, const Check& check) {
    std::ofstream f(report_path_);
    if (!f) {
      RCLCPP_ERROR(get_logger(), "Cannot write dry-run report");
      return;
    }
    f << std::setprecision(17)
      << "{\n\"dry_run\":true,\"hardware_action_client_created\":"
      << (action_ ? "true" : "false")
      << ",\"approved\":" << (check.ok ? "true" : "false") << ",\"reason\":\""
      << check.reason << "\",\n\"live_mode\":\"" << mode_
      << "\",\"homed\":" << (in.homed ? "true" : "false")
      << ",\"runstopped\":" << (in.runstopped ? "true" : "false")
      << ",\"state_fresh\":" << (in.fresh ? "true" : "false")
      << ",\"diagnostics_ok\":" << (in.diagnostics_ok ? "true" : "false")
      << ",\"action_server_available\":"
      << (in.action_server_available ? "true" : "false")
      << ",\"status_message_received\":"
      << (in.status_message_received ? "true" : "false")
      << ",\"active_goal_present\":"
      << (in.active_goal_present ? "true" : "false")
      << ",\"controller_idle\":" << (in.controller_idle ? "true" : "false")
      << ",\"startpoint_guard_verified\":"
      << (in.startpoint_guard_verified ? "true" : "false") << ",\n\"points\":[";
    // Inspection-only trajectory can be generated even when physical readiness
    // blocks approval. It is labelled approved=false and is never published.
    try {
      if (guard_ && guard_->plan()) {
        auto t = Guard::trajectory(*guard_->plan(), config_.lift_only);
        for (size_t i = 0; i < t.points.size(); ++i) {
          auto& v = t.points[i];
          if (i) f << ",";
          f << "{\"t\":" << rclcpp::Duration(v.time_from_start).seconds();
          for (size_t axis = 0; axis < v.positions.size(); ++axis)
            f << ",\"p" << axis << "\":" << v.positions[axis];
          f << "}";
        }
      } else if (in.fresh && candidate_.valid) {
        Plan p;
        p.start = in.q;
        p.target = config_.lift_only ? in.q : candidate_.q;
        p.target[0] = candidate_.relative_lift
                          ? in.q[0] + *candidate_.relative_lift
                          : candidate_.q[0];
        p.duration = duration_;
        auto t = Guard::trajectory(p, config_.lift_only);
        for (size_t i = 0; i < t.points.size(); ++i) {
          auto& v = t.points[i];
          if (i) f << ",";
          f << "{\"t\":" << rclcpp::Duration(v.time_from_start).seconds()
            << ",\"lift\":" << v.positions[0]
            << ",\"velocity\":" << v.velocities[0]
            << ",\"acceleration\":" << v.accelerations[0] << "}";
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Inspection trajectory rejected: %s", e.what());
    }
    f << "]\n}\n";
  }
  void publish() {
    last_publish_ = steady();
    std_msgs::msg::String m;
    std::ostringstream s;
    const auto controller =
        controller_state_.preflight(actionServerAvailable(), pending_);
    s << "state=" << (guard_ ? stateName(guard_->state()) : "DISARMED")
      << " armed=" << (guard_ && guard_->armed()) << " dry_run=" << dry_
      << " enabled=" << config_.execution_enabled
      << " hardware_client=" << bool(action_) << " pending_goal=" << pending_
      << " action_server_available=" << controller.action_server_available
      << " status_message_received=" << controller.status_message_received
      << " active_goal_present=" << controller.active_goal_present
      << " controller_idle=" << controller.controller_idle
      << " tracking_synchronized=" << tracking_synchronized_
      << " stop_unconfirmed=" << stop_unconfirmed_ << " mode=" << mode_
      << " startpoint_guard=" << start_guard_
      << " candidate_revision=" << candidate_.revision
      << " sequence_active=" << sequence_active_
      << " sequence_planned=" << sequence_planned_
      << " sequence_target=[" << sequence_target_.transpose() << "]"
      << " planned_trajectory_points="
      << prepared_trajectory_.states.size()
      << " planning_status={" << planning_status_ << "}"
      << " reason=" << (guard_ ? guard_->reason() : "waiting for model");
    if (guard_ && guard_->plan())
      s << " start=[" << guard_->plan()->start.transpose() << "] target=["
        << guard_->plan()->target.transpose()
        << "] duration=" << guard_->plan()->duration;
    s << " max_tracking_error=[" << max_tracking_error_.transpose() << "]";
    s << " diagnostic=" << diag_reason_;
    m.data = s.str();
    status_->publish(m);
    publishLiftStatus();
  }
  void publishLiftStatus() {
    if (!lift_status_) return;
    std_msgs::msg::String message;
    const auto measured = input();
    std::ostringstream text;
    text << "state=" << lift_state_ << " active=" << bool(lift_goal_)
         << " current=" << measured.q[0]
         << " target=" << lift_target_display_
         << " segment="
         << (lift_goal_ ? lift_sequence_.currentSegment() : 0)
         << " total=" << (lift_goal_ ? lift_sequence_.totalSegments() : 0)
         << " max_tracking_error=" << lift_max_tracking_error_
         << " joints=[joint_lift]";
    message.data = text.str();
    lift_status_->publish(message);
  }
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_lock_;
  Config config_;
  DiagnosticPolicy diagnostic_policy_;
  ControllerState controller_state_;
  Bounds hardware_;
  Bounds active_bounds_;
  bool bounds_ready_ = false;
  std::unique_ptr<Guard> guard_;
  LiftSequence lift_sequence_;
  const void* model_identity_ = nullptr;
  bool dry_ = true, sim_bypass_hardware_checks_ = false, transport_test_mode_ = false, manual_ = false,
       streaming_ = true, diag_ok_ = false,
       start_guard_ = false, param_pending_ = false, preview_ok_ = false,
       pending_ = false, accepted_ = false, stop_unconfirmed_ = false,
       runstop_requested_ = false, tracking_synchronized_ = false,
       lift_next_pending_ = false, lift_cancel_requested_ = false,
       controller_success_pending_ = false,
       collision_planning_enabled_ = false,
       collision_marker_enabled_ = true;
  double duration_ = 5, lift_delta_ = .01, mode_time_ = -1, stream_time_ = -1,
         diag_time_ = -1, param_time_ = -1, last_param_query_ = -1,
         param_sent_ = -1, preview_time_ = -1, preview_status_time_ = -1,
         preview_stamp_ = 0, sent_time_ = 0, feedback_time_ = 0, stop_sent_ = 0,
         last_publish_ = 0, segment_target_ = 0, lift_target_display_ = 0,
         lift_max_tracking_error_ = 0, controller_result_time_ = 0,
         preview_target_z_ = std::numeric_limits<double>::quiet_NaN(),
         preview_transformed_z_ = std::numeric_limits<double>::quiet_NaN(),
         commanded_lift_ = std::numeric_limits<double>::quiet_NaN();
  std::string preview_target_frame_;
  Eigen::Isometry3d odom_base_ = Eigen::Isometry3d::Identity();
  std::string odom_frame_;
  double odom_time_ = -1, self_ignore_margin_ = 0.08, mode_ready_since_ = -1;
  bool replan_after_mode_ = false;
  bool odom_still_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  int64_t param_request_id_ = 0;
  uint64_t revision_ = 0;
  Candidate candidate_;
  bool lift_trace_active_ = false;
  bool sequence_active_ = false, sequence_planned_ = false,
       preparing_mode_ = false;
  double mode_request_sent_ = 0.;
  Vector sequence_target_ = Vector::Zero();
  collision::PlannerOptions collision_options_;
  collision::CollisionEnvironmentBackend collision_backend_ =
      collision::CollisionEnvironmentBackend::Primitive;
  std::shared_ptr<const collision::RobotCollisionModel> robot_collision_model_;
  std::unique_ptr<collision::OctoMapSceneStore> octomap_scene_store_;
  collision::OctoMapSceneSnapshot active_octomap_snapshot_;
  PlanningTransaction active_transaction_;
  uint64_t active_scene_id_ = 0;
  std::unique_ptr<collision::StateValidityChecker> collision_checker_;
  std::unique_ptr<collision::RrtConnectPlanner> collision_planner_;
  TimedTrajectory prepared_trajectory_;
  std::vector<Vector> planned_path_raw_, planned_path_simplified_;
  Vector preview_q_ = Vector::Zero(), status_q_ = Vector::Zero(),
         preview_seed_ = Vector::Zero(), max_tracking_error_ = Vector::Zero();
  struct DiagSource { double time; std::string blocking, warnings; };
  std::map<std::string, DiagSource> diag_sources_;
  std::string mode_ = "unknown", diag_reason_, report_path_, action_name_,
              octomap_topic_, lift_state_ = "DISARMED",
              planning_status_ = "collision_planning=uninitialized";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_, lift_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      planned_path_markers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stream_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_sub_,
      preview_status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr preview_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      action_status_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr parameters_;
  rclcpp_action::Client<TrajectoryAction>::SharedPtr action_;
  TrajectoryHandle::SharedPtr handle_;
  rclcpp_action::Server<LiftAction>::SharedPtr lift_action_server_;
  std::shared_ptr<LiftHandle> lift_goal_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_, trajectory_mode_client_;
  rclcpp::Client<SetBool>::SharedPtr runstop_client_;
  rclcpp::Service<SetBool>::SharedPtr arm_service_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t planning_request_id_ = 0, active_planning_request_id_ = 0;
};
// Keep ROS alive for a short cancellation drain on SIGINT/SIGTERM.
#include <csignal>
namespace {
volatile std::sig_atomic_t stopping = 0;
void signalStop(int) { stopping = 1; }
}  // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signalStop);
  std::signal(SIGTERM, signalStop);
  try {
    auto node = std::make_shared<Executor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    try {
      while (rclcpp::ok() && !stopping)
        executor.spin_once(std::chrono::milliseconds(20));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Executor fault, requesting STOP: %s",
                   e.what());
      node->shutdownStop();
      auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
        executor.spin_once(std::chrono::milliseconds(20));
      throw;
    }
    node->shutdownStop();
    auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
      executor.spin_once(std::chrono::milliseconds(20));
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
