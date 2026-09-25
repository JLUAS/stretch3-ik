#include <action_msgs/msg/goal_status_array.hpp>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "stretch_guarded_executor/controller_state.hpp"
#include "stretch_guarded_executor/diagnostic_policy.hpp"
#include "stretch_guarded_executor/action/execute_lift_target.hpp"
#include "stretch_guarded_executor/guard.hpp"
#include "stretch_guarded_executor/lift_sequence.hpp"
#include "stretch_robot_interface/interface_node.hpp"
using namespace stretch_guarded_executor;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
using TrajectoryHandle = rclcpp_action::ClientGoalHandle<TrajectoryAction>;
using LiftAction = stretch_guarded_executor::action::ExecuteLiftTarget;
using LiftHandle = rclcpp_action::ServerGoalHandle<LiftAction>;
class Executor : public stretch_robot_interface::InterfaceNode {
 public:
  Executor() : InterfaceNode("executor") {
    if (std::string(get_namespace()) != "/guarded_executor")
      throw std::invalid_argument("Required namespace: /guarded_executor");
    dry_ = immutable<bool>("dry_run", true);
    sim_bypass_hardware_checks_ = immutable<bool>("sim_bypass_hardware_checks", false);
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
    if (action_name_ != "/stretch_controller/follow_joint_trajectory")
      throw std::invalid_argument(
          "Only the inspected Stretch action is supported");
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
          diag_ok_ = !m->status.empty();
          diag_reason_.clear();
          for (const auto& s : m->status) {
            const auto decision = diagnostic_policy_.classify(s);
            if (decision.blocking()) {
              diag_ok_ = false;
              diag_reason_ += s.name + ": " + s.message + "; ";
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
              diag_reason_ += "WARN: " + s.name + ": " + decision.reason + "; ";
            }
          }
          diag_time_ = steady();
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
          auto check = guard_->validate(in, candidate_, duration_, steady());
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
    if (get_parameter("sim_bypass_hardware_checks").as_bool()) {
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
      auto c = tracking_synchronized_
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
    if (preparing_mode_ && sequence_active_) {
      const auto measured = input();
      if (measured.mode_fresh && measured.trajectory_mode) {
        preparing_mode_ = false;
        Trigger::Response response;
        startNextSegment(response);
      } else if (steady() - mode_request_sent_ > 3.) {
        preparing_mode_ = false;
        sequence_active_ = false;
        manual_ = false;
        guard_->abort("timeout waiting for trajectory mode");
        publish();
      }
    }
    if (sequence_active_ && guard_->state() == State::ABORTED) {
      sequence_active_ = false;
      manual_ = false;
    }
    if (sequence_active_ && !pending_ && !stop_unconfirmed_ &&
        guard_->state() == State::COMPLETE) {
      Trigger::Response response;
      startNextSegment(response);
    }
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
    if (check.ok) check = guard_->validate(measured, candidate_, duration_, steady());
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
                  !stop_client_->service_is_ready() ||
                  !runstop_client_->service_is_ready())) {
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
    TrajectoryAction::Goal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance =
        rclcpp::Duration::from_seconds(config_.execution_slack);
    control_msgs::msg::JointTolerance tol;
    tol.name = "joint_lift";
    tol.position = config_.tracking_error[0];
    goal.path_tolerance = {tol};
    tol.position = config_.goal_tolerance[0];
    goal.goal_tolerance = {tol};
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
    auto c = guard_->finish(input(), true, steady());
    if (!c.ok) {
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
    preparing_mode_ = false;
    sequence_active_ = false;
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
      if (in.fresh && candidate_.valid) {
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
      << " sequence_target=[" << sequence_target_.transpose() << "]"
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
       controller_success_pending_ = false;
  double duration_ = 5, lift_delta_ = .01, mode_time_ = -1, stream_time_ = -1,
         diag_time_ = -1, param_time_ = -1, last_param_query_ = -1,
         param_sent_ = -1, preview_time_ = -1, preview_status_time_ = -1,
         preview_stamp_ = 0, sent_time_ = 0, feedback_time_ = 0, stop_sent_ = 0,
         last_publish_ = 0, segment_target_ = 0, lift_target_display_ = 0,
         lift_max_tracking_error_ = 0, controller_result_time_ = 0;
  int64_t param_request_id_ = 0;
  uint64_t revision_ = 0;
  Candidate candidate_;
  bool sequence_active_ = false, preparing_mode_ = false;
  double mode_request_sent_ = 0.;
  Vector sequence_target_ = Vector::Zero();
  Vector preview_q_ = Vector::Zero(), status_q_ = Vector::Zero(),
         preview_seed_ = Vector::Zero(), max_tracking_error_ = Vector::Zero();
  std::string mode_ = "unknown", diag_reason_, report_path_, action_name_,
              lift_state_ = "DISARMED";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_, lift_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stream_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_sub_,
      preview_status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr preview_sub_;
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
