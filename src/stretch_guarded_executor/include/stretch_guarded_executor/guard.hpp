#pragma once
#include <optional>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "stretch_custom_kinematics/inverse_kinematics.hpp"
namespace stretch_guarded_executor {
using Vector = stretch_custom_kinematics::JointVector;
using Bounds = stretch_custom_kinematics::JointBounds;
enum class State { DISARMED, ARMED, READY, EXECUTING, COMPLETE, ABORTED };
const char* stateName(State s);
struct Config {
  bool execution_enabled = false;
  bool lift_only = true;  // Commissioning restriction; false enables 5-DoF.
  Vector max_delta = (Vector() << .03, .03, .15, .15, .15).finished();
  Vector tracking_error = (Vector() << .01, .01, .05, .05, .05).finished();
  Vector start_drift = (Vector() << .001, .001, .005, .005, .005).finished();
  double min_duration = 3, max_duration = 10, approval_timeout = 15,
         execution_slack = 2;
  // Per-joint velocity and acceleration caps.
  // Index: 0=lift, 1=arm_extension, 2=wrist_yaw, 3=wrist_pitch, 4=wrist_roll.
  Vector max_velocity =
      (Vector() << .02, .02, .1, .1, .1).finished();
  Vector max_acceleration =
      (Vector() << .02, .02, .1, .1, .1).finished();
  Vector goal_tolerance =
      (Vector() << .0025, .0025, .015, .015, .015).finished();
  double minimum_motion = .00001;
  void validate() const;
};
struct Input {
  Vector q = Vector::Zero();
  bool fresh = false, homed = false, runstopped = true, required_joints = false;
  // Physical prerequisites are checked in dry run too, never silently waived.
  bool mode_fresh = false, trajectory_mode = false, streaming_known = false,
       streaming = false;
  bool controller_idle = false, diagnostics_fresh = false,
       diagnostics_ok = false;
  bool action_server_available = false, status_message_received = false,
       active_goal_present = false;
  bool startpoint_guard_verified = false;
};
struct Candidate {
  Vector q = Vector::Zero(), seed = Vector::Zero();
  bool valid = false;
  double received = -1;
  uint64_t revision = 0;
  std::optional<double> relative_lift;
};
struct Plan {
  Vector start = Vector::Zero(), target = Vector::Zero();
  double duration = 0, approved_at = 0, started_at = 0;
  uint64_t revision = 0;
  Vector candidate_q = Vector::Zero(), candidate_seed = Vector::Zero();
  std::optional<double> relative_lift;
};
struct Check {
  bool ok = false;
  std::string reason;
};
class Guard {
 public:
  Guard(Config config, Bounds bounds);
  Check arm();
  void disarm();
  Check validate(const Input& input, const Candidate& candidate,
                 double duration, double now);
  Check begin(const Input& input, const Candidate& candidate, double now);
  Check preflight(const Input& input) const { return prerequisites(input); }
  Check monitorPrerequisites(const Input& input);
  Check synchronizeStart(const Input& input, double now);
  Check monitor(const Input& input, double now);
  Check finish(const Input& input, bool controller_success, double now);
  void completeDryRun();
  void abort(const std::string& reason);
  void invalidate(const std::string& reason);
  State state() const { return state_; }
  bool armed() const { return armed_; }
  const std::string& reason() const { return reason_; }
  const std::optional<Plan>& plan() const { return plan_; }
  static Vector reference(const Plan& plan, double elapsed);
  static trajectory_msgs::msg::JointTrajectory trajectory(const Plan& plan,
                                                          bool lift_only);
  // Physical joint names for the trajectory message.
  static std::vector<std::string> trajectoryJointNames(bool lift_only);

 private:
  Check prerequisites(const Input& input) const;
  Check target(const Input& input, const Candidate& c, double duration,
               double now) const;
  Config config_;
  Bounds bounds_;
  State state_ = State::DISARMED;
  bool armed_ = false;
  std::optional<Plan> plan_;
  std::string reason_ = "startup: disarmed";
};
}  // namespace stretch_guarded_executor
