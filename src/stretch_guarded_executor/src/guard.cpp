#include "stretch_guarded_executor/guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
namespace stretch_guarded_executor {
namespace {
bool sameVector(const Vector& a, const Vector& b, double tolerance = 1e-12) {
  return (a - b).cwiseAbs().maxCoeff() <= tolerance;
}

bool strictlyIncreasingTrajectory(const std::vector<Vector>& trajectory,
                                  const std::vector<double>& times) {
  if (trajectory.size() < 2 || trajectory.size() != times.size())
    return false;
  if (!std::isfinite(times.front()) || std::abs(times.front()) > 1e-12)
    return false;
  double previous = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < trajectory.size(); ++i) {
    if (!trajectory[i].allFinite() || !std::isfinite(times[i]) ||
        times[i] <= previous)
      return false;
    previous = times[i];
  }
  return true;
}

void fillTrajectoryPoint(trajectory_msgs::msg::JointTrajectoryPoint& point,
                         const Vector& pos, const Vector& vel,
                         const Vector& acc, bool lift_only) {
  if (lift_only) {
    point.positions = {pos[0]};
    point.velocities = {vel[0]};
    point.accelerations = {acc[0]};
    return;
  }

  // Map logical 5-DoF to physical 8 joints.
  // arm_extension (index 1) distributes equally to 4 telescope stages.
  const double arm_pos = pos[1] / 4.0;
  const double arm_vel = vel[1] / 4.0;
  const double arm_acc = acc[1] / 4.0;
  point.positions = {pos[0], arm_pos, arm_pos, arm_pos, arm_pos,
                     pos[2], pos[3], pos[4]};
  point.velocities = {vel[0], arm_vel, arm_vel, arm_vel, arm_vel,
                      vel[2], vel[3], vel[4]};
  point.accelerations = {acc[0], arm_acc, arm_acc, arm_acc, arm_acc,
                         acc[2], acc[3], acc[4]};
}
}  // namespace

const char* stateName(State s) {
  switch (s) {
    case State::DISARMED:
      return "DISARMED";
    case State::ARMED:
      return "ARMED";
    case State::READY:
      return "READY";
    case State::EXECUTING:
      return "EXECUTING";
    case State::COMPLETE:
      return "COMPLETE";
    case State::ABORTED:
      return "ABORTED";
  }
  return "INVALID";
}
void Config::validate() const {
  const Vector delta_cap = (Vector() << .3, .3, 1.5, 1.5, 1.5).finished();
  const Vector drift_cap =
      (Vector() << .001, .001, .005, .005, .005).finished();
  // Same caps for simulation and the real robot (targets 0.10 m/s, 0.6 rad/s).
  const Vector velocity_cap = (Vector() << .12, .12, .8, .8, .8).finished();
  const Vector accel_cap = (Vector() << .2, .2, 2., 2., 2.).finished();
  // Raised 2026-11-06 for testing an uncalibrated robot (was 3 cm / 0.15 rad).
  const Vector tracking_error_cap =
      (Vector() << .06, .06, .30, .30, .30).finished();
  // Wrist servos settle ~1 deg short under the gripper load (measured 0.015 rad).
  const Vector goal_tolerance_cap =
      (Vector() << .003, .003, .035, .035, .035).finished();
  if ((start_drift.array() > drift_cap.array()).any() ||
      !max_delta.allFinite() || !tracking_error.allFinite() ||
      !start_drift.allFinite() || !max_velocity.allFinite() ||
      !max_acceleration.allFinite() || !goal_tolerance.allFinite() ||
      (max_delta.array() <= 0).any() ||
      (max_delta.array() > delta_cap.array()).any() ||
      (tracking_error.array() <= 0).any() ||
      (tracking_error.array() > tracking_error_cap.array()).any() ||
      (start_drift.array() <= 0).any() ||
      (start_drift.array() > tracking_error.array()).any() ||
      (max_velocity.array() <= 0).any() ||
      (max_velocity.array() > velocity_cap.array()).any() ||
      (max_acceleration.array() <= 0).any() ||
      (max_acceleration.array() > accel_cap.array()).any() ||
      (goal_tolerance.array() <= 0).any() ||
      (goal_tolerance.array() > goal_tolerance_cap.array()).any() ||
      (goal_tolerance.array() > tracking_error.array()).any())
    throw std::invalid_argument("Invalid/excessive guard envelope");
  for (double v : {min_duration, max_duration, approval_timeout,
                   execution_slack, minimum_motion})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Guard values must be finite and positive");
  if (min_duration < .3 || max_duration > 60 || min_duration > max_duration ||
      approval_timeout > 30 || execution_slack > 3 ||
      minimum_motion > max_delta[0])
    throw std::invalid_argument("Guard hard safety cap exceeded");
}
Guard::Guard(Config c, Bounds b) : config_(c), bounds_(b) {
  c.validate();
  if (!b.lower.allFinite() || !b.upper.allFinite() ||
      (b.lower.array() >= b.upper.array()).any())
    throw std::invalid_argument("Invalid bounds");
}
Check Guard::arm() {
  if (!config_.execution_enabled)
    return {false, "execution explicitly disabled"};
  if (state_ == State::EXECUTING) return {false, "motion already executing"};
  if (armed_) return {false, "already armed; disarm to reset"};
  armed_ = true;
  state_ = State::ARMED;
  plan_.reset();
  reason_ = "operator armed";
  return {true, reason_};
}
void Guard::disarm() {
  if (state_ == State::EXECUTING) {
    abort("operator disarm during execution");
    return;
  }
  armed_ = false;
  state_ = State::DISARMED;
  plan_.reset();
  reason_ = "operator disarmed";
}
void Guard::completeDryRun() {
  if (state_ != State::EXECUTING)
    throw std::logic_error("No dry-run plan executing");
  armed_ = false;
  state_ = State::COMPLETE;
  reason_ =
      "DRY RUN complete: no goal sent, physical tracking not exercised; "
      "disarmed";
}
void Guard::abort(const std::string& r) {
  armed_ = false;
  state_ = State::ABORTED;
  reason_ = r;
}
void Guard::invalidate(const std::string& r) {
  if (state_ == State::READY) {
    state_ = State::ARMED;
    plan_.reset();
    reason_ = r;
  }
}
Check Guard::prerequisites(const Input& in) const {
  if (!config_.execution_enabled)
    return {false, "execution explicitly disabled"};
  if (!armed_) return {false, "executor disarmed"};
  if (!in.fresh) return {false, "robot state/status stale"};
  if (!in.required_joints || !in.q.allFinite())
    return {false, "required manipulation joints absent/invalid"};
  if (!in.homed) return {false, "robot unhomed"};
  if (in.runstopped) return {false, "runstop active"};
  // Check all active joints against bounds.
  if (config_.lift_only) {
    if (in.q[0] < bounds_.lower[0] || in.q[0] > bounds_.upper[0])
      return {false, "measured lift outside hardware/model limits"};
  } else {
    for (int i = 0; i < 5; ++i)
      if (in.q[i] < bounds_.lower[i] - 1e-8 ||
          in.q[i] > bounds_.upper[i] + 1e-8)
        return {false, "measured joint " + std::to_string(i) +
                           " outside hardware/model limits"};
  }
  if (!in.mode_fresh || !in.trajectory_mode)
    return {false,
            "fresh trajectory mode required; position mode ignores timing"};
  if (!in.streaming_known || in.streaming)
    return {false, "streaming position unknown/active"};
  if (!in.diagnostics_fresh)
    return {false, "diagnostics stale (no /diagnostics for 2.5 s)"};
  if (!in.diagnostics_ok)
    return {false, "diagnostics ERROR/STALE present"};
  if (!in.startpoint_guard_verified)
    return {false, "driver initial-point guard not verified true"};
  if (!in.action_server_available)
    return {false, "trajectory action server unavailable"};
  if (!in.base_still) return {false, "mobile base moving or odometry stale"};
  return {true, "prerequisites valid"};
}
Check Guard::target(const Input& in, const Candidate& c, double duration,
                    double now) const {
  auto check = prerequisites(in);
  if (!check.ok) return check;
  if (in.active_goal_present || !in.controller_idle)
    return {false, "controller busy: active or unresolved local goal"};
  if (!c.valid || !std::isfinite(now) || !std::isfinite(c.received) ||
      now < c.received || now - c.received > config_.approval_timeout)
    return {false, "candidate invalid/stale"};
  if (!std::isfinite(duration) || duration < config_.min_duration ||
      duration > config_.max_duration)
    return {false, "invalid trajectory duration"};
  Vector q = c.q;
  if (c.relative_lift) {
    if (!std::isfinite(*c.relative_lift))
      return {false, "invalid relative lift"};
    q = in.q;
    q[0] += *c.relative_lift;
  }
  if (!q.allFinite() || !c.seed.allFinite())
    return {false, "target or seed contains non-finite values"};
  // Check target against bounds for all active joints.
  if (config_.lift_only) {
    if (q[0] < bounds_.lower[0] || q[0] > bounds_.upper[0])
      return {false, "lift target outside hardware/model limits"};
  } else {
    for (int i = 0; i < 5; ++i)
      if (q[i] < bounds_.lower[i] - 1e-8 || q[i] > bounds_.upper[i] + 1e-8)
        return {false, "joint " + std::to_string(i) +
                           " target outside hardware/model limits"};
  }
  Vector delta = q - in.q;
  if ((delta.cwiseAbs().array() > config_.max_delta.array() + 1e-12).any())
    return {false, "per-joint delta envelope exceeded"};
  if (config_.lift_only) {
    // Lift-only milestone: reject arm/wrist target changes.
    if (!c.relative_lift) {
      for (int i = 1; i < 5; ++i)
        if (std::abs(q[i] - c.seed[i]) > 1e-8)
          return {false,
                  "lift-only milestone: arm/wrist target change rejected"};
      if (std::abs(in.q[0] - c.seed[0]) > config_.start_drift[0])
        return {false, "current lift moved from preview seed"};
    }
    if (std::abs(delta[0]) < config_.minimum_motion)
      return {false, "no nonzero lift motion"};
  } else {
    // 5-DoF: verify seed drift for all joints.
    for (int i = 0; i < 5; ++i)
      if (std::abs(in.q[i] - c.seed[i]) > config_.start_drift[i])
        return {false, "joint " + std::to_string(i) +
                           " moved from preview seed"};
    // At least one joint must move.
    if (delta.cwiseAbs().maxCoeff() < config_.minimum_motion)
      return {false, "no nonzero joint motion"};
  }
  // Per-joint velocity and acceleration check using quintic blend extrema:
  // max s' = 1.875, max |s''| = 10/sqrt(3).
  for (int i = 0; i < 5; ++i) {
    if (std::abs(delta[i]) < 1e-15) continue;
    double peak_vel = 1.875 * std::abs(delta[i]) / duration;
    double peak_accel =
        (10 / std::sqrt(3.)) * std::abs(delta[i]) / (duration * duration);
    if (peak_vel > config_.max_velocity[i] + 1e-12)
      return {false, "velocity limit exceeded on joint " + std::to_string(i)};
    if (peak_accel > config_.max_acceleration[i] + 1e-12)
      return {false,
              "acceleration limit exceeded on joint " + std::to_string(i)};
  }
  return {true, "target approved"};
}
Check Guard::validate(const Input& in, const Candidate& c, double duration,
                      double now) {
  if (state_ != State::ARMED && state_ != State::READY)
    return {false, "must be ARMED before validation"};
  invalidate("revalidating");
  auto check = target(in, c, duration, now);
  if (!check.ok) {
    reason_ = check.reason;
    return check;
  }
  Plan p;
  p.start = in.q;
  p.target = c.q;
  p.duration = duration;
  p.approved_at = now;
  p.revision = c.revision;
  p.relative_lift = c.relative_lift;
  p.candidate_q = c.q;
  p.candidate_seed = c.seed;
  if (c.relative_lift) {
    p.target = in.q;
    p.target[0] += *c.relative_lift;
  }
  if (config_.lift_only) {
    // Passive axes remain measured; the outgoing message contains joint_lift
    // only.
    for (int i = 1; i < 5; ++i) p.target[i] = p.start[i];
  }
  plan_ = p;
  state_ = State::READY;
  reason_ = config_.lift_only ? "approved immutable lift-only plan"
                              : "approved immutable 5-DoF plan";
  return {true, reason_};
}
Check Guard::trajectoryTarget(
    const Input& in, const Candidate& c, const std::vector<Vector>& trajectory,
    const std::vector<double>& times_from_start, double now) const {
  auto check = prerequisites(in);
  if (!check.ok) return check;
  if (in.active_goal_present || !in.controller_idle)
    return {false, "controller busy: active or unresolved local goal"};
  if (config_.lift_only)
    return {false, "collision-planned trajectory requires 5-DoF execution"};
  if (!c.valid || !std::isfinite(now) || !std::isfinite(c.received) ||
      now < c.received || now - c.received > config_.approval_timeout)
    return {false, "candidate invalid/stale"};
  if (!c.q.allFinite() || !c.seed.allFinite())
    return {false, "target or seed contains non-finite values"};
  if (!strictlyIncreasingTrajectory(trajectory, times_from_start))
    return {false, "invalid timed trajectory"};
  const double duration = times_from_start.back();
  if (duration < config_.min_duration || duration > config_.max_duration)
    return {false, "invalid timed trajectory duration"};
  if (!sameVector(trajectory.front(), in.q, config_.start_drift.maxCoeff()))
    return {false, "trajectory start does not match measured state"};
  if (!sameVector(trajectory.back(), c.q, 1e-9))
    return {false, "trajectory endpoint does not match candidate target"};
  for (int i = 0; i < 5; ++i)
    if (std::abs(in.q[i] - c.seed[i]) > config_.start_drift[i])
      return {false, "joint " + std::to_string(i) +
                         " moved from preview seed"};
  if ((c.q - in.q).cwiseAbs().maxCoeff() < config_.minimum_motion)
    return {false, "no nonzero joint motion"};

  for (size_t index = 0; index < trajectory.size(); ++index) {
    const auto& q = trajectory[index];
    for (int axis = 0; axis < 5; ++axis) {
      if (q[axis] < bounds_.lower[axis] - 1e-8 ||
          q[axis] > bounds_.upper[axis] + 1e-8) {
        return {false, "trajectory point " + std::to_string(index) +
                           " joint " + std::to_string(axis) +
                           " outside hardware/model limits"};
      }
    }
  }

  // Average velocity per sample and its change per sample: for a sampled
  // smooth profile both are bounded by the profile's peaks (mean value).
  Vector previous_velocity = Vector::Zero();
  double previous_dt = 0.0;
  for (size_t index = 1; index < trajectory.size(); ++index) {
    const Vector delta = trajectory[index] - trajectory[index - 1];
    const double dt = times_from_start[index] - times_from_start[index - 1];
    if (!std::isfinite(dt) || dt <= 0.0)
      return {false, "invalid timed trajectory segment"};
    if ((delta.cwiseAbs().array() > config_.max_delta.array() + 1e-12).any())
      return {false, "trajectory segment delta envelope exceeded"};
    const Vector velocity = delta / dt;
    const Vector acceleration =
        (velocity - previous_velocity) / (0.5 * (dt + previous_dt));
    for (int axis = 0; axis < 5; ++axis) {
      if (std::abs(velocity[axis]) > config_.max_velocity[axis] * (1 + 1e-9))
        return {false, "trajectory velocity limit exceeded on joint " +
                           std::to_string(axis)};
      if (std::abs(acceleration[axis]) >
          config_.max_acceleration[axis] * (1 + 1e-6))
        return {false, "trajectory acceleration limit exceeded on joint " +
                           std::to_string(axis)};
    }
    previous_velocity = velocity;
    previous_dt = dt;
  }
  return {true, "timed trajectory approved"};
}

Check Guard::validateTrajectory(
    const Input& in, const Candidate& c, const std::vector<Vector>& trajectory,
    const std::vector<double>& times_from_start, double now) {
  if (state_ != State::ARMED && state_ != State::READY)
    return {false, "must be ARMED before validation"};
  invalidate("revalidating");
  auto check = trajectoryTarget(in, c, trajectory, times_from_start, now);
  if (!check.ok) {
    reason_ = check.reason;
    return check;
  }
  Plan p;
  p.start = trajectory.front();
  p.target = trajectory.back();
  p.duration = times_from_start.back();
  p.approved_at = now;
  p.revision = c.revision;
  p.candidate_q = c.q;
  p.candidate_seed = c.seed;
  p.relative_lift = c.relative_lift;
  p.trajectory = trajectory;
  p.times_from_start = times_from_start;
  plan_ = p;
  state_ = State::READY;
  std::ostringstream message;
  message << "approved collision-planned 5-DoF trajectory with "
          << trajectory.size() << " samples";
  reason_ = message.str();
  return {true, reason_};
}
Check Guard::begin(const Input& in, const Candidate& c, double now) {
  if (state_ != State::READY || !plan_) return {false, "no READY approval"};
  if (!std::isfinite(now) || now < plan_->approved_at ||
      now - plan_->approved_at > config_.approval_timeout ||
      c.revision != plan_->revision ||
      c.relative_lift != plan_->relative_lift ||
      !c.q.isApprox(plan_->candidate_q, 1e-12) ||
      !c.seed.isApprox(plan_->candidate_seed, 1e-12)) {
    invalidate("approval expired or candidate changed");
    return {false, reason_};
  }
  // Check start drift on active joints.
  if (config_.lift_only) {
    if (std::abs(in.q[0] - plan_->start[0]) > config_.start_drift[0]) {
      abort("lift moved after approval");
      return {false, reason_};
    }
  } else {
    for (int i = 0; i < 5; ++i)
      if (std::abs(in.q[i] - plan_->start[i]) > config_.start_drift[i]) {
        abort("joint " + std::to_string(i) + " moved after approval");
        return {false, reason_};
      }
  }
  auto check = plan_->trajectory.empty()
                   ? target(in, c, plan_->duration, now)
                   : trajectoryTarget(in, c, plan_->trajectory,
                                      plan_->times_from_start, now);
  if (!check.ok) {
    abort(check.reason);
    return check;
  }
  if (plan_->trajectory.empty()) {
    plan_->start = in.q;
    if (plan_->relative_lift)
      plan_->target[0] = in.q[0] + *plan_->relative_lift;
    if (config_.lift_only) {
      for (int i = 1; i < 5; ++i) plan_->target[i] = in.q[i];
    }
  }
  plan_->started_at = now;
  state_ = State::EXECUTING;
  reason_ = "explicit execute accepted";
  return {true, reason_};
}
Check Guard::monitorPrerequisites(const Input& in) {
  if (state_ != State::EXECUTING || !plan_) return {false, "not executing"};
  auto check = prerequisites(in);
  if (!check.ok) abort(check.reason);
  return check;
}
Check Guard::synchronizeStart(const Input& in, double now) {
  auto check = monitorPrerequisites(in);
  if (!check.ok) return check;
  if (!std::isfinite(now) || now < plan_->started_at) {
    abort("controller feedback clock failure");
    return {false, reason_};
  }
  // The first action feedback is the driver's evidence that the hardware
  // trajectory has started.  It can arrive after goal acceptance, so anchor
  // the tracking reference here while requiring the measured robot to remain
  // inside the original tracking envelope.
  if ((config_.lift_only && std::abs(in.q[0] - plan_->start[0]) > config_.tracking_error[0]) ||
      (!config_.lift_only && ((in.q - plan_->start).cwiseAbs().array() > config_.tracking_error.array()).any())) {
    abort("measured lift moved before first controller feedback");
    return {false, reason_};
  }
  plan_->started_at = now;
  return {true, "tracking clock synchronized to first controller feedback"};
}
Vector Guard::reference(const Plan& p, double elapsed) {
  if (!p.trajectory.empty()) {
    if (!strictlyIncreasingTrajectory(p.trajectory, p.times_from_start))
      throw std::invalid_argument("Invalid timed trajectory");
    if (elapsed <= 0.0) return p.trajectory.front();
    if (elapsed >= p.times_from_start.back()) return p.trajectory.back();
    auto upper =
        std::upper_bound(p.times_from_start.begin(), p.times_from_start.end(),
                         elapsed);
    const size_t index =
        static_cast<size_t>(std::distance(p.times_from_start.begin(), upper));
    const size_t right = std::min(index, p.trajectory.size() - 1);
    const size_t left = right - 1;
    const double t0 = p.times_from_start[left];
    const double t1 = p.times_from_start[right];
    const double u = (elapsed - t0) / (t1 - t0);
    return (1.0 - u) * p.trajectory[left] + u * p.trajectory[right];
  }
  double u = std::clamp(elapsed / p.duration, 0., 1.);
  double s = u * u * u * (10 + u * (-15 + 6 * u));
  return p.start + s * (p.target - p.start);
}
std::vector<std::string> Guard::trajectoryJointNames(bool lift_only) {
  if (lift_only)
    return {"joint_lift"};
  return {"joint_lift",        "joint_arm_l3",     "joint_arm_l2",
          "joint_arm_l1",      "joint_arm_l0",     "joint_wrist_yaw",
          "joint_wrist_pitch", "joint_wrist_roll"};
}
trajectory_msgs::msg::JointTrajectory Guard::trajectory(const Plan& p,
                                                        bool lift_only) {
  if (!p.trajectory.empty()) {
    if (!strictlyIncreasingTrajectory(p.trajectory, p.times_from_start) ||
        !std::isfinite(p.duration) || p.duration <= 0 || p.duration > 60 ||
        std::abs(p.duration - p.times_from_start.back()) > 1e-9)
      throw std::invalid_argument("Invalid timed trajectory plan");
    if (lift_only)
      throw std::invalid_argument("Only 5-DoF timed trajectories are allowed");

    trajectory_msgs::msg::JointTrajectory t;
    t.joint_names = trajectoryJointNames(false);
    for (size_t i = 0; i < p.trajectory.size(); ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      Vector velocity = Vector::Zero();
      Vector acceleration = Vector::Zero();
      if (i > 0 && i + 1 < p.trajectory.size())
        velocity = (p.trajectory[i + 1] - p.trajectory[i - 1]) /
                   (p.times_from_start[i + 1] - p.times_from_start[i - 1]);
      fillTrajectoryPoint(point, p.trajectory[i], velocity, acceleration,
                          false);
      const int64_t ns =
          std::llround(p.times_from_start[i] * 1000000000.0);
      point.time_from_start.sec = ns / 1000000000;
      point.time_from_start.nanosec = ns % 1000000000;
      t.points.push_back(point);
    }
    return t;
  }
  if (!p.start.allFinite() || !p.target.allFinite() ||
      !std::isfinite(p.duration) || p.duration <= 0 || p.duration > 60)
    throw std::invalid_argument("Invalid plan");
  if (lift_only) {
    if (std::abs(p.target[0] - p.start[0]) > .030000000001)
      throw std::invalid_argument("Lift-only delta exceeded");
    for (int i = 1; i < 5; ++i)
      if (p.target[i] != p.start[i])
        throw std::invalid_argument("Only lift trajectories are allowed");
  }
  const auto names = trajectoryJointNames(lift_only);
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = names;
  // Header stamp zero: required by Stretch trajectory mode.
  int steps = static_cast<int>(std::ceil(p.duration / .1));
  Vector d = p.target - p.start;
  for (int k = 0; k <= steps; ++k) {
    double u = double(k) / steps;
    trajectory_msgs::msg::JointTrajectoryPoint v;
    Vector pos = reference(p, u * p.duration);
    // Quintic velocity: s'(u) = 30*u^2*(1-u)^2
    // Quintic acceleration: s''(u) = 60*u*(1-u)*(1-2*u)
    Vector vel = d * 30 * u * u * (1 - u) * (1 - u) / p.duration;
    Vector acc = d * 60 * u * (1 - u) * (1 - 2 * u) / (p.duration * p.duration);
    fillTrajectoryPoint(v, pos, vel, acc, lift_only);
    int64_t ns = std::llround(u * p.duration * 1e9);
    v.time_from_start.sec = ns / 1000000000;
    v.time_from_start.nanosec = ns % 1000000000;
    t.points.push_back(v);
  }
  return t;
}
Check Guard::monitor(const Input& in, double now) {
  if (state_ != State::EXECUTING || !plan_) return {false, "not executing"};
  auto check = prerequisites(in);
  if (!check.ok) {
    abort(check.reason);
    return check;
  }
  double elapsed = now - plan_->started_at;
  if (!std::isfinite(elapsed) || elapsed < 0 ||
      elapsed > plan_->duration + config_.execution_slack) {
    abort("execution timeout/clock failure");
    return {false, reason_};
  }
  Vector ref = reference(*plan_, elapsed);
  if (config_.lift_only) {
    // Only track lift joint.
    const double error = std::abs(in.q[0] - ref[0]);
    if (error > config_.tracking_error[0]) {
      std::ostringstream reason;
      reason << "tracking error exceeded on joint 0 (lift): error=" << error
             << " limit=" << config_.tracking_error[0];
      abort(reason.str());
      return {false, reason_};
    }
  } else {
    // Track all active (commanded) joints.
    static constexpr const char* kAxisNames[] = {
        "lift", "arm_extension", "wrist_yaw", "wrist_pitch", "wrist_roll"};
    for (int i = 0; i < 5; ++i) {
      const double error = std::abs(in.q[i] - ref[i]);
      if (error > config_.tracking_error[i]) {
        std::ostringstream reason;
        reason << "tracking error exceeded on joint " << i << " ("
               << kAxisNames[i] << "): error=" << error
               << " limit=" << config_.tracking_error[i];
        abort(reason.str());
        return {false, reason_};
      }
    }
  }
  return {true, "execution monitored"};
}
Check Guard::finish(const Input& in, bool success, double now) {
  auto check = monitor(in, now);
  if (!check.ok) return check;
  if (!success) {
    abort("controller failure");
    return {false, reason_};
  }
  const Vector endpoint_error = (in.q - plan_->target).cwiseAbs();
  int worst_axis = 0;
  double worst_ratio = endpoint_error[0] / config_.goal_tolerance[0];
  if (!config_.lift_only) {
    for (int axis = 1; axis < 5; ++axis) {
      const double ratio = endpoint_error[axis] / config_.goal_tolerance[axis];
      if (ratio > worst_ratio) {
        worst_ratio = ratio;
        worst_axis = axis;
      }
    }
  }
  if (worst_ratio > 1.0) {
    static constexpr const char* kAxisNames[] = {
        "lift", "arm_extension", "wrist_yaw", "wrist_pitch", "wrist_roll"};
    std::ostringstream reason;
    reason << "measured goal error on " << kAxisNames[worst_axis]
           << ": error=" << endpoint_error[worst_axis]
           << " limit=" << config_.goal_tolerance[worst_axis];
    abort(reason.str());
    return {false, reason_};
  }
  armed_ = false;
  state_ = State::COMPLETE;
  reason_ =
      "controller success and measured endpoint verified; automatically "
      "disarmed";
  return {true, reason_};
}
}  // namespace stretch_guarded_executor
