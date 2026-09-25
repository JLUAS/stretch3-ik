#include <gtest/gtest.h>

#include <limits>

#include "stretch_guarded_executor/controller_state.hpp"
#include "stretch_guarded_executor/diagnostic_policy.hpp"
#include "stretch_guarded_executor/guard.hpp"
using namespace stretch_guarded_executor;
class GuardTest : public ::testing::Test {
 protected:
  Config c;
  Bounds b;
  Input in;
  Candidate candidate;
  void SetUp() override {
    c.execution_enabled = true;
    b.lower << 0, 0, -1.26, -1.57, -2.91;
    b.upper << 1.099, .516, 4, .45, 2.917;
    in.q << .5, .2, 0, 0, 0;
    in.fresh = in.homed = in.required_joints = in.mode_fresh =
        in.trajectory_mode = true;
    in.action_server_available = true;
    in.streaming_known = in.controller_idle = in.diagnostics_fresh =
        in.diagnostics_ok = in.startpoint_guard_verified = true;
    in.runstopped = false;
    candidate.q = candidate.seed = in.q;
    candidate.q[0] += .01;
    candidate.valid = true;
    candidate.received = 10;
    candidate.revision = 1;
  }
  Guard armed() {
    Guard g(c, b);
    EXPECT_TRUE(g.arm().ok);
    return g;
  }
  Guard running() {
    auto g = armed();
    EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
    EXPECT_TRUE(g.begin(in, candidate, 10).ok);
    return g;
  }
};
TEST_F(GuardTest, StartupAlwaysDisarmed) {
  Guard g(c, b);
  EXPECT_EQ(g.state(), State::DISARMED);
  EXPECT_FALSE(g.armed());
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
}
TEST_F(GuardTest, FiveAxisCompletionChecksEveryEndpoint) {
  c.lift_only = false;
  for (int axis = 1; axis < 5; ++axis) {
    auto g = running();
    in.q = candidate.q;
    // Stay inside the runtime tracking envelope but outside endpoint tolerance.
    in.q[axis] += c.goal_tolerance[axis] +
                  .5 * (c.tracking_error[axis] - c.goal_tolerance[axis]);
    EXPECT_FALSE(g.finish(in, true, 15).ok) << axis;
    in.q = candidate.seed;
  }
}
TEST_F(GuardTest, FiveAxisTracksHoldingJoints) {
  c.lift_only = false;
  auto g = running();
  in.q[2] += c.tracking_error[2] + .001;
  EXPECT_FALSE(g.monitor(in, 10).ok);
}
TEST_F(GuardTest, FiveAxisFirstFeedbackChecksEveryJoint) {
  c.lift_only = false;
  auto g = running();
  in.q[1] += c.tracking_error[1] + .001;
  EXPECT_FALSE(g.synchronizeStart(in, 10).ok);
}
TEST_F(GuardTest, DisabledCannotArm) {
  c.execution_enabled = false;
  Guard g(c, b);
  EXPECT_FALSE(g.arm().ok);
}
TEST_F(GuardTest, ExplicitSequence) {
  auto g = armed();
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  EXPECT_EQ(g.state(), State::READY);
  ASSERT_TRUE(g.begin(in, candidate, 10).ok);
  EXPECT_EQ(g.state(), State::EXECUTING);
  EXPECT_FALSE(g.arm().ok);
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, CompletionDisarms) {
  auto g = running();
  in.q = candidate.q;
  ASSERT_TRUE(g.finish(in, true, 15).ok);
  EXPECT_EQ(g.state(), State::COMPLETE);
  EXPECT_FALSE(g.armed());
  EXPECT_FALSE(g.begin(in, candidate, 15).ok);
  EXPECT_TRUE(g.arm().ok);
  EXPECT_EQ(g.state(), State::ARMED);
}
TEST_F(GuardTest, FailureDisarms) {
  auto g = running();
  ASSERT_FALSE(g.finish(in, false, 10).ok);
  EXPECT_EQ(g.state(), State::ABORTED);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, StopIndependentOfCandidate) {
  auto g = running();
  candidate.valid = false;
  g.abort("operator STOP");
  EXPECT_FALSE(g.armed());
  EXPECT_EQ(g.state(), State::ABORTED);
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
}
TEST_F(GuardTest, DisarmDuringMotionAborts) {
  auto g = running();
  g.disarm();
  EXPECT_EQ(g.state(), State::ABORTED);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, DisarmClearsApproval) {
  auto g = armed();
  g.validate(in, candidate, 5, 10);
  g.disarm();
  EXPECT_EQ(g.state(), State::DISARMED);
  EXPECT_FALSE(g.plan());
}
TEST_F(GuardTest, DryRunTerminalDisarms) {
  auto g = running();
  g.completeDryRun();
  EXPECT_EQ(g.state(), State::COMPLETE);
  EXPECT_FALSE(g.armed());
  EXPECT_THROW(g.completeDryRun(), std::logic_error);
}
TEST_F(GuardTest, RejectEveryMissingPrerequisite) {
  for (bool Input::* member :
       {&Input::fresh, &Input::homed, &Input::required_joints,
        &Input::mode_fresh, &Input::trajectory_mode, &Input::streaming_known,
        &Input::controller_idle, &Input::diagnostics_fresh,
        &Input::diagnostics_ok, &Input::startpoint_guard_verified}) {
    auto bad = in;
    bad.*member = false;
    auto g = armed();
    EXPECT_FALSE(g.validate(bad, candidate, 5, 10).ok);
  }
  auto g = armed();
  in.runstopped = true;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  in.runstopped = false;
  in.streaming = true;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, InvalidCandidate) {
  auto g = armed();
  candidate.valid = false;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  candidate.valid = true;
  candidate.q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, StaleFutureCandidate) {
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 26).ok);
  EXPECT_FALSE(g.validate(in, candidate, 5, 9).ok);
  EXPECT_FALSE(
      g.validate(in, candidate, 5, std::numeric_limits<double>::quiet_NaN())
          .ok);
}
TEST_F(GuardTest, HardwareLimitsEachJoint) {
  for (int i = 0; i < 5; i++) {
    auto g = armed();
    auto bad = candidate;
    bad.q[i] = b.upper[i] + .001;
    EXPECT_FALSE(g.validate(in, bad, 5, 10).ok);
    bad.q[i] = b.lower[i] - .001;
    EXPECT_FALSE(g.validate(in, bad, 5, 10).ok);
  }
}
TEST_F(GuardTest, MeasuredStateLimits) {
  auto g = armed();
  in.q[0] = -.01;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  in.q[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, PerJointDelta) {
  for (int i = 0; i < 5; i++) {
    auto g = armed();
    auto bad = candidate;
    bad.q[i] = in.q[i] + c.max_delta[i] + .0001;
    EXPECT_FALSE(g.validate(in, bad, 5, 10).ok);
  }
}
TEST_F(GuardTest, PassiveAxisMotionAlwaysRejected) {
  for (int i = 1; i < 5; i++) {
    auto g = armed();
    auto bad = candidate;
    bad.q[i] += .00001;
    EXPECT_FALSE(g.validate(in, bad, 5, 10).ok);
  }
}
TEST_F(GuardTest, PreviewSeedDrift) {
  auto g = armed();
  candidate.seed[0] -= .002;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DurationAndSpeed) {
  auto g = armed();
  for (double t : {0., 2.9, 10.1, std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(g.validate(in, candidate, t, 10).ok);
  EXPECT_TRUE(g.validate(in, candidate, 3, 10).ok);
  candidate.q[0] = in.q[0] + c.max_delta[0];
  EXPECT_FALSE(g.validate(in, candidate, 2.9, 10).ok);
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, NoOpRejected) {
  auto g = armed();
  candidate.q = in.q;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, ApprovalExpires) {
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  candidate.received = 26;
  EXPECT_FALSE(g.begin(in, candidate, 26).ok);
  EXPECT_EQ(g.state(), State::ARMED);
}
TEST_F(GuardTest, RevisionInvalidatesApproval) {
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  candidate.revision++;
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
}
TEST_F(GuardTest, SameRevisionCannotChangeTarget) {
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  candidate.q[0] -= .001;
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
}
TEST_F(GuardTest, ChangedStateBeforeExecute) {
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  in.q[0] += .002;
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, ExecuteRechecksRunstop) {
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  in.runstopped = true;
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, EveryRuntimePrerequisiteAborts) {
  for (bool Input::* member :
       {&Input::fresh, &Input::homed, &Input::required_joints,
        &Input::mode_fresh, &Input::trajectory_mode, &Input::streaming_known,
        &Input::diagnostics_fresh, &Input::diagnostics_ok,
        &Input::startpoint_guard_verified}) {
    auto g = running();
    auto bad = in;
    bad.*member = false;
    EXPECT_FALSE(g.monitor(bad, 10).ok);
    EXPECT_FALSE(g.armed());
    EXPECT_EQ(g.state(), State::ABORTED);
  }
}
TEST_F(GuardTest, RuntimeRunstop) {
  auto g = running();
  in.runstopped = true;
  EXPECT_FALSE(g.monitor(in, 10).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, RuntimeStreaming) {
  auto g = running();
  in.streaming = true;
  EXPECT_FALSE(g.monitor(in, 10).ok);
}
TEST_F(GuardTest, LiftOnlyTrackingIgnoresPassiveJointObservation) {
  auto g = running();
  auto passive = in;
  for (int i = 1; i < 5; ++i) passive.q[i] += .5;
  EXPECT_TRUE(g.monitor(passive, 10).ok);

  auto tracking = running();
  auto bad = in;
  bad.q[0] += c.tracking_error[0] + .0001;
  EXPECT_FALSE(tracking.monitor(bad, 10).ok);
  EXPECT_FALSE(tracking.armed());
}
TEST_F(GuardTest, RuntimePrerequisitesCanBeCheckedBeforeTrackingStarts) {
  auto g = running();
  EXPECT_TRUE(g.monitorPrerequisites(in).ok);
  in.runstopped = true;
  EXPECT_FALSE(g.monitorPrerequisites(in).ok);
  EXPECT_EQ(g.state(), State::ABORTED);
}
TEST_F(GuardTest, FirstFeedbackSynchronizesTrackingClock) {
  auto g = running();
  ASSERT_TRUE(g.synchronizeStart(in, 10.66).ok);
  EXPECT_DOUBLE_EQ(g.plan()->started_at, 10.66);
  EXPECT_TRUE(g.monitor(in, 10.66).ok);
}
TEST_F(GuardTest, FirstFeedbackRejectsMotionOutsideTrackingEnvelope) {
  auto g = running();
  in.q[0] += c.tracking_error[0] + .0001;
  EXPECT_FALSE(g.synchronizeStart(in, 10.66).ok);
  EXPECT_EQ(g.state(), State::ABORTED);
}
TEST_F(GuardTest, Timeout) {
  auto g = running();
  in.q = candidate.q;
  EXPECT_FALSE(g.monitor(in, 17.01).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, ControllerFailure) {
  auto g = running();
  in.q = candidate.q;
  EXPECT_FALSE(g.finish(in, false, 15).ok);
  EXPECT_EQ(g.reason(), "controller failure");
}
TEST_F(GuardTest, PrematureResult) {
  auto g = running();
  EXPECT_FALSE(g.finish(in, true, 10).ok);
  EXPECT_NE(g.reason().find("measured goal error on lift"), std::string::npos);
}
TEST_F(GuardTest, SuccessfulMeasuredEndpointIsNotRejectedForFeedbackLatency) {
  auto g = running();
  ASSERT_TRUE(g.synchronizeStart(in, 10.66).ok);
  in.q = candidate.q;
  ASSERT_TRUE(g.finish(in, true, 15).ok);
  EXPECT_EQ(g.state(), State::COMPLETE);
}
TEST_F(GuardTest, EndpointError) {
  auto g = running();
  in.q = candidate.q;
  in.q[0] -= .00275;
  EXPECT_FALSE(g.finish(in, true, 15).ok);
  EXPECT_NE(g.reason().find("measured goal error on lift"), std::string::npos);
  EXPECT_NE(g.reason().find("limit=0.0025"), std::string::npos);
}
TEST_F(GuardTest, RelativeLiftRebasesWithinDrift) {
  auto g = armed();
  candidate.relative_lift = .01;
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  in.q[0] -= .00001;
  ASSERT_TRUE(g.begin(in, candidate, 10).ok);
  EXPECT_NEAR(g.plan()->target[0] - g.plan()->start[0], .01, 1e-12);
  EXPECT_DOUBLE_EQ(g.plan()->start[0], in.q[0]);
}
TEST_F(GuardTest, RelativeLiftRespectsUpperLimit) {
  auto g = armed();
  in.q[0] = b.upper[0] - .005;
  candidate.relative_lift = .01;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, RelativeLiftCannotChangeAfterApproval) {
  auto g = armed();
  candidate.relative_lift = .01;
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  candidate.relative_lift = .009;
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
}
TEST_F(GuardTest, SmoothLiftOnlyTrajectory) {
  auto g = running();
  auto t = Guard::trajectory(*g.plan(), true);
  ASSERT_EQ(t.joint_names, std::vector<std::string>{"joint_lift"});
  EXPECT_EQ(t.header.stamp.sec, 0);
  EXPECT_EQ(t.header.stamp.nanosec, 0u);
  ASSERT_EQ(t.points.size(), 51u);
  double previous = -1, position = in.q[0];
  for (auto& p : t.points) {
    double stamp = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec;
    EXPECT_GT(stamp, previous);
    previous = stamp;
    ASSERT_EQ(p.positions.size(), 1u);
    EXPECT_GE(p.positions[0], position - 1e-12);
    position = p.positions[0];
    EXPECT_LE(std::abs(p.velocities[0]), .005);
    EXPECT_LE(std::abs(p.accelerations[0]), .005);
  }
  EXPECT_DOUBLE_EQ(t.points.front().positions[0], in.q[0]);
  EXPECT_DOUBLE_EQ(t.points.back().positions[0], candidate.q[0]);
  for (auto p : {t.points.front(), t.points.back()}) {
    EXPECT_DOUBLE_EQ(p.velocities[0], 0);
    EXPECT_DOUBLE_EQ(p.accelerations[0], 0);
  }
  EXPECT_DOUBLE_EQ(previous, 5);
}
TEST_F(GuardTest, InvalidTrajectoryRejected) {
  Plan p;
  p.start = in.q;
  p.target = candidate.q;
  p.duration = 5;
  p.target[1] += .001;
  EXPECT_THROW(Guard::trajectory(p, true), std::invalid_argument);
  p.target = candidate.q;
  p.duration = 0;
  EXPECT_THROW(Guard::trajectory(p, true), std::invalid_argument);
}
TEST_F(GuardTest, ContinuousReference) {
  auto g = running();
  EXPECT_TRUE(Guard::reference(*g.plan(), 0).isApprox(in.q));
  EXPECT_TRUE(Guard::reference(*g.plan(), 5).isApprox(candidate.q));
  EXPECT_NEAR(Guard::reference(*g.plan(), 2.5)[0], .505, 1e-12);
  for (int k = 0; k <= 50; k++) {
    in.q = Guard::reference(*g.plan(), k * .1);
    ASSERT_TRUE(g.monitor(in, 10 + k * .1).ok);
  }
}
TEST_F(GuardTest, ConfigCannotWidenEnvelope) {
  for (int i = 0; i < 5; i++) {
    auto bad = c;
    bad.max_delta[i] += .0001;
    EXPECT_THROW(bad.validate(), std::invalid_argument);
  }
  auto bad = c;
  bad.max_velocity[0] = .021;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = c;
  bad.min_duration = 2;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = c;
  bad.tracking_error[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = c;
  bad.tracking_error[0] = .0101;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  bad = c;
  bad.goal_tolerance[2] = .0201;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
}
TEST_F(GuardTest, InvalidBounds) {
  b.upper[0] = b.lower[0];
  EXPECT_THROW(Guard(c, b), std::invalid_argument);
}

namespace {
using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
constexpr char kCliffDiagnostic[] = "stretch_driver: Pimu/Cliff";
Diagnostic cliffDiagnostic(uint8_t level = Diagnostic::ERROR) {
  Diagnostic status;
  status.name = kCliffDiagnostic;
  status.level = level;
  status.message = "cliff_range=75.556 out of range [-50,20]";
  return status;
}
DiagnosticPolicy liftProfile() {
  DiagnosticPolicy policy;
  policy.error_to_warn_allowlist = {kCliffDiagnostic};
  return policy;
}
}  // namespace
TEST_F(GuardTest, DiagnosticDefaultCliffErrorBlocks) {
  DiagnosticPolicy policy;
  const auto result = policy.classify(cliffDiagnostic());
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
  in.diagnostics_ok = !result.blocking();
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticAllowlistedCliffBaseDisabledWarnsAndValidates) {
  const auto policy = liftProfile();
  EXPECT_NO_THROW(policy.validate());
  const auto result = policy.classify(cliffDiagnostic());
  EXPECT_EQ(result.original_level, Diagnostic::ERROR);
  EXPECT_EQ(result.effective_level, Diagnostic::WARN);
  EXPECT_TRUE(result.downgraded);
  EXPECT_FALSE(result.blocking());
  EXPECT_NE(result.reason.find("bounded cliff exception"), std::string::npos);
  EXPECT_NE(result.reason.find("[-100,200]"), std::string::npos);
  EXPECT_NE(result.reason.find("base motion disabled"), std::string::npos);
  in.diagnostics_ok = !result.blocking();
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  EXPECT_EQ(g.state(), State::READY);
  EXPECT_EQ(Guard::trajectory(*g.plan(), true).joint_names,
            std::vector<std::string>{"joint_lift"});
  EXPECT_NEAR(g.plan()->target[0] - g.plan()->start[0], .01, 1e-12);
}
TEST_F(GuardTest, DiagnosticCliffBaseEnabledRejects) {
  auto policy = liftProfile();
  policy.base_motion_enabled = true;
  const auto result = policy.classify(cliffDiagnostic());
  EXPECT_FALSE(result.downgraded);
  EXPECT_TRUE(result.blocking());
  in.diagnostics_ok = !result.blocking();
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticCliffOutsideCalibratedRangeBlocks) {
  auto status = cliffDiagnostic();
  status.message = "cliff_range=225.0 out of range [-50,20]";
  const auto result = liftProfile().classify(status);
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
}
TEST_F(GuardTest, DiagnosticCliffKeyValueInsideRangeWarns) {
  auto status = cliffDiagnostic();
  status.message = "legacy cliff warning";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "cliff_range";
  kv.value = "-99.5";
  status.values.push_back(kv);
  const auto result = liftProfile().classify(status);
  EXPECT_FALSE(result.blocking());
  EXPECT_TRUE(result.downgraded);
}
TEST_F(GuardTest, DiagnosticInvalidCliffRangeConfigRejects) {
  auto policy = liftProfile();
  policy.cliff_range_min = 200.0;
  policy.cliff_range_max = -100.0;
  EXPECT_THROW(policy.validate(), std::invalid_argument);
}
TEST_F(GuardTest, DiagnosticArbitraryErrorRejects) {
  auto status = cliffDiagnostic();
  status.name = "other_driver: unrelated sensor";
  const auto result = liftProfile().classify(status);
  EXPECT_TRUE(result.blocking());
  in.diagnostics_ok = !result.blocking();
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticLiftAndControllerFaultsAlwaysReject) {
  for (const auto name :
       {"stretch_driver: Lift/Device", "stretch_driver: Lift/Motor",
        "stretch_controller: follow_joint_trajectory"}) {
    auto status = cliffDiagnostic();
    status.name = name;
    auto policy = liftProfile();
    // Even a mistaken explicit allowlist entry cannot downgrade a protected
    // fault.
    policy.error_to_warn_allowlist.push_back(name);
    EXPECT_THROW(policy.validate(), std::invalid_argument);
    const auto result = policy.classify(status);
    EXPECT_TRUE(result.blocking());
    EXPECT_FALSE(result.downgraded);
    in.diagnostics_ok = !result.blocking();
    auto g = armed();
    EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  }
}
TEST_F(GuardTest, DiagnosticStaleStreamRejectsDespiteCliffException) {
  in.diagnostics_ok = !liftProfile().classify(cliffDiagnostic()).blocking();
  in.diagnostics_fresh = false;
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticStaleStreamDuringExecutionAborts) {
  in.diagnostics_ok = !liftProfile().classify(cliffDiagnostic()).blocking();
  auto g = running();
  in.diagnostics_fresh = false;
  EXPECT_FALSE(g.monitor(in, 10).ok);
  EXPECT_EQ(g.state(), State::ABORTED);
  EXPECT_FALSE(g.armed());
}
TEST_F(GuardTest, DiagnosticStaleStatusCannotBeDowngraded) {
  const auto result =
      liftProfile().classify(cliffDiagnostic(Diagnostic::STALE));
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
  EXPECT_EQ(result.effective_level, Diagnostic::STALE);
  in.diagnostics_ok = !result.blocking();
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticAllowlistMustMatchExactly) {
  for (const auto name : {"stretch_driver: Pimu/Cliff ", "Pimu/Cliff",
                          "stretch_driver: Pimu/Cliff/Lift", "*"}) {
    auto status = cliffDiagnostic();
    status.name = name;
    auto policy = liftProfile();
    EXPECT_TRUE(policy.classify(status).blocking());
    policy.error_to_warn_allowlist = {name};
    EXPECT_THROW(policy.validate(), std::invalid_argument);
    EXPECT_TRUE(policy.classify(status).blocking());
  }
}
TEST_F(GuardTest, DiagnosticWarnAndOkArePreserved) {
  for (const auto level : {Diagnostic::OK, Diagnostic::WARN}) {
    const auto result = liftProfile().classify(cliffDiagnostic(level));
    EXPECT_EQ(result.effective_level, level);
    EXPECT_FALSE(result.downgraded);
    EXPECT_FALSE(result.blocking());
  }
  EXPECT_TRUE(liftProfile().classify(cliffDiagnostic(255)).blocking());
}
TEST_F(GuardTest, DiagnosticExceptionCannotBypassStateGuards) {
  in.diagnostics_ok = !liftProfile().classify(cliffDiagnostic()).blocking();
  for (bool Input::* member :
       {&Input::fresh, &Input::homed, &Input::required_joints}) {
    auto bad = in;
    bad.*member = false;
    auto g = armed();
    EXPECT_FALSE(g.validate(bad, candidate, 5, 10).ok);
  }
  auto g = armed();
  in.runstopped = true;
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(GuardTest, DiagnosticExceptionCannotBypassTrackingOrControllerFailure) {
  in.diagnostics_ok = !liftProfile().classify(cliffDiagnostic()).blocking();
  auto tracking = running();
  auto bad = in;
  bad.q[0] += c.tracking_error[0] + .0001;
  EXPECT_FALSE(tracking.monitor(bad, 10).ok);
  EXPECT_EQ(tracking.state(), State::ABORTED);
  auto controller = running();
  EXPECT_FALSE(controller.finish(in, false, 10).ok);
  EXPECT_EQ(controller.reason(), "controller failure");
}

class ControllerGuardTest : public GuardTest {
 protected:
  ControllerState controller;
  void apply(bool available, bool pending = false) {
    const auto report = controller.preflight(available, pending);
    in.action_server_available = report.action_server_available;
    in.status_message_received = report.status_message_received;
    in.active_goal_present = report.active_goal_present;
    in.controller_idle = report.controller_idle;
  }
  void status(std::initializer_list<int8_t> states) {
    action_msgs::msg::GoalStatusArray message;
    for (const auto state : states) {
      action_msgs::msg::GoalStatus goal;
      goal.status = state;
      message.status_list.push_back(goal);
    }
    controller.observe(message);
  }
  void rejects(int8_t active) {
    status({active});
    apply(true);
    EXPECT_TRUE(in.status_message_received);
    EXPECT_TRUE(in.active_goal_present);
    EXPECT_FALSE(in.controller_idle);
    auto g = armed();
    EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  }
};
using GoalStatus = action_msgs::msg::GoalStatus;
TEST_F(ControllerGuardTest, ServerUnavailableRejects) {
  apply(false);
  EXPECT_FALSE(in.controller_idle);
  auto g = armed();
  auto result = g.validate(in, candidate, 5, 10);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "trajectory action server unavailable");
  in.controller_idle =
      true;  // Cannot bypass availability with inconsistent input.
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, AvailableWithoutPriorStatusIsIdle) {
  apply(true);
  EXPECT_FALSE(in.status_message_received);
  EXPECT_FALSE(in.active_goal_present);
  EXPECT_TRUE(in.controller_idle);
  auto g = armed();
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, EmptyStatusMeansIdle) {
  status({});
  apply(true);
  EXPECT_TRUE(in.status_message_received);
  EXPECT_TRUE(in.controller_idle);
  auto g = armed();
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, AcceptedBlocks) {
  rejects(GoalStatus::STATUS_ACCEPTED);
}
TEST_F(ControllerGuardTest, ExecutingBlocks) {
  rejects(GoalStatus::STATUS_EXECUTING);
}
TEST_F(ControllerGuardTest, CancelingBlocks) {
  rejects(GoalStatus::STATUS_CANCELING);
}
TEST_F(ControllerGuardTest, TerminalGoalsOnlyAreIdle) {
  status({GoalStatus::STATUS_SUCCEEDED, GoalStatus::STATUS_ABORTED,
          GoalStatus::STATUS_CANCELED});
  apply(true);
  EXPECT_TRUE(in.controller_idle);
  EXPECT_FALSE(in.active_goal_present);
  auto g = armed();
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, ActiveAmongTerminalGoalsStillBlocks) {
  status({GoalStatus::STATUS_SUCCEEDED, GoalStatus::STATUS_EXECUTING,
          GoalStatus::STATUS_ABORTED});
  apply(true);
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest,
       LocalPendingGoalBlocksWithoutStatusAndWithEmptyStatus) {
  apply(true, true);
  EXPECT_TRUE(in.active_goal_present);
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  status({});
  apply(true, true);
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  apply(true, false);
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, ServerLossDoesNotForgetKnownActiveGoal) {
  status({GoalStatus::STATUS_CANCELING});
  apply(false);
  EXPECT_TRUE(in.active_goal_present);
  apply(true);
  EXPECT_FALSE(in.controller_idle);
  auto g = armed();
  EXPECT_FALSE(g.validate(in, candidate, 5, 10).ok);
  status({GoalStatus::STATUS_CANCELED});
  apply(true);
  EXPECT_TRUE(g.validate(in, candidate, 5, 10).ok);
}
TEST_F(ControllerGuardTest, ActiveGoalRecheckedBeforeExecute) {
  apply(true);
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  status({GoalStatus::STATUS_ACCEPTED});
  apply(true);
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(ControllerGuardTest, ServerUnavailableAfterApprovalRejects) {
  apply(true);
  auto g = armed();
  ASSERT_TRUE(g.validate(in, candidate, 5, 10).ok);
  apply(false);
  EXPECT_FALSE(g.begin(in, candidate, 10).ok);
  EXPECT_FALSE(g.armed());
}
TEST_F(ControllerGuardTest, ServerLossDuringExecutionAborts) {
  apply(true);
  auto g = running();
  apply(false);
  EXPECT_FALSE(g.monitor(in, 10).ok);
  EXPECT_EQ(g.state(), State::ABORTED);
  EXPECT_FALSE(g.armed());
}

// --- Pimu/Temperature bounded diagnostic tests ---
namespace {
constexpr char kTempDiagnostic[] = "stretch_driver: Pimu/Temperature";
Diagnostic tempDiagnostic(double temp_c, uint8_t level = Diagnostic::ERROR) {
  Diagnostic status;
  status.name = kTempDiagnostic;
  status.level = level;
  status.message = "temperature=" + std::to_string(temp_c) + " C";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "temperature";
  kv.value = std::to_string(temp_c);
  status.values.push_back(kv);
  return status;
}
DiagnosticPolicy tempPolicy() {
  DiagnosticPolicy policy;
  policy.pimu_temperature_error_can_warn = true;
  policy.pimu_temperature_warn_max_c = 50.0;
  return policy;
}
}  // namespace

TEST_F(GuardTest, DiagnosticTemperatureDefaultBlocks) {
  // Default policy: pimu_temperature_error_can_warn = false
  DiagnosticPolicy policy;
  const auto result = policy.classify(tempDiagnostic(40.0));
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
}

TEST_F(GuardTest, DiagnosticTemperatureWithinCeilingWarns) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(tempDiagnostic(49.5));
  EXPECT_EQ(result.original_level, Diagnostic::ERROR);
  EXPECT_EQ(result.effective_level, Diagnostic::WARN);
  EXPECT_TRUE(result.downgraded);
  EXPECT_FALSE(result.blocking());
  EXPECT_NE(result.reason.find("bounded temperature"), std::string::npos);
}

TEST_F(GuardTest, DiagnosticTemperatureExactCeilingWarns) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(tempDiagnostic(50.0));
  EXPECT_FALSE(result.blocking());
  EXPECT_TRUE(result.downgraded);
}

TEST_F(GuardTest, DiagnosticTemperatureAboveCeilingBlocks) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(tempDiagnostic(50.1));
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
}

TEST_F(GuardTest, DiagnosticTemperatureNonFiniteBlocks) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(
      tempDiagnostic(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
}

TEST_F(GuardTest, DiagnosticTemperatureHardCapValidation) {
  DiagnosticPolicy policy;
  policy.pimu_temperature_error_can_warn = true;
  policy.pimu_temperature_warn_max_c = 50.1;  // exceeds hard cap of 50.0
  EXPECT_THROW(policy.validate(), std::invalid_argument);
}

TEST_F(GuardTest, DiagnosticTemperatureNegativeThresholdValidation) {
  DiagnosticPolicy policy;
  policy.pimu_temperature_error_can_warn = true;
  policy.pimu_temperature_warn_max_c = -1.0;
  EXPECT_THROW(policy.validate(), std::invalid_argument);
}

TEST_F(GuardTest, DiagnosticTemperatureStaleNotDowngraded) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(tempDiagnostic(40.0, Diagnostic::STALE));
  EXPECT_TRUE(result.blocking());
  EXPECT_FALSE(result.downgraded);
}

TEST_F(GuardTest, DiagnosticTemperatureWarnPreserved) {
  const auto policy = tempPolicy();
  const auto result = policy.classify(tempDiagnostic(40.0, Diagnostic::WARN));
  EXPECT_FALSE(result.blocking());
  EXPECT_FALSE(result.downgraded);  // Not downgraded since it was already WARN
}
