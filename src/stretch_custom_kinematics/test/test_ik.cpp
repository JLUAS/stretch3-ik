#include <gtest/gtest.h>

#include <fstream>
#include <random>
#include <sstream>

#include "stretch_custom_kinematics/inverse_kinematics.hpp"
using namespace stretch_custom_kinematics;
std::shared_ptr<stretch_robot_interface::RobotModel> model() {
  std::ifstream f(TEST_URDF);
  std::stringstream s;
  s << f.rdbuf();
  return std::make_shared<stretch_robot_interface::RobotModel>(s.str());
}
RobotState seed() {
  RobotState s;
  s.valid = true;
  s.lift = .6;
  s.arm_extension = .1;
  s.wrist_pitch = -.6;
  return s;
}
TEST(IK, HardwareIntersection) {
  InverseKinematics ik(model(), JointBounds::load(TEST_LIMITS));
  EXPECT_DOUBLE_EQ(ik.bounds().upper[1], .5167554651511435);
  EXPECT_DOUBLE_EQ(ik.bounds().upper[2], 4.0);
  EXPECT_DOUBLE_EQ(ik.bounds().lower[3], -1.57);
}
TEST(IK, InvalidTarget) {
  InverseKinematics ik(model(), JointBounds::load(TEST_LIMITS));
  geometry_msgs::msg::Pose p;
  p.orientation.w = 0;
  EXPECT_EQ(ik.solve(p, seed(), 0).reason, IkFailure::InvalidTarget);
  p.orientation.w = 1;
  p.position.x = NAN;
  EXPECT_EQ(ik.solve(p, seed(), 0).reason, IkFailure::InvalidTarget);
  auto t = Eigen::Isometry3d::Identity();
  t.linear()(0, 0) = 2;
  EXPECT_EQ(ik.solve(t, seed(), 0).reason, IkFailure::InvalidTarget);
}
TEST(IK, StaleState) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  auto t = ForwardKinematics(m).computeTcpTransform(seed());
  EXPECT_EQ(ik.solve(t, seed(), 2).reason, IkFailure::StaleState);
  auto s = seed();
  s.valid = false;
  EXPECT_EQ(ik.solve(t, s, 0).reason, IkFailure::StaleState);
  EXPECT_EQ(ik.solve(t, seed(), -.2).reason, IkFailure::StaleState);
}
TEST(IK, ExplicitUnreachablePoses) {
  InverseKinematics ik(model(), JointBounds::load(TEST_LIMITS));
  for (auto p : {Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, -10, 0),
                 Eigen::Vector3d(0, 0, 10), Eigen::Vector3d(0, 0, -10)}) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = p;
    auto r = ik.solve(t, seed(), 0);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.reason, IkFailure::UnreachableTarget);
  }
}
TEST(IK, SeedLimitViolation) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  auto s = seed();
  s.arm_extension = .52;
  EXPECT_EQ(
      ik.solve(ForwardKinematics(m).computeTcpTransform(seed()), s, 0).reason,
      IkFailure::JointLimit);
}
TEST(IK, ActiveJointLimitFailure) {
  auto m = model();
  auto s = seed();
  JointBounds b;
  b.lower = coordinates(s) - JointVector::Constant(.001);
  b.upper = coordinates(s) + JointVector::Constant(.001);
  InverseKinematics ik(m, b);
  auto target = s;
  target.lift += .2;
  auto r = ik.solve(ForwardKinematics(m).computeTcpTransform(target), s, 0);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.reason, IkFailure::JointLimit);
}
TEST(IK, LiftWorkspaceMatrixDoesNotClampReachableTargets) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  ForwardKinematics fk(m);
  const auto start = seed();
  for (double requested : {0.15, 0.55, 0.90, ik.bounds().upper[0] - 0.01}) {
    auto target_state = start;
    target_state.lift = requested;
    const auto target = fk.computeTcpTransform(target_state);
    const auto result = ik.solve(target, start, 0);
    ASSERT_TRUE(result.success)
        << "requested lift " << requested << ": " << failureName(result.reason);
    EXPECT_NEAR(result.solution.lift, requested, 1e-5);
  }

  auto upper_state = start;
  upper_state.lift = ik.bounds().upper[0];
  auto unreachable_target = fk.computeTcpTransform(upper_state);
  unreachable_target.translation().z() += 0.20;
  const auto unreachable = ik.solve(unreachable_target, start, 0);
  EXPECT_FALSE(unreachable.success);
  EXPECT_TRUE(unreachable.reason == IkFailure::JointLimit ||
              unreachable.reason == IkFailure::UnreachableTarget);
}
TEST(IK, ConvergenceBudgetFailure) {
  auto m = model();
  IkOptions o;
  o.max_iterations = 0;
  o.max_attempts = 1;
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS), o);
  auto target = seed();
  target.lift += .1;
  EXPECT_EQ(
      ik.solve(ForwardKinematics(m).computeTcpTransform(target), seed(), 0)
          .reason,
      IkFailure::ConvergenceFailure);
}
TEST(IK, TaskLeavesSpinAboutTargetZFree) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  auto target = ForwardKinematics(m).computeTcpTransform(seed());
  target.linear() =
      target.linear() *
      Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto r = ik.solve(target, seed(), 0);
  ASSERT_TRUE(r.success);
  EXPECT_LT(r.direction_error, 1e-10);
  EXPECT_NEAR(r.orientation_error, 1., 1e-10);
  EXPECT_EQ(r.attempts, 1);
}
TEST(IK, RandomFkIkFk) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  ForwardKinematics fk(m);
  std::mt19937 rng(934);
  std::uniform_real_distribution<double> u(0, 1);
  for (int k = 0; k < 100; ++k) {
    JointVector q;
    for (int j = 0; j < 5; ++j)
      q[j] = ik.bounds().lower[j] +
             u(rng) * (ik.bounds().upper[j] - ik.bounds().lower[j]);
    auto target = fk.computeTcpTransform(withCoordinates(seed(), q));
    auto r = ik.solve(target, seed(), 0);
    ASSERT_TRUE(r.success) << "sample " << k << " " << failureName(r.reason);
    EXPECT_TRUE(ik.bounds().contains(coordinates(r.solution)));
    auto actual = fk.computeTcpTransform(r.solution);
    EXPECT_LT((target.translation() - actual.translation()).norm(), 1e-6);
    EXPECT_LT((target.linear().col(2) - actual.linear().col(2)).norm(), 1e-5);
  }
}
TEST(IK, GraspFixedBaseRoundTrip) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  ForwardKinematics fk(m);
  std::mt19937 rng(77);
  std::uniform_real_distribution<double> u(0, 1);
  for (int k = 0; k < 30; ++k) {
    JointVector q;
    for (int j = 0; j < 5; ++j)
      q[j] = ik.bounds().lower[j] +
             u(rng) * (ik.bounds().upper[j] - ik.bounds().lower[j]);
    auto target = fk.computeTcpTransform(withCoordinates(seed(), q));
    auto r = ik.solveGrasp(target, seed(), 0, false);
    ASSERT_TRUE(r.success) << "sample " << k << " " << failureName(r.reason);
    EXPECT_EQ(r.base_dx, 0);
    EXPECT_LT(r.position_error, 1e-3);
    EXPECT_LT(r.approach_error, .12);
    EXPECT_LT(r.direction_error, .15);
  }
}
TEST(IK, GraspFreeBaseRecoversOffset) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  ForwardKinematics fk(m);
  RobotState q = seed();
  q.lift = .9;
  q.arm_extension = .3;
  q.wrist_yaw = 0;
  q.wrist_pitch = 0;
  q.wrist_roll = 0;
  auto target = fk.computeTcpTransform(q);
  target.pretranslate(Eigen::Vector3d(.15, 0, 0));
  EXPECT_FALSE(ik.solveGrasp(target, seed(), 0, false).success);
  auto r = ik.solveGrasp(target, seed(), 0, true);
  ASSERT_TRUE(r.success) << failureName(r.reason);
  EXPECT_NEAR(r.base_dx, .15, 1e-3);
  EXPECT_NEAR(r.solution.lift, .9, 1e-3);
  EXPECT_NEAR(r.solution.arm_extension, .3, 1e-3);
}
TEST(IK, ShelfSideGraspAfterBaseOffset) {
  auto m = model();
  InverseKinematics ik(m, JointBounds::load(TEST_LIMITS));
  // Horizontal approach toward the robot's right (-Y), fingers closing along
  // base X, at shelf height 1.0 m and 0.10 m forward of the base origin.
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d r;
  r.col(0) = -Eigen::Vector3d::UnitY();
  r.col(1) = Eigen::Vector3d::UnitX();
  r.col(2) = r.col(0).cross(r.col(1));
  target.linear() = r;
  target.translation() = Eigen::Vector3d(.10, -.55, 1.0);
  auto free = ik.solveGrasp(target, seed(), 0, true);
  ASSERT_TRUE(free.success) << failureName(free.reason);
  auto moved = target;
  moved.pretranslate(Eigen::Vector3d(-free.base_dx, 0, 0));
  auto fixed = ik.solveGrasp(moved, seed(), 0, false);
  ASSERT_TRUE(fixed.success) << failureName(fixed.reason);
  EXPECT_LT(fixed.approach_error, .05);
  EXPECT_NEAR(fixed.solution.lift, free.solution.lift, 5e-3);
}
TEST(IK, MissingLimitFile) {
  EXPECT_THROW(JointBounds::load("/nonexistent/stretch_limits.csv"),
               std::invalid_argument);
}
