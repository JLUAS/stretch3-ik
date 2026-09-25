#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "round_trip.hpp"
#include "stretch_custom_kinematics/forward_kinematics.hpp"
using namespace stretch_robot_interface;
using namespace stretch_custom_kinematics;
std::shared_ptr<RobotModel> model() {
  std::ifstream f(TEST_URDF);
  std::stringstream s;
  s << f.rdbuf();
  return std::make_shared<RobotModel>(s.str());
}
TEST(FK, GoldenConfiguration0) {
  ForwardKinematics fk(model());
  RobotState q;
  q.lift = 0;
  q.arm_extension = 0;
  q.wrist_yaw = 0;
  q.wrist_pitch = 0;
  q.wrist_roll = 0;
  q.valid = true;
  auto actual = fk.computeTcpTransform(q);
  Eigen::Matrix4d expected;
  expected << 0.0068917464147982945, 0.99950893075994318, 0.030567976093751146,
      -0.021184760481462474, -0.99972747296326558, 0.0075686428736086468,
      -0.022083827737370734, -0.41007843040758263, -0.022304341143288824,
      -0.030407449353173282, 0.99928869872024351, 0.087605441790038735, 0, 0, 0,
      1;
  EXPECT_LT((actual.matrix() - expected).norm(), 1e-12);
}
TEST(FK, GoldenConfiguration1) {
  ForwardKinematics fk(model());
  RobotState q;
  q.lift = 0.6;
  q.arm_extension = 0.1;
  q.wrist_yaw = 0.3;
  q.wrist_pitch = -0.4;
  q.wrist_roll = 0.2;
  q.valid = true;
  auto actual = fk.computeTcpTransform(q);
  Eigen::Matrix4d expected;
  expected << 0.26622017389433433, 0.96279175342753298, -0.046463518416123749,
      0.035298874472866629, -0.86902362974858871, 0.21887879104631694,
      -0.44372289299594347, -0.47730436352800842, -0.41704286344482283,
      0.15850588115914765, 0.89495873406973592, 0.58081041705417591, 0, 0, 0, 1;
  EXPECT_LT((actual.matrix() - expected).norm(), 1e-12);
}
TEST(FK, GoldenConfiguration2) {
  ForwardKinematics fk(model());
  RobotState q;
  q.lift = 1;
  q.arm_extension = 0.48;
  q.wrist_yaw = -1.2;
  q.wrist_pitch = 0.4;
  q.wrist_roll = -2;
  q.valid = true;
  auto actual = fk.computeTcpTransform(q);
  Eigen::Matrix4d expected;
  expected << -0.84383748798363656, -0.50798431636328156, 0.1728879064665737,
      -0.27517395100317543, -0.34876333639541185, 0.27434886695385163,
      -0.89615670191517161, -0.7368885473591883, 0.40780194832762345,
      -0.81650758326549966, -0.40867216373289589, 1.1946842451571049, 0, 0, 0,
      1;
  EXPECT_LT((actual.matrix() - expected).norm(), 1e-12);
}

TEST(FK, LiftTranslationFromUrdf) {
  auto m = model();
  ForwardKinematics fk(m);
  RobotState a;
  a.valid = true;
  auto b = a;
  b.lift = .4;
  auto x = fk.computeTcpTransform(a), y = fk.computeTcpTransform(b);
  EXPECT_NEAR((y.translation() - x.translation()).norm(), .4, 1e-12);
  EXPECT_LT(poseError(x, y).orientation_rad, 1e-12);
}
TEST(FK, RejectInvalidAndOutOfBounds) {
  ForwardKinematics fk(model());
  RobotState q;
  EXPECT_THROW(fk.computeTcpPose(q), std::invalid_argument);
  q.valid = true;
  q.lift = 2;
  EXPECT_THROW(fk.computeTcpPose(q), std::invalid_argument);
  EXPECT_THROW(ForwardKinematics(nullptr), std::invalid_argument);
}
TEST(FK, PoseMessageAndErrorUnits) {
  ForwardKinematics fk(model());
  RobotState q;
  q.valid = true;
  auto p = fk.computeTcpPose(q);
  auto t = fk.computeTcpTransform(q);
  EXPECT_NEAR(p.position.x, t.translation().x(), 1e-15);
  EXPECT_NEAR(Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                 p.orientation.y, p.orientation.z)
                  .norm(),
              1, 1e-12);
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity(), b = a;
  b.translation().x() = .001;
  b.linear() =
      Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto e = poseError(a, b);
  EXPECT_NEAR(e.position_m, .001, 1e-15);
  EXPECT_NEAR(e.orientation_rad, .2, 1e-15);
}
TEST(RoundTrip, LogicalPhysicalLogicalGrid) {
  auto m = model();
  ForwardKinematics fk(m);
  for (double lift : {0., .6, 1.1})
    for (double arm : {0., .1, .52})
      for (double yaw : {-1., 0., 3.}) {
        RobotState q;
        q.valid = true;
        q.lift = lift;
        q.arm_extension = arm;
        q.wrist_yaw = yaw;
        q.wrist_pitch = -.4;
        q.wrist_roll = .2;
        sensor_msgs::msg::JointState s;
        for (const auto& [n, v] : m->physicalJoints(q)) {
          s.name.push_back(n);
          s.position.push_back(v);
        }
        auto back = m->decode(s);
        auto e =
            poseError(fk.computeTcpTransform(q), fk.computeTcpTransform(back));
        EXPECT_LT(e.position_m, 1e-12);
        EXPECT_LT(e.orientation_rad, 1e-12);
      }
}
TEST(RoundTrip, FutureSolverHarnessDetectsResidual) {
  ForwardKinematics fk(model());
  RobotState seed;
  seed.valid = true;
  auto e = test_support::checkRoundTrip(
      fk, seed, [](const Eigen::Isometry3d&, RobotState q) {
        q.lift = .01;
        return q;
      });
  EXPECT_NEAR(e.position_m, .01, 1e-12);
}
