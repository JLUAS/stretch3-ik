#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <sstream>

#include "stretch_robot_interface/model.hpp"
using namespace stretch_robot_interface;
std::string xml() {
  std::ifstream f(TEST_URDF);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}
sensor_msgs::msg::JointState sample() {
  sensor_msgs::msg::JointState m;
  m.header.stamp.sec = 123;
  m.name = {"joint_wrist_roll",  "joint_arm_l0", "joint_lift",
            "joint_wrist_pitch", "joint_arm_l2", "joint_wrist_yaw",
            "joint_arm_l3",      "joint_arm_l1", "wrist_extension"};
  m.position = {0.2, .025, .6, -.4, .025, .3, .025, .025, .1};
  return m;
}
TEST(Mapping, NamesNotArrayOrder) {
  RobotModel m(xml());
  auto q = m.decode(sample());
  EXPECT_DOUBLE_EQ(q.lift, .6);
  EXPECT_DOUBLE_EQ(q.arm_extension, .1);
  EXPECT_DOUBLE_EQ(q.wrist_yaw, .3);
  EXPECT_DOUBLE_EQ(q.wrist_pitch, -.4);
  EXPECT_DOUBLE_EQ(q.wrist_roll, .2);
  EXPECT_EQ(q.timestamp.sec, 123);
  EXPECT_TRUE(q.valid);
}
TEST(Mapping, TelescopeSumAndExpansion) {
  RobotModel m(xml());
  auto s = sample();
  s.position[1] = .01;
  s.position[4] = .04;
  auto q = m.decode(s);
  EXPECT_NEAR(q.arm_extension, .1, 1e-15);
  auto p = m.physicalJoints(q);
  for (auto& n : RobotModel::arm_names) EXPECT_NEAR(p.at(n), .025, 1e-15);
}
TEST(Mapping, RejectInconsistentAlias) {
  RobotModel m(xml());
  auto s = sample();
  s.position.back() = .2;
  EXPECT_THROW(m.decode(s), std::invalid_argument);
}
TEST(Mapping, AliasOptionalStagesRequired) {
  RobotModel m(xml());
  auto s = sample();
  s.name.pop_back();
  s.position.pop_back();
  EXPECT_NO_THROW(m.decode(s));
  s.name.erase(s.name.begin() + 1);
  s.position.erase(s.position.begin() + 1);
  EXPECT_THROW(m.decode(s), std::invalid_argument);
}
TEST(Mapping, RejectMissingMismatchedDuplicateNonfinite) {
  RobotModel m(xml());
  auto s = sample();
  s.position.pop_back();
  EXPECT_THROW(m.decode(s), std::invalid_argument);
  s = sample();
  s.name[0] = "unknown";
  EXPECT_THROW(m.decode(s), std::invalid_argument);
  s = sample();
  s.name[0] = s.name[1];
  EXPECT_THROW(m.decode(s), std::invalid_argument);
  s = sample();
  s.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(m.decode(s), std::invalid_argument);
}
TEST(Limits, BoundariesAndViolation) {
  RobotModel m(xml());
  auto q = m.decode(sample());
  q.lift = 1.1;
  q.arm_extension = .52;
  q.wrist_yaw = 4;
  q.wrist_pitch = .56;
  q.wrist_roll = 3.14;
  EXPECT_NO_THROW(m.physicalJoints(q));
  q.lift = 1.1001;
  EXPECT_THROW(m.physicalJoints(q), std::invalid_argument);
  q.lift = .5;
  q.arm_extension = .5201;
  EXPECT_THROW(m.physicalJoints(q), std::invalid_argument);
  q.arm_extension = -.01;
  EXPECT_THROW(m.physicalJoints(q), std::invalid_argument);
}
TEST(Limits, PhysicalStageMustRespectLimit) {
  RobotModel m(xml());
  auto s = sample();
  s.position[1] = .14;
  s.position.back() = .215;
  EXPECT_THROW(m.decode(s), std::invalid_argument);
}
TEST(Freshness, ReceiptAndHeaderBothRequired) {
  EXPECT_TRUE(fresh(.01, .02, 1));
  EXPECT_FALSE(fresh(2, .02, 1));
  EXPECT_FALSE(fresh(.01, 2, 1));
  EXPECT_FALSE(fresh(.01, -1, 1));
  EXPECT_FALSE(fresh(.01, -.051, 1));
  EXPECT_TRUE(fresh(.01, -.109, 1, .15));
  EXPECT_FALSE(fresh(.01, -.151, 1, .15));
  EXPECT_FALSE(fresh(.01, -.01, 1, .21));
  EXPECT_FALSE(fresh(-1, .01, 1));
  EXPECT_FALSE(fresh(.01, std::numeric_limits<double>::quiet_NaN(), 1));
  EXPECT_FALSE(fresh(.01, .01, 0));
}
TEST(Model, MalformedAndAbsentLinks) {
  EXPECT_THROW(RobotModel("<broken>"), std::invalid_argument);
  EXPECT_THROW(RobotModel(xml(), "missing"), std::invalid_argument);
  EXPECT_THROW(RobotModel(xml(), "base_link", "missing"),
               std::invalid_argument);
  EXPECT_THROW(RobotModel(xml(), "link_head"), std::invalid_argument);
}
TEST(Model, RejectMimicInsteadOfGuessing) {
  auto s = xml();
  auto p = s.find("<joint name=\"joint_arm_l2\"");
  p = s.find('>', p) + 1;
  s.insert(p, "<mimic joint=\"joint_arm_l3\" multiplier=\"1\"/>");
  EXPECT_THROW((void)RobotModel(s), std::invalid_argument);
}
TEST(Model, RejectNonparallelTelescope) {
  auto s = xml();
  auto p = s.find("<joint name=\"joint_arm_l2\"");
  p = s.find("<axis xyz=\"0.0 0.0 1.0\"", p);
  ASSERT_NE(p, std::string::npos);
  s.replace(p, std::string("<axis xyz=\"0.0 0.0 1.0\"").size(),
            "<axis xyz=\"1.0 0.0 0.0\"");
  EXPECT_THROW((void)RobotModel(s), std::invalid_argument);
}
