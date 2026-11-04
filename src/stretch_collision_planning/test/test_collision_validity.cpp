#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <memory>
#include <sstream>

#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/state_validity.hpp"

using namespace stretch_collision_planning;

namespace {
std::shared_ptr<RobotModel> model() {
  std::ifstream file(TEST_URDF);
  std::stringstream xml;
  xml << file.rdbuf();
  return std::make_shared<RobotModel>(xml.str());
}

StateValidityChecker checker() {
  return StateValidityChecker(
      model(), JointBounds::load(TEST_LIMITS),
      std::make_shared<PrimitiveRobotCollisionModel>(
          PrimitiveRobotCollisionModel::makeConservativeStretchArm()),
      std::make_shared<PrimitiveEnvironment>(
          PrimitiveEnvironment::makeDeskBenchmark()));
}

JointVector q(double lift, double arm, double yaw = 0.0, double pitch = -0.6,
              double roll = 0.0) {
  JointVector value;
  value << lift, arm, yaw, pitch, roll;
  return value;
}
}  // namespace

TEST(CollisionValidity, KnownFreeConfiguration) {
  auto c = checker();
  const auto result = c.isStateValid(q(0.35, 0.06));
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.code, ValidityCode::Free);
}

TEST(CollisionValidity, KnownCollisionConfiguration) {
  auto c = checker();
  const auto result = c.isStateValid(q(0.55, 0.25));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.code, ValidityCode::EnvironmentCollision);
  ASSERT_TRUE(result.contact.has_value());
  EXPECT_EQ(result.contact->environment_geometry, "desk_tabletop");
  EXPECT_EQ(result.contact->kind, CollisionContactKind::Occupied);
  EXPECT_TRUE(result.contact->robot_capsule.has_value());
  EXPECT_TRUE(result.contact->environment_box.has_value());
}

TEST(CollisionValidity, JointLimitViolation) {
  auto c = checker();
  auto bad = q(0.35, 0.06);
  bad[0] = c.bounds().lower[0] - 0.001;
  const auto result = c.isStateValid(bad);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.code, ValidityCode::JointLimit);
}

TEST(CollisionValidity, NonFiniteConfiguration) {
  auto c = checker();
  auto bad = q(0.35, 0.06);
  bad[1] = std::numeric_limits<double>::quiet_NaN();
  const auto result = c.isStateValid(bad);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.code, ValidityCode::NonFinite);
}

TEST(CollisionValidity, ValidEdge) {
  auto c = checker();
  const auto result = c.isEdgeValid(q(0.80, 0.06), q(0.90, 0.12), 0.02);
  EXPECT_TRUE(result.valid) << result.first_invalid.reason;
  EXPECT_GE(result.samples_checked, 2);
}

TEST(CollisionValidity, EdgeEndpointsFreeButInteriorCollides) {
  auto c = checker();
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  ASSERT_TRUE(c.isStateValid(start).valid);
  ASSERT_TRUE(c.isStateValid(goal).valid);
  const auto edge = c.isEdgeValid(start, goal, 0.01);
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.first_invalid.code, ValidityCode::EnvironmentCollision);
}

TEST(CollisionValidity, DirectPathCollisionBenchmark) {
  auto c = checker();
  const auto direct = c.isEdgeValid(q(0.35, 0.06), q(0.75, 0.44), 0.005);
  EXPECT_FALSE(direct.valid);
  EXPECT_GT(direct.samples_checked, 2);
}

TEST(CollisionValidity, HighLiftBypassEdgeIsFree) {
  auto c = checker();
  const auto lifted = c.isEdgeValid(q(0.80, 0.06), q(0.90, 0.44), 0.01);
  EXPECT_TRUE(lifted.valid) << lifted.first_invalid.reason;
}
