#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <memory>
#include <sstream>

#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/rrt_connect.hpp"

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

PlannerOptions benchmarkOptions() {
  PlannerOptions options;
  options.max_iterations = 4000;
  options.planning_timeout = 3.0;
  options.max_extension_distance = 0.12;
  options.edge_resolution = 0.01;
  options.goal_bias = 0.10;
  options.random_seed = 29;
  options.shortcut_attempts = 200;
  return options;
}

void expectExactConfig(const JointVector& actual, const JointVector& expected) {
  for (int i = 0; i < 5; ++i) EXPECT_DOUBLE_EQ(actual[i], expected[i]) << i;
}

void expectValidPath(const RrtConnectPlanner& planner,
                     const std::vector<JointVector>& path,
                     double edge_resolution) {
  const auto validation = planner.validatePath(path, edge_resolution);
  EXPECT_TRUE(validation.valid) << validation.reason;
}

RrtConnectPlanner impossiblePlanner(const JointVector& start,
                                    const JointVector& goal,
                                    const JointBounds& bounds) {
  auto same = [](const JointVector& a, const JointVector& b) {
    return (a - b).cwiseAbs().maxCoeff() <= 1e-12;
  };
  return RrtConnectPlanner(
      bounds,
      [=](const JointVector& value) {
        if (same(value, start) || same(value, goal))
          return StateValidityResult::free();
        return StateValidityResult::invalid(ValidityCode::EnvironmentCollision,
                                            "synthetic blocked state");
      },
      [=](const JointVector& a, const JointVector& b, double) {
        EdgeValidityResult edge;
        edge.samples_checked = 2;
        edge.resolution = 0.01;
        if (same(a, b)) {
          edge.valid = true;
          edge.first_invalid = StateValidityResult::free();
        } else {
          edge.first_invalid = StateValidityResult::invalid(
              ValidityCode::EnvironmentCollision, "synthetic blocked edge");
        }
        return edge;
      });
}
}  // namespace

TEST(RrtConnect, InvalidStartRejected) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.55, 0.25), q(0.75, 0.44), benchmarkOptions());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::InvalidStart);
}

TEST(RrtConnect, InvalidGoalRejected) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.35, 0.06), q(0.55, 0.25), benchmarkOptions());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::InvalidGoal);
}

TEST(RrtConnect, StartEqualsGoalTrivialPath) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.35, 0.06);
  const auto result = planner.plan(start, start, benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.path().size(), 1u);
  expectExactConfig(result.path().front(), start);
  expectExactConfig(result.path().back(), start);
  EXPECT_TRUE(result.final_path_valid);
}

TEST(RrtConnect, DirectFreePathReturnsDirectPath) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.80, 0.06);
  const auto goal = q(0.90, 0.12);
  const auto result = planner.plan(start, goal, benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.direct_path);
  ASSERT_EQ(result.raw_path.size(), 2u);
  expectExactConfig(result.raw_path.front(), start);
  expectExactConfig(result.raw_path.back(), goal);
}

TEST(RrtConnect, DirectPathCollisionFindsAlternatePath) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  ASSERT_FALSE(c.isEdgeValid(start, goal, 0.01).valid);
  const auto result = planner.plan(start, goal, benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.direct_path);
  ASSERT_GE(result.raw_path.size(), 3u);
  ASSERT_GE(result.path().size(), 3u);
  expectExactConfig(result.path().front(), start);
  expectExactConfig(result.path().back(), goal);
  expectValidPath(planner, result.raw_path, benchmarkOptions().edge_resolution);
  expectValidPath(planner, result.path(), benchmarkOptions().edge_resolution);
}

TEST(RrtConnect, ImpossibleProblemReturnsFailure) {
  auto c = checker();
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  RrtConnectPlanner planner = impossiblePlanner(start, goal, c.bounds());
  auto options = benchmarkOptions();
  options.max_iterations = 30;
  const auto result = planner.plan(start, goal, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::IterationLimit);
}

TEST(RrtConnect, ReconstructedPathStartsAndEndsExactly) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  const auto result = planner.plan(start, goal, benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  expectExactConfig(result.raw_path.front(), start);
  expectExactConfig(result.raw_path.back(), goal);
  expectExactConfig(result.path().front(), start);
  expectExactConfig(result.path().back(), goal);
}

TEST(RrtConnect, EveryReturnedStateIsValid) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.35, 0.06), q(0.75, 0.44), benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  for (const auto& state : result.path()) {
    const auto validity = c.isStateValid(state);
    EXPECT_TRUE(validity.valid) << validity.reason;
  }
}

TEST(RrtConnect, EveryReturnedEdgeIsValid) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.35, 0.06), q(0.75, 0.44), benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  for (size_t i = 1; i < result.path().size(); ++i) {
    const auto edge = c.isEdgeValid(result.path()[i - 1], result.path()[i],
                                    benchmarkOptions().edge_resolution);
    EXPECT_TRUE(edge.valid) << edge.first_invalid.reason;
  }
}

TEST(RrtConnect, DeterministicSeedReproducesPath) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  const auto a = planner.plan(start, goal, benchmarkOptions());
  const auto b = planner.plan(start, goal, benchmarkOptions());
  ASSERT_TRUE(a.success) << a.message;
  ASSERT_TRUE(b.success) << b.message;
  ASSERT_EQ(a.raw_path.size(), b.raw_path.size());
  ASSERT_EQ(a.path().size(), b.path().size());
  for (size_t i = 0; i < a.raw_path.size(); ++i)
    expectExactConfig(a.raw_path[i], b.raw_path[i]);
  for (size_t i = 0; i < a.path().size(); ++i)
    expectExactConfig(a.path()[i], b.path()[i]);
}

TEST(RrtConnect, PlannerDiagnosticsArePopulated) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.35, 0.06), q(0.75, 0.44), benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_GT(result.iterations, 0);
  EXPECT_GT(result.sampled_states, 0);
  EXPECT_GT(result.tree_start_size, 1);
  EXPECT_GT(result.tree_goal_size, 1);
  EXPECT_GT(result.raw_path.size(), 0u);
  EXPECT_GT(result.simplified_path.size(), 0u);
  EXPECT_GE(result.planning_time_ms, 0.0);
}

TEST(RrtConnect, IterationLimitIsRespected) {
  auto c = checker();
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  RrtConnectPlanner planner = impossiblePlanner(start, goal, c.bounds());
  auto options = benchmarkOptions();
  options.max_iterations = 7;
  options.planning_timeout = 10.0;
  const auto result = planner.plan(start, goal, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::IterationLimit);
  EXPECT_LE(result.iterations, options.max_iterations);
}

TEST(RrtConnect, ZeroIterationBudgetFailsCleanly) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  auto options = benchmarkOptions();
  options.max_iterations = 0;
  options.planning_timeout = 10.0;
  const auto result = planner.plan(q(0.35, 0.06), q(0.75, 0.44), options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::IterationLimit);
  EXPECT_EQ(result.iterations, 0);
}

TEST(RrtConnect, TimeoutIsRespected) {
  auto c = checker();
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  RrtConnectPlanner planner = impossiblePlanner(start, goal, c.bounds());
  auto options = benchmarkOptions();
  options.max_iterations = 1000000;
  options.planning_timeout = 1e-9;
  const auto result = planner.plan(start, goal, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, PlanningFailure::Timeout);
}

TEST(RrtConnect, ShortcuttingPreservesValidityAndDoesNotGrowPath) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto result = planner.plan(q(0.35, 0.06), q(0.75, 0.44), benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.simplified_path.empty());
  EXPECT_LE(result.simplified_path.size(), result.raw_path.size());
  expectValidPath(planner, result.simplified_path, benchmarkOptions().edge_resolution);
}

TEST(RrtConnect, FreeEndpointsCollidingInteriorRemainsRejectedByEdgeChecker) {
  auto c = checker();
  RrtConnectPlanner planner(c);
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  ASSERT_TRUE(c.isStateValid(start).valid);
  ASSERT_TRUE(c.isStateValid(goal).valid);
  EXPECT_FALSE(c.isEdgeValid(start, goal, 0.01).valid);
  const auto result = planner.plan(start, goal, benchmarkOptions());
  ASSERT_TRUE(result.success) << result.message;
  for (size_t i = 1; i < result.path().size(); ++i) {
    const auto edge = c.isEdgeValid(result.path()[i - 1], result.path()[i], 0.01);
    EXPECT_TRUE(edge.valid) << i << ": " << edge.first_invalid.reason;
  }
}
