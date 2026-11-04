#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <octomap_msgs/conversions.h>

#include "stretch_collision_planning/octomap_environment.hpp"
#include "stretch_collision_planning/octomap_scene_store.hpp"
#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/rrt_connect.hpp"
#include "stretch_collision_planning/state_validity.hpp"

using namespace stretch_collision_planning;
using namespace std::chrono_literals;

namespace {
std::shared_ptr<octomap::OcTree> tree(double resolution = 0.05) {
  return std::make_shared<octomap::OcTree>(resolution);
}

Eigen::Vector3d p(double x, double y, double z) {
  return Eigen::Vector3d(x, y, z);
}

CollisionCapsule capsule(Eigen::Vector3d a, Eigen::Vector3d b,
                         double radius = 0.02) {
  CollisionCapsule c;
  c.id = "test_capsule";
  c.a = std::move(a);
  c.b = std::move(b);
  c.radius = radius;
  return c;
}

double centerStart(double min_value, double resolution) {
  return std::floor((min_value - 0.5 * resolution) / resolution) * resolution +
         0.5 * resolution;
}

bool boxesOverlap(const AxisAlignedBox& a, const AxisAlignedBox& b) {
  return (a.min().array() <= b.max().array()).all() &&
         (a.max().array() >= b.min().array()).all();
}

void addBox(octomap::OcTree& map, const AxisAlignedBox& box) {
  const double res = map.getResolution();
  for (double x = centerStart(box.min().x(), res);
       x <= box.max().x() + 0.5 * res + 1e-12;
       x += res) {
    for (double y = centerStart(box.min().y(), res);
         y <= box.max().y() + 0.5 * res + 1e-12;
         y += res) {
      for (double z = centerStart(box.min().z(), res);
           z <= box.max().z() + 0.5 * res + 1e-12;
           z += res) {
        AxisAlignedBox voxel{"fixture_voxel", p(x, y, z),
                             Eigen::Vector3d::Constant(res)};
        if (!boxesOverlap(voxel, box)) continue;
        map.updateNode(octomap::point3d(x, y, z), true);
      }
    }
  }
  map.updateInnerOccupancy();
}

std::shared_ptr<octomap::OcTree> deskFixture(double resolution = 0.04) {
  auto map = tree(resolution);
  const auto primitive = PrimitiveEnvironment::makeDeskBenchmark();
  for (const auto& box : primitive.boxes()) addBox(*map, box);
  return map;
}

std::shared_ptr<octomap::OcTree> observedDeskFixture(
    double resolution = 0.02) {
  auto map = tree(resolution);
  const auto first_center = [resolution](double value) {
    return std::floor(value / resolution) * resolution + 0.5 * resolution;
  };
  for (double x = first_center(-0.50); x <= 0.30; x += resolution) {
    for (double y = first_center(-0.85); y <= 0.20; y += resolution) {
      for (double z = first_center(0.05); z <= 1.35; z += resolution) {
        map->updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }
  for (const auto& box : PrimitiveEnvironment::makeDeskBenchmark().boxes())
    addBox(*map, box);
  map->updateInnerOccupancy();
  return map;
}

std::shared_ptr<RobotModel> model() {
  std::ifstream file(TEST_URDF);
  std::stringstream xml;
  xml << file.rdbuf();
  return std::make_shared<RobotModel>(xml.str());
}

JointVector q(double lift, double arm, double yaw = 0.0, double pitch = -0.6,
              double roll = 0.0) {
  JointVector value;
  value << lift, arm, yaw, pitch, roll;
  return value;
}

octomap_msgs::msg::Octomap message(const octomap::OcTree& map,
                                   const std::string& frame = "base_link") {
  octomap_msgs::msg::Octomap result;
  EXPECT_TRUE(octomap_msgs::binaryMapToMsg(map, result));
  result.header.frame_id = frame;
  return result;
}
}  // namespace

TEST(CollisionEnvironmentSelection, ParsesBothBackendsAndRejectsTypos) {
  EXPECT_EQ(parseCollisionEnvironmentBackend("primitive"),
            CollisionEnvironmentBackend::Primitive);
  EXPECT_EQ(parseCollisionEnvironmentBackend("OCTOMAP"),
            CollisionEnvironmentBackend::OctoMap);
  EXPECT_THROW(parseCollisionEnvironmentBackend("automatic"),
               std::invalid_argument);
}

TEST(OctoMapSceneStore, UnavailableStaleAndFrameMismatchAreExplicit) {
  OctoMapEnvironmentOptions options;
  options.freshness_timeout = 1.0;
  OctoMapSceneStore store(options, "base_link");
  const auto now = OctoMapEnvironment::Clock::now();
  EXPECT_EQ(store.snapshot(now).status, SceneFreshness::Unavailable);

  auto map = tree();
  map->updateNode(octomap::point3d(0.1, 0.1, 0.1), false);
  map->updateInnerOccupancy();
  EXPECT_FALSE(store.update(message(*map, "odom"), now));
  EXPECT_EQ(store.snapshot(now).status, SceneFreshness::Unavailable);
  EXPECT_NE(store.lastError().find("does not match"), std::string::npos);

  ASSERT_TRUE(store.update(message(*map), now - 2s));
  EXPECT_EQ(store.snapshot(now).status, SceneFreshness::Stale);
}

TEST(OctoMapSceneStore, PlanningSnapshotIsImmutableAcrossMapUpdates) {
  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapSceneStore store(options, "base_link");
  const auto now = OctoMapEnvironment::Clock::now();
  const auto query = p(0.125, 0.125, 0.125);

  auto first_map = tree();
  first_map->updateNode(octomap::point3d(query.x(), query.y(), query.z()), false);
  first_map->updateInnerOccupancy();
  ASSERT_TRUE(store.update(message(*first_map), now));
  const auto first = store.snapshot(now);
  ASSERT_TRUE(first.available());
  EXPECT_EQ(first.environment->occupancyAt(query), OccupancyStatus::Free);
  const auto probe = capsule(query - p(0.02, 0.0, 0.0),
                             query + p(0.02, 0.0, 0.0), 0.005);
  EXPECT_FALSE(first.environment->collides(probe));

  auto second_map = tree();
  second_map->updateNode(octomap::point3d(query.x(), query.y(), query.z()), true);
  second_map->updateInnerOccupancy();
  ASSERT_TRUE(store.update(message(*second_map), now + 10ms));
  const auto second = store.snapshot(now + 10ms);
  ASSERT_TRUE(second.available());
  EXPECT_GT(second.id, first.id);
  EXPECT_EQ(second.environment->occupancyAt(query), OccupancyStatus::Occupied);
  EXPECT_TRUE(second.environment->collides(probe));
  EXPECT_EQ(first.environment->occupancyAt(query), OccupancyStatus::Free);
  EXPECT_FALSE(first.environment->collides(probe));
}

TEST(OctoMapSceneStore, InvalidNewMapCannotLeaveOldSceneExecutable) {
  OctoMapSceneStore store({}, "base_link");
  const auto now = OctoMapEnvironment::Clock::now();
  auto map = tree();
  map->updateNode(octomap::point3d(0.1, 0.1, 0.1), false);
  map->updateInnerOccupancy();
  ASSERT_TRUE(store.update(message(*map), now));
  ASSERT_TRUE(store.snapshot(now).available());

  EXPECT_FALSE(store.update(message(*map, "odom"), now));
  const auto rejected = store.snapshot(now);
  EXPECT_EQ(rejected.status, SceneFreshness::Unavailable);
  EXPECT_FALSE(rejected.environment);
  EXPECT_NE(rejected.reason.find("does not match"), std::string::npos);
}

TEST(OctoMapEnvironment, OccupiedVoxelQuery) {
  auto map = tree();
  map->updateNode(octomap::point3d(0.425, -0.175, 0.525), true);
  map->updateInnerOccupancy();

  OctoMapEnvironment env;
  ASSERT_TRUE(env.setMap(map, "base_link"));
  EXPECT_EQ(env.occupancyAt(p(0.425, -0.175, 0.525)),
            OccupancyStatus::Occupied);
  EXPECT_TRUE(env.isOccupied(p(0.425, -0.175, 0.525)));
}

TEST(OctoMapEnvironment, FreeVoxelQuery) {
  auto map = tree();
  map->updateNode(octomap::point3d(0.15, 0.10, 0.25), false);
  map->updateInnerOccupancy();

  OctoMapEnvironment env;
  ASSERT_TRUE(env.setMap(map, "base_link"));
  EXPECT_EQ(env.occupancyAt(p(0.15, 0.10, 0.25)), OccupancyStatus::Free);
  EXPECT_FALSE(env.isOccupied(p(0.15, 0.10, 0.25)));
}

TEST(OctoMapEnvironment, UnknownVoxelPolicyIsExplicit) {
  auto map = tree();
  OctoMapEnvironment fail_safe;
  ASSERT_TRUE(fail_safe.setMap(map, "base_link"));
  EXPECT_EQ(fail_safe.occupancyAt(p(1.0, 1.0, 1.0)), OccupancyStatus::Unknown);
  EXPECT_TRUE(fail_safe.isOccupied(p(1.0, 1.0, 1.0)));
  CollisionContact contact;
  EXPECT_TRUE(fail_safe.collides(
      capsule(p(0.95, 1.0, 1.0), p(1.05, 1.0, 1.0)), &contact));
  EXPECT_EQ(contact.kind, CollisionContactKind::Unknown);
  ASSERT_TRUE(contact.robot_capsule.has_value());
  ASSERT_TRUE(contact.environment_box.has_value());
  EXPECT_NE(contact.environment_geometry.find("octomap_unknown"),
            std::string::npos);

  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment permissive(options);
  ASSERT_TRUE(permissive.setMap(map, "base_link"));
  EXPECT_FALSE(permissive.isOccupied(p(1.0, 1.0, 1.0)));
}

TEST(OctoMapEnvironment, CollisionPrimitiveHitsOccupiedVoxelVolume) {
  auto map = tree();
  map->updateNode(octomap::point3d(0.50, 0.0, 0.20), true);
  map->updateInnerOccupancy();

  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment env(options);
  ASSERT_TRUE(env.setMap(map, "base_link"));

  CollisionContact contact;
  EXPECT_TRUE(env.collides(capsule(p(0.35, 0.0, 0.20), p(0.65, 0.0, 0.20)),
                           &contact));
  EXPECT_EQ(contact.robot_geometry, "test_capsule");
  EXPECT_NE(contact.environment_geometry.find("octomap_occupied"),
            std::string::npos);
  EXPECT_EQ(contact.kind, CollisionContactKind::Occupied);
  ASSERT_TRUE(contact.robot_capsule.has_value());
  ASSERT_TRUE(contact.environment_box.has_value());
  EXPECT_EQ(contact.robot_capsule->id, "test_capsule");
  EXPECT_DOUBLE_EQ(contact.environment_box->size.x(), map->getResolution());
}

TEST(OctoMapEnvironment, NonCollisionPrimitiveMissesOccupiedVoxel) {
  auto map = tree();
  map->updateNode(octomap::point3d(0.50, 0.0, 0.20), true);
  map->updateInnerOccupancy();

  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment env(options);
  ASSERT_TRUE(env.setMap(map, "base_link"));

  EXPECT_FALSE(env.collides(capsule(p(0.35, 0.40, 0.20),
                                    p(0.65, 0.40, 0.20))));
}

TEST(OctoMapEnvironment, SceneUnavailableAndStaleAreRepresented) {
  OctoMapEnvironmentOptions options;
  options.freshness_timeout = 1.0;
  OctoMapEnvironment env(options);
  const auto now = OctoMapEnvironment::Clock::now();
  EXPECT_EQ(env.freshness(now), SceneFreshness::Unavailable);

  ASSERT_TRUE(env.setMap(tree(), "base_link", now - 2s));
  EXPECT_EQ(env.freshness(now), SceneFreshness::Stale);
}

TEST(OctoMapEnvironment, SceneAvailableAndMapFrameAreStored) {
  OctoMapEnvironmentOptions options;
  options.freshness_timeout = 1.0;
  OctoMapEnvironment env(options);
  const auto now = OctoMapEnvironment::Clock::now();
  ASSERT_TRUE(env.setMap(tree(0.04), "odom", now - 100ms));
  EXPECT_EQ(env.freshness(now), SceneFreshness::Available);
  EXPECT_EQ(env.mapFrame(), "odom");
  EXPECT_DOUBLE_EQ(env.resolution(), 0.04);
}

TEST(OctoMapEnvironment, DeterministicDeskFixtureContainsTabletop) {
  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment env(options);
  ASSERT_TRUE(env.setMap(deskFixture(), "base_link"));

  EXPECT_EQ(env.occupancyAt(p(-0.11, -0.40, 0.60)),
            OccupancyStatus::Occupied);
  EXPECT_FALSE(env.collides(capsule(p(0.40, 0.30, 0.80),
                                    p(0.60, 0.30, 0.80))));
}

TEST(OctoMapEnvironment, PrimitiveEnvironmentStillWorks) {
  const auto primitive = PrimitiveEnvironment::makeDeskBenchmark();
  EXPECT_GE(primitive.boxes().size(), 4u);
  EXPECT_TRUE(primitive.collides(capsule(p(-0.20, -0.40, 0.60),
                                         p(0.00, -0.40, 0.60), 0.03)));
  EXPECT_FALSE(primitive.collides(capsule(p(0.40, 0.30, 0.80),
                                          p(0.60, 0.30, 0.80), 0.03)));
}

TEST(OctoMapEnvironment, CrossValidatesAgainstPrimitiveDeskFixture) {
  auto robot = std::make_shared<PrimitiveRobotCollisionModel>(
      PrimitiveRobotCollisionModel::makeConservativeStretchArm());
  auto primitive_scene = std::make_shared<PrimitiveEnvironment>(
      PrimitiveEnvironment::makeDeskBenchmark());

  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  auto octomap_scene = std::make_shared<OctoMapEnvironment>(options);
  ASSERT_TRUE(octomap_scene->setMap(deskFixture(0.02), "base_link"));

  StateValidityChecker primitive_checker(model(), JointBounds::load(TEST_LIMITS),
                                         robot, primitive_scene);
  StateValidityChecker octomap_checker(model(), JointBounds::load(TEST_LIMITS),
                                       robot, octomap_scene);

  int total = 0;
  int agreements = 0;
  int false_free = 0;
  int false_collision = 0;

  for (double lift : {0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90}) {
    for (double arm : {0.04, 0.10, 0.16, 0.22, 0.28, 0.34, 0.40, 0.46}) {
      const auto value = q(lift, arm);
      const bool primitive_free = primitive_checker.isStateValid(value).valid;
      const bool octomap_free = octomap_checker.isStateValid(value).valid;
      ++total;
      if (primitive_free == octomap_free) {
        ++agreements;
      } else if (!primitive_free && octomap_free) {
        ++false_free;
      } else {
        ++false_collision;
      }
    }
  }

  const double agreement_rate =
      static_cast<double>(agreements) / static_cast<double>(total);
  std::cout << "Cross-validation PrimitiveEnvironment vs OctoMap fixture: "
            << "total=" << total << " agreement_rate=" << agreement_rate
            << " false_free=" << false_free
            << " false_collision=" << false_collision << std::endl;

  EXPECT_EQ(false_free, 0) << "primitive collision but OctoMap fixture free";
  EXPECT_GE(agreement_rate, 0.85) << "false_collision=" << false_collision;
}

TEST(StateValidityChecker, ReportsCollisionQueryStatistics) {
  auto robot = std::make_shared<PrimitiveRobotCollisionModel>(
      PrimitiveRobotCollisionModel::makeConservativeStretchArm());
  auto primitive_scene = std::make_shared<PrimitiveEnvironment>();
  StateValidityChecker checker(model(), JointBounds::load(TEST_LIMITS), robot,
                               primitive_scene);
  checker.resetStatistics();
  EXPECT_TRUE(checker.isStateValid(q(0.8, 0.06)).valid);
  EXPECT_TRUE(checker.isEdgeValid(q(0.8, 0.06), q(0.85, 0.08), 0.02).valid);
  const auto statistics = checker.statistics();
  EXPECT_GE(statistics.state_checks, 2u);
  EXPECT_EQ(statistics.edge_checks, 1u);
  EXPECT_GT(statistics.environment_queries, 0u);
  EXPECT_GT(statistics.edge_samples, 0u);
  EXPECT_GE(statistics.collision_checking_time_ms, 0.0);
}

TEST(OctoMapPlanning, DirectPathSkipsRrt) {
  auto scene = std::make_shared<OctoMapEnvironment>();
  ASSERT_TRUE(scene->setMap(observedDeskFixture(), "base_link"));
  ASSERT_EQ(scene->occupancyAt(p(-0.13, -0.17, 0.95)),
            OccupancyStatus::Free);
  StateValidityChecker checker(
      model(), JointBounds::load(TEST_LIMITS),
      std::make_shared<PrimitiveRobotCollisionModel>(
          PrimitiveRobotCollisionModel::makeConservativeStretchArm()),
      scene);
  RrtConnectPlanner planner(checker);
  PlannerOptions options;
  options.edge_resolution = 0.01;
  checker.resetStatistics();
  const auto result = planner.plan(q(0.80, 0.06), q(0.90, 0.12), options);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.direct_path);
  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(result.final_path_valid);
  const auto statistics = checker.statistics();
  std::cout << "OctoMap direct benchmark: planning_time_ms="
            << result.planning_time_ms
            << " state_checks=" << statistics.state_checks
            << " edge_checks=" << statistics.edge_checks
            << " environment_queries=" << statistics.environment_queries
            << std::endl;
}

TEST(OctoMapPlanning, ObstructedPathUsesRrtAndRemainsValid) {
  auto scene = std::make_shared<OctoMapEnvironment>();
  ASSERT_TRUE(scene->setMap(observedDeskFixture(), "base_link"));
  StateValidityChecker checker(
      model(), JointBounds::load(TEST_LIMITS),
      std::make_shared<PrimitiveRobotCollisionModel>(
          PrimitiveRobotCollisionModel::makeConservativeStretchArm()),
      scene);
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.75, 0.44);
  const auto start_state = checker.isStateValid(start);
  const auto goal_state = checker.isStateValid(goal);
  ASSERT_TRUE(start_state.valid) << start_state.reason;
  ASSERT_TRUE(goal_state.valid) << goal_state.reason;
  ASSERT_FALSE(checker.isEdgeValid(start, goal, 0.01).valid);

  PlannerOptions options;
  options.max_iterations = 4000;
  options.planning_timeout = 3.0;
  options.max_extension_distance = 0.12;
  options.edge_resolution = 0.01;
  options.goal_bias = 0.10;
  options.random_seed = 29;
  options.shortcut_attempts = 200;
  RrtConnectPlanner planner(checker);
  checker.resetStatistics();
  const auto result = planner.plan(start, goal, options);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.direct_path);
  EXPECT_GT(result.iterations, 0);
  EXPECT_TRUE(result.final_path_valid);
  EXPECT_TRUE(planner.validatePath(result.path(), options.edge_resolution).valid);
  const auto statistics = checker.statistics();
  std::cout << "OctoMap obstructed benchmark: planning_time_ms="
            << result.planning_time_ms << " iterations=" << result.iterations
            << " state_checks=" << statistics.state_checks
            << " edge_checks=" << statistics.edge_checks
            << " environment_queries=" << statistics.environment_queries
            << " raw_waypoints=" << result.raw_path.size()
            << " simplified_waypoints=" << result.path().size() << std::endl;
}

TEST(OctoMapPlanning, TrappedEndpointsProduceNoPath) {
  const auto bounds = JointBounds::load(TEST_LIMITS);
  auto robot = std::make_shared<PrimitiveRobotCollisionModel>(
      PrimitiveRobotCollisionModel::makeConservativeStretchArm());
  const auto start = q(0.35, 0.06);
  const auto goal = q(0.90, 0.44, -0.8, -0.2, 0.7);
  StateValidityChecker transforms(
      model(), bounds, robot, std::make_shared<PrimitiveEnvironment>());
  const double resolution = 0.04;
  auto map = tree(resolution);
  const auto first_center = [resolution](double value) {
    return std::floor(value / resolution) * resolution + 0.5 * resolution;
  };
  for (double x = first_center(-0.8); x <= 0.8; x += resolution) {
    for (double y = first_center(-1.0); y <= 0.5; y += resolution) {
      for (double z = first_center(-0.1); z <= 1.6; z += resolution)
        map->updateNode(octomap::point3d(x, y, z), true);
    }
  }
  for (const auto& value : {start, goal}) {
    for (const auto& capsule : robot->capsules(transforms.linkTransforms(value))) {
      const Eigen::Vector3d padding = Eigen::Vector3d::Constant(
          capsule.radius + 2.0 * resolution);
      const Eigen::Vector3d min = capsule.a.cwiseMin(capsule.b) - padding;
      const Eigen::Vector3d max = capsule.a.cwiseMax(capsule.b) + padding;
      for (double x = first_center(min.x()); x <= max.x(); x += resolution) {
        for (double y = first_center(min.y()); y <= max.y(); y += resolution) {
          for (double z = first_center(min.z()); z <= max.z(); z += resolution) {
            const octomap::point3d point(x, y, z);
            map->setNodeValue(map->coordToKey(point),
                              map->getClampingThresMinLog());
          }
        }
      }
    }
  }
  map->updateInnerOccupancy();
  auto scene = std::make_shared<OctoMapEnvironment>();
  ASSERT_TRUE(scene->setMap(map, "base_link"));
  StateValidityChecker checker(model(), bounds, robot, scene);
  const auto start_state = checker.isStateValid(start);
  const auto goal_state = checker.isStateValid(goal);
  ASSERT_TRUE(start_state.valid) << start_state.reason;
  ASSERT_TRUE(goal_state.valid) << goal_state.reason;
  ASSERT_FALSE(checker.isEdgeValid(start, goal, 0.01).valid);

  PlannerOptions options;
  options.max_iterations = 100;
  options.planning_timeout = 3.0;
  options.edge_resolution = 0.01;
  options.random_seed = 29;
  RrtConnectPlanner planner(checker);
  checker.resetStatistics();
  const auto result = planner.plan(start, goal, options);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.failure == PlanningFailure::IterationLimit ||
              result.failure == PlanningFailure::Timeout);
  const auto statistics = checker.statistics();
  std::cout << "OctoMap no-path benchmark: planning_time_ms="
            << result.planning_time_ms << " iterations=" << result.iterations
            << " state_checks=" << statistics.state_checks
            << " edge_checks=" << statistics.edge_checks
            << " environment_queries=" << statistics.environment_queries
            << std::endl;
}

TEST(OctoMapEnvironment, OdomMapIsQueriedThroughBasePose) {
  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment environment(options);
  auto map = tree();
  // Obstacle at odom (2.0, 1.0, 0.5); the base sits at odom (1.0, 1.0).
  map->updateNode(octomap::point3d(2.025, 1.025, 0.525), true);
  map->updateInnerOccupancy();
  ASSERT_TRUE(environment.setMap(map, "odom"));
  const auto base_capsule = capsule(p(1.0, 0.0, 0.5), p(1.05, 0.05, 0.55));
  EXPECT_FALSE(environment.collides(base_capsule));
  Eigen::Isometry3d odom_from_base = Eigen::Isometry3d::Identity();
  odom_from_base.translation() = p(1.0, 1.0, 0.0);
  environment.setMapFromBase(odom_from_base);
  EXPECT_TRUE(environment.collides(base_capsule));
}

TEST(OctoMapEnvironment, VoxelsInsideCurrentBodyAreIgnored) {
  OctoMapEnvironmentOptions options;
  options.unknown_policy = UnknownSpacePolicy::UnknownIsFree;
  OctoMapEnvironment environment(options);
  auto map = tree();
  map->updateNode(octomap::point3d(0.525, 0.025, 0.525), true);  // the arm
  map->updateNode(octomap::point3d(1.025, 0.025, 0.525), true);  // a box
  map->updateInnerOccupancy();
  ASSERT_TRUE(environment.setMap(map, "base_link"));
  const auto through_arm = capsule(p(0.45, 0.0, 0.5), p(0.6, 0.05, 0.55));
  const auto through_box = capsule(p(0.95, 0.0, 0.5), p(1.1, 0.05, 0.55));
  EXPECT_TRUE(environment.collides(through_arm));
  environment.setIgnoredCapsules({capsule(p(0.5, 0.0, 0.5), p(0.55, 0.05, 0.55), 0.06)});
  EXPECT_FALSE(environment.collides(through_arm));
  EXPECT_TRUE(environment.collides(through_box));
}
