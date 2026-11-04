#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_custom_kinematics/inverse_kinematics.hpp"

namespace stretch_collision_planning {

using JointVector = stretch_custom_kinematics::JointVector;
using JointBounds = stretch_custom_kinematics::JointBounds;
using RobotModel = stretch_robot_interface::RobotModel;

enum class ValidityCode {
  Free,
  NonFinite,
  JointLimit,
  KinematicsFailure,
  EnvironmentCollision,
  SelfCollision
};

const char* validityCodeName(ValidityCode code);

struct StateValidityResult {
  bool valid = false;
  ValidityCode code = ValidityCode::KinematicsFailure;
  std::string reason;
  std::optional<CollisionContact> contact;

  static StateValidityResult free();
  static StateValidityResult invalid(ValidityCode code, std::string reason);
};

struct EdgeValidityResult {
  bool valid = false;
  int samples_checked = 0;
  double resolution = 0.0;
  StateValidityResult first_invalid;
};

struct CollisionCheckStatistics {
  uint64_t state_checks = 0;
  uint64_t edge_checks = 0;
  uint64_t environment_queries = 0;
  uint64_t edge_samples = 0;
  double collision_checking_time_ms = 0.0;
};

class StateValidityChecker {
 public:
  StateValidityChecker(std::shared_ptr<const RobotModel> model,
                       const JointBounds& bounds,
                       std::shared_ptr<const RobotCollisionModel> robot,
                       std::shared_ptr<const CollisionEnvironment> environment);

  StateValidityResult isStateValid(const JointVector& q) const;
  EdgeValidityResult isEdgeValid(const JointVector& a, const JointVector& b,
                                 double resolution) const;
  LinkTransforms linkTransforms(const JointVector& q) const;

  CollisionCheckStatistics statistics() const { return statistics_; }
  void resetStatistics() const { statistics_ = {}; }

  const JointBounds& bounds() const { return bounds_; }

 private:
  stretch_robot_interface::RobotState stateFromVector(const JointVector& q) const;

  std::shared_ptr<const RobotModel> model_;
  JointBounds bounds_;
  std::shared_ptr<const RobotCollisionModel> robot_;
  std::shared_ptr<const CollisionEnvironment> environment_;
  mutable CollisionCheckStatistics statistics_;
};

}  // namespace stretch_collision_planning
