#include "stretch_collision_planning/state_validity.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace stretch_collision_planning {
namespace {
double maxNormalizedDelta(const JointVector& a, const JointVector& b,
                          const JointBounds& bounds) {
  double value = 0.0;
  for (int i = 0; i < 5; ++i) {
    const double span = bounds.upper[i] - bounds.lower[i];
    if (span <= 0.0 || !std::isfinite(span))
      throw std::invalid_argument("Invalid bounds span");
    value = std::max(value, std::abs(a[i] - b[i]) / span);
  }
  return value;
}
}  // namespace

const char* validityCodeName(ValidityCode code) {
  switch (code) {
    case ValidityCode::Free:
      return "FREE";
    case ValidityCode::NonFinite:
      return "NON_FINITE";
    case ValidityCode::JointLimit:
      return "JOINT_LIMIT";
    case ValidityCode::KinematicsFailure:
      return "KINEMATICS_FAILURE";
    case ValidityCode::EnvironmentCollision:
      return "ENVIRONMENT_COLLISION";
    case ValidityCode::SelfCollision:
      return "SELF_COLLISION";
  }
  return "UNKNOWN";
}

StateValidityResult StateValidityResult::free() {
  StateValidityResult r;
  r.valid = true;
  r.code = ValidityCode::Free;
  r.reason = validityCodeName(ValidityCode::Free);
  return r;
}

StateValidityResult StateValidityResult::invalid(ValidityCode code,
                                                 std::string reason) {
  StateValidityResult r;
  r.valid = false;
  r.code = code;
  r.reason = std::move(reason);
  return r;
}

StateValidityChecker::StateValidityChecker(
    std::shared_ptr<const RobotModel> model, const JointBounds& bounds,
    std::shared_ptr<const RobotCollisionModel> robot,
    std::shared_ptr<const CollisionEnvironment> environment)
    : model_(std::move(model)),
      bounds_(model_ ? bounds.intersectModel(*model_) : bounds),
      robot_(std::move(robot)),
      environment_(std::move(environment)) {
  if (!model_) throw std::invalid_argument("State validity requires a model");
  if (!robot_) throw std::invalid_argument("State validity requires robot geometry");
  if (!environment_)
    throw std::invalid_argument("State validity requires an environment");
}

stretch_robot_interface::RobotState StateValidityChecker::stateFromVector(
    const JointVector& q) const {
  stretch_robot_interface::RobotState state;
  state.valid = true;
  state.lift = q[0];
  state.arm_extension = q[1];
  state.wrist_yaw = q[2];
  state.wrist_pitch = q[3];
  state.wrist_roll = q[4];
  return state;
}

LinkTransforms StateValidityChecker::linkTransforms(const JointVector& q) const {
  if (!bounds_.contains(q))
    throw std::invalid_argument("Cannot compute transforms outside bounds");

  const auto physical = model_->physicalJoints(stateFromVector(q));
  LinkTransforms transforms;
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  transforms[model_->base()] = current;

  for (const auto& joint : model_->chain()) {
    current = current * RobotModel::origin(joint->parent_to_joint_origin_transform);
    if (joint->type != urdf::Joint::FIXED) {
      const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
      Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
      if (joint->type == urdf::Joint::PRISMATIC)
        motion.translation() = axis * physical.at(joint->name);
      else
        motion.linear() =
            Eigen::AngleAxisd(physical.at(joint->name), axis).toRotationMatrix();
      current = current * motion;
    }
    transforms[joint->child_link_name] = current;
  }
  return transforms;
}

StateValidityResult StateValidityChecker::isStateValid(
    const JointVector& q) const {
  ++statistics_.state_checks;
  if (!q.allFinite())
    return StateValidityResult::invalid(ValidityCode::NonFinite,
                                        "configuration contains non-finite value");
  if (!bounds_.contains(q))
    return StateValidityResult::invalid(ValidityCode::JointLimit,
                                        "configuration outside joint limits");

  std::vector<CollisionCapsule> capsules;
  try {
    capsules = robot_->capsules(linkTransforms(q));
  } catch (const std::exception& e) {
    return StateValidityResult::invalid(ValidityCode::KinematicsFailure, e.what());
  }

  for (const auto& capsule : capsules) {
    if (!validCapsule(capsule)) {
      return StateValidityResult::invalid(ValidityCode::KinematicsFailure,
                                          "invalid robot collision geometry");
    }
    CollisionContact contact;
    const auto collision_begin = std::chrono::steady_clock::now();
    ++statistics_.environment_queries;
    const bool collides = environment_->collides(capsule, &contact);
    statistics_.collision_checking_time_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - collision_begin)
            .count();
    if (collides) {
      auto result = StateValidityResult::invalid(
          ValidityCode::EnvironmentCollision,
          "robot geometry '" + contact.robot_geometry +
              "' collides with environment geometry '" +
              contact.environment_geometry + "'");
      result.contact = contact;
      return result;
    }
  }

  for (const auto& pair : robot_->selfCollisionPairs()) {
    if (pair.first >= capsules.size() || pair.second >= capsules.size())
      return StateValidityResult::invalid(ValidityCode::KinematicsFailure,
                                          "invalid self-collision pair");
    const auto& a = capsules[pair.first];
    const auto& b = capsules[pair.second];
    const double threshold = a.radius + b.radius;
    if (segmentSegmentDistanceSquared(a.a, a.b, b.a, b.b) <=
        threshold * threshold) {
      return StateValidityResult::invalid(
          ValidityCode::SelfCollision,
          "robot geometry '" + a.id + "' collides with robot geometry '" +
              b.id + "'");
    }
  }

  return StateValidityResult::free();
}

EdgeValidityResult StateValidityChecker::isEdgeValid(const JointVector& a,
                                                     const JointVector& b,
                                                     double resolution) const {
  EdgeValidityResult edge;
  ++statistics_.edge_checks;
  edge.resolution = resolution;
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    edge.first_invalid =
        StateValidityResult::invalid(ValidityCode::KinematicsFailure,
                                     "edge resolution must be positive");
    return edge;
  }

  int steps = 1;
  try {
    steps = std::max(1, static_cast<int>(
                            std::ceil(maxNormalizedDelta(a, b, bounds_) /
                                      resolution)));
  } catch (const std::exception& e) {
    edge.first_invalid =
        StateValidityResult::invalid(ValidityCode::KinematicsFailure, e.what());
    return edge;
  }

  for (int i = 0; i <= steps; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(steps);
    const JointVector q = (1.0 - u) * a + u * b;
    auto state = isStateValid(q);
    ++edge.samples_checked;
    ++statistics_.edge_samples;
    if (!state.valid) {
      edge.first_invalid = state;
      return edge;
    }
  }
  edge.valid = true;
  edge.first_invalid = StateValidityResult::free();
  return edge;
}

}  // namespace stretch_collision_planning
