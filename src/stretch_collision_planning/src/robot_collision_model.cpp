#include "stretch_collision_planning/robot_collision_model.hpp"

#include <cmath>
#include <stdexcept>

namespace stretch_collision_planning {

PrimitiveRobotCollisionModel::PrimitiveRobotCollisionModel(
    std::vector<CapsuleSpec> specs)
    : specs_(std::move(specs)) {
  if (specs_.empty())
    throw std::invalid_argument("Robot collision model must contain geometry");
  for (const auto& spec : specs_) {
    if (spec.id.empty() || spec.link_a.empty() || spec.link_b.empty() ||
        !std::isfinite(spec.radius) || spec.radius <= 0.0)
      throw std::invalid_argument("Invalid capsule specification");
  }
}

std::vector<CollisionCapsule> PrimitiveRobotCollisionModel::capsules(
    const LinkTransforms& transforms) const {
  std::vector<CollisionCapsule> result;
  result.reserve(specs_.size());
  for (const auto& spec : specs_) {
    const auto a = transforms.find(spec.link_a);
    const auto b = transforms.find(spec.link_b);
    if (a == transforms.end() || b == transforms.end())
      throw std::invalid_argument("Collision model references missing link");
    CollisionCapsule capsule;
    capsule.id = spec.id;
    capsule.link_a = spec.link_a;
    capsule.link_b = spec.link_b;
    capsule.a = a->second.translation();
    capsule.b = b->second.translation();
    capsule.radius = spec.radius;
    result.push_back(capsule);
  }
  return result;
}

PrimitiveRobotCollisionModel
PrimitiveRobotCollisionModel::makeConservativeStretchArm() {
  return PrimitiveRobotCollisionModel({
      {"lift_to_shoulder", "link_lift", "link_arm_l4", 0.035},
      {"arm_l4_l3", "link_arm_l4", "link_arm_l3", 0.040},
      {"arm_l3_l2", "link_arm_l3", "link_arm_l2", 0.040},
      {"arm_l2_l1", "link_arm_l2", "link_arm_l1", 0.040},
      {"arm_l1_l0", "link_arm_l1", "link_arm_l0", 0.040},
      {"arm_to_wrist", "link_arm_l0", "link_wrist_yaw", 0.045},
      {"wrist_yaw_pitch", "link_wrist_yaw", "link_wrist_pitch", 0.035},
      {"wrist_pitch_roll", "link_wrist_pitch", "link_wrist_roll", 0.035},
      {"wrist_to_gripper", "link_wrist_roll", "link_gripper_s3_body", 0.035},
      {"gripper_body", "link_gripper_s3_body", "link_grasp_center", 0.025},
  });
}

}  // namespace stretch_collision_planning
