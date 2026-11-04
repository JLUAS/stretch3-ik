#include "stretch_collision_planning/primitive_environment.hpp"

#include <stdexcept>

namespace stretch_collision_planning {

void PrimitiveEnvironment::addBox(const AxisAlignedBox& box) {
  if (!validBox(box)) throw std::invalid_argument("Invalid environment box");
  boxes_.push_back(box);
}

bool PrimitiveEnvironment::collides(const CollisionCapsule& capsule,
                                    CollisionContact* contact) const {
  for (const auto& box : boxes_) {
    if (capsuleIntersectsBox(capsule, box)) {
      if (contact) {
        contact->robot_geometry = capsule.id;
        contact->environment_geometry = box.id;
        contact->kind = CollisionContactKind::Occupied;
        contact->robot_capsule = capsule;
        contact->environment_box = box;
      }
      return true;
    }
  }
  return false;
}

PrimitiveEnvironment PrimitiveEnvironment::makeDeskBenchmark() {
  PrimitiveEnvironment scene;
  scene.addBox({"floor", Eigen::Vector3d(0.0, 0.0, -0.055),
                Eigen::Vector3d(2.5, 2.5, 0.10)});
  scene.addBox({"desk_tabletop", Eigen::Vector3d(-0.11, -0.40, 0.60),
                Eigen::Vector3d(0.42, 0.24, 0.10)});
  scene.addBox({"desk_left_leg", Eigen::Vector3d(-0.27, -0.50, 0.28),
                Eigen::Vector3d(0.06, 0.06, 0.56)});
  scene.addBox({"desk_right_leg", Eigen::Vector3d(0.05, -0.50, 0.28),
                Eigen::Vector3d(0.06, 0.06, 0.56)});
  return scene;
}

}  // namespace stretch_collision_planning
