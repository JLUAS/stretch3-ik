#pragma once

#include <optional>
#include <string>

#include "stretch_collision_planning/collision_geometry.hpp"

namespace stretch_collision_planning {

enum class CollisionContactKind { Unspecified, Occupied, Unknown };

struct CollisionContact {
  std::string robot_geometry;
  std::string environment_geometry;
  CollisionContactKind kind = CollisionContactKind::Unspecified;
  std::optional<CollisionCapsule> robot_capsule;
  std::optional<AxisAlignedBox> environment_box;
};

class CollisionEnvironment {
 public:
  virtual ~CollisionEnvironment() = default;
  virtual bool collides(const CollisionCapsule& capsule,
                        CollisionContact* contact = nullptr) const = 0;
};

}  // namespace stretch_collision_planning
