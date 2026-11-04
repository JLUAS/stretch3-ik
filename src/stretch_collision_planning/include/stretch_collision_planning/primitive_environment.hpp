#pragma once

#include <optional>
#include <vector>

#include "stretch_collision_planning/collision_geometry.hpp"
#include "stretch_collision_planning/environment.hpp"

namespace stretch_collision_planning {

class PrimitiveEnvironment : public CollisionEnvironment {
 public:
  void addBox(const AxisAlignedBox& box);
  const std::vector<AxisAlignedBox>& boxes() const { return boxes_; }
  bool collides(const CollisionCapsule& capsule,
                CollisionContact* contact = nullptr) const override;

  static PrimitiveEnvironment makeDeskBenchmark();

 private:
  std::vector<AxisAlignedBox> boxes_;
};

}  // namespace stretch_collision_planning
