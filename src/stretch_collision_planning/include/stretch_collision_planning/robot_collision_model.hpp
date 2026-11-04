#pragma once

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "stretch_collision_planning/collision_geometry.hpp"

namespace stretch_collision_planning {

using LinkTransforms = std::map<std::string, Eigen::Isometry3d>;

class RobotCollisionModel {
 public:
  virtual ~RobotCollisionModel() = default;
  virtual std::vector<CollisionCapsule> capsules(
      const LinkTransforms& transforms) const = 0;
  virtual std::vector<std::pair<size_t, size_t>> selfCollisionPairs() const {
    return {};
  }
};

struct CapsuleSpec {
  std::string id;
  std::string link_a;
  std::string link_b;
  double radius = 0.0;
};

class PrimitiveRobotCollisionModel : public RobotCollisionModel {
 public:
  explicit PrimitiveRobotCollisionModel(std::vector<CapsuleSpec> specs);
  std::vector<CollisionCapsule> capsules(
      const LinkTransforms& transforms) const override;

  static PrimitiveRobotCollisionModel makeConservativeStretchArm();

 private:
  std::vector<CapsuleSpec> specs_;
};

}  // namespace stretch_collision_planning
