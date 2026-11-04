#pragma once

#include <Eigen/Geometry>
#include <string>

namespace stretch_collision_planning {

struct AxisAlignedBox {
  std::string id;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();

  Eigen::Vector3d min() const { return center - 0.5 * size; }
  Eigen::Vector3d max() const { return center + 0.5 * size; }
};

struct CollisionCapsule {
  std::string id;
  std::string link_a;
  std::string link_b;
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

bool validBox(const AxisAlignedBox& box);
bool validCapsule(const CollisionCapsule& capsule);
bool segmentIntersectsAabb(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           const Eigen::Vector3d& min,
                           const Eigen::Vector3d& max);
bool capsuleIntersectsBox(const CollisionCapsule& capsule,
                          const AxisAlignedBox& box, double padding = 0.0);
double segmentSegmentDistanceSquared(const Eigen::Vector3d& p1,
                                     const Eigen::Vector3d& q1,
                                     const Eigen::Vector3d& p2,
                                     const Eigen::Vector3d& q2);

}  // namespace stretch_collision_planning
