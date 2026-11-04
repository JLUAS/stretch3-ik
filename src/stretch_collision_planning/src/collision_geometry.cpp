#include "stretch_collision_planning/collision_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace stretch_collision_planning {
namespace {
double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}
}  // namespace

bool validBox(const AxisAlignedBox& box) {
  return box.center.allFinite() && box.size.allFinite() &&
         (box.size.array() > 0.0).all();
}

bool validCapsule(const CollisionCapsule& capsule) {
  return capsule.a.allFinite() && capsule.b.allFinite() &&
         std::isfinite(capsule.radius) && capsule.radius >= 0.0;
}

bool segmentIntersectsAabb(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           const Eigen::Vector3d& min,
                           const Eigen::Vector3d& max) {
  if (!a.allFinite() || !b.allFinite() || !min.allFinite() ||
      !max.allFinite() || (min.array() > max.array()).any())
    return false;

  double t_min = 0.0;
  double t_max = 1.0;
  const Eigen::Vector3d d = b - a;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(d[axis]) < 1e-12) {
      if (a[axis] < min[axis] || a[axis] > max[axis]) return false;
      continue;
    }
    double t1 = (min[axis] - a[axis]) / d[axis];
    double t2 = (max[axis] - a[axis]) / d[axis];
    if (t1 > t2) std::swap(t1, t2);
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_min > t_max) return false;
  }
  return true;
}

bool capsuleIntersectsBox(const CollisionCapsule& capsule,
                          const AxisAlignedBox& box, double padding) {
  if (!validCapsule(capsule) || !validBox(box) || !std::isfinite(padding) ||
      padding < 0.0)
    return false;
  const double inflate = capsule.radius + padding;
  const Eigen::Vector3d delta =
      Eigen::Vector3d::Constant(std::max(0.0, inflate));
  return segmentIntersectsAabb(capsule.a, capsule.b, box.min() - delta,
                               box.max() + delta);
}

double segmentSegmentDistanceSquared(const Eigen::Vector3d& p1,
                                     const Eigen::Vector3d& q1,
                                     const Eigen::Vector3d& p2,
                                     const Eigen::Vector3d& q2) {
  if (!p1.allFinite() || !q1.allFinite() || !p2.allFinite() ||
      !q2.allFinite())
    return std::numeric_limits<double>::infinity();

  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;
  if (a <= 1e-18 && e <= 1e-18) return (p1 - p2).squaredNorm();
  if (a <= 1e-18) {
    t = clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= 1e-18) {
      s = clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (denom != 0.0)
        s = clamp((b * f - c * e) / denom, 0.0, 1.0);
      else
        s = 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  const Eigen::Vector3d c1 = p1 + d1 * s;
  const Eigen::Vector3d c2 = p2 + d2 * t;
  return (c1 - c2).squaredNorm();
}

}  // namespace stretch_collision_planning
