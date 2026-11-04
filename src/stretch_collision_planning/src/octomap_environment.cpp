#include "stretch_collision_planning/octomap_environment.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>

namespace stretch_collision_planning {
namespace {
AxisAlignedBox voxelBox(const octomap::point3d& center, double size,
                        std::string id) {
  return {std::move(id),
          Eigen::Vector3d(center.x(), center.y(), center.z()),
          Eigen::Vector3d::Constant(size)};
}

std::string voxelId(const char* prefix, const Eigen::Vector3d& center) {
  std::ostringstream out;
  out << prefix << "(" << center.x() << "," << center.y() << ","
      << center.z() << ")";
  return out.str();
}

double alignedVoxelCenter(double value, double resolution) {
  return std::floor(value / resolution) * resolution + 0.5 * resolution;
}
}  // namespace

const char* occupancyStatusName(OccupancyStatus status) {
  switch (status) {
    case OccupancyStatus::Free:
      return "FREE";
    case OccupancyStatus::Occupied:
      return "OCCUPIED";
    case OccupancyStatus::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char* sceneFreshnessName(SceneFreshness freshness) {
  switch (freshness) {
    case SceneFreshness::Available:
      return "SCENE_AVAILABLE";
    case SceneFreshness::Stale:
      return "SCENE_STALE";
    case SceneFreshness::Unavailable:
      return "SCENE_UNAVAILABLE";
  }
  return "SCENE_UNAVAILABLE";
}

OctoMapEnvironment::OctoMapEnvironment(OctoMapEnvironmentOptions options)
    : options_(std::move(options)),
      map_frame_(options_.default_map_frame.empty() ? "base_link"
                                                    : options_.default_map_frame) {
  if (!std::isfinite(options_.freshness_timeout) ||
      options_.freshness_timeout < 0.0) {
    options_.freshness_timeout = 1.0;
  }
}

bool OctoMapEnvironment::setMap(std::shared_ptr<const octomap::OcTree> tree,
                                std::string map_frame, TimePoint stamp) {
  if (!tree || tree->getResolution() <= 0.0 ||
      !std::isfinite(tree->getResolution())) {
    clear();
    return false;
  }
  tree_ = std::move(tree);
  map_frame_ = map_frame.empty() ? options_.default_map_frame : std::move(map_frame);
  if (map_frame_.empty()) map_frame_ = "base_link";
  stamp_ = stamp;
  return true;
}

bool OctoMapEnvironment::setMapFromMessage(
    const octomap_msgs::msg::Octomap& msg, TimePoint stamp) {
  std::unique_ptr<octomap::AbstractOcTree> abstract(
      octomap_msgs::msgToMap(msg));
  auto* octree = dynamic_cast<octomap::OcTree*>(abstract.get());
  if (!octree) {
    clear();
    return false;
  }
  abstract.release();
  return setMap(std::shared_ptr<const octomap::OcTree>(octree),
                msg.header.frame_id, stamp);
}

void OctoMapEnvironment::clear() {
  tree_.reset();
  map_frame_ =
      options_.default_map_frame.empty() ? "base_link" : options_.default_map_frame;
  stamp_ = TimePoint{};
}

OccupancyStatus OctoMapEnvironment::occupancyAt(
    const Eigen::Vector3d& point) const {
  if (!tree_ || !point.allFinite()) return OccupancyStatus::Unknown;
  const auto* node = tree_->search(point.x(), point.y(), point.z());
  if (!node) return OccupancyStatus::Unknown;
  return tree_->isNodeOccupied(node) ? OccupancyStatus::Occupied
                                     : OccupancyStatus::Free;
}

bool OctoMapEnvironment::isOccupied(const Eigen::Vector3d& point) const {
  const auto status = occupancyAt(point);
  return status == OccupancyStatus::Occupied ||
         (status == OccupancyStatus::Unknown &&
          options_.unknown_policy == UnknownSpacePolicy::UnknownIsCollision);
}

bool OctoMapEnvironment::collides(const CollisionCapsule& base_capsule,
                                  CollisionContact* contact) const {
  if (!tree_ || !validCapsule(base_capsule)) return false;
  CollisionCapsule capsule = base_capsule;
  capsule.a = map_from_base_ * base_capsule.a;
  capsule.b = map_from_base_ * base_capsule.b;
  if (collidesWithOccupied(capsule, contact)) return true;
  if (options_.unknown_policy == UnknownSpacePolicy::UnknownIsCollision)
    return collidesWithUnknown(capsule, contact);
  return false;
}

SceneFreshness OctoMapEnvironment::freshness(TimePoint now) const {
  if (!tree_) return SceneFreshness::Unavailable;
  const double age =
      std::chrono::duration<double>(now - stamp_).count();
  if (!std::isfinite(age) || age < 0.0 ||
      age > options_.freshness_timeout) {
    return SceneFreshness::Stale;
  }
  return SceneFreshness::Available;
}

double OctoMapEnvironment::resolution() const {
  return tree_ ? tree_->getResolution() : 0.0;
}

bool OctoMapEnvironment::collidesWithOccupied(
    const CollisionCapsule& capsule, CollisionContact* contact) const {
  const Eigen::Vector3d min =
      capsule.a.cwiseMin(capsule.b) - Eigen::Vector3d::Constant(capsule.radius);
  const Eigen::Vector3d max =
      capsule.a.cwiseMax(capsule.b) + Eigen::Vector3d::Constant(capsule.radius);
  const octomap::point3d bbx_min(min.x(), min.y(), min.z());
  const octomap::point3d bbx_max(max.x(), max.y(), max.z());
  for (auto it = tree_->begin_leafs_bbx(bbx_min, bbx_max),
            end = tree_->end_leafs_bbx();
       it != end; ++it) {
    if (!tree_->isNodeOccupied(*it)) continue;
    const auto center = it.getCoordinate();
    const Eigen::Vector3d c(center.x(), center.y(), center.z());
    bool self = false;
    for (const auto& r : ignored_)
      self = self || segmentSegmentDistanceSquared(c, c, r.a, r.b) <=
                         r.radius * r.radius;
    if (self) continue;
    const AxisAlignedBox box =
        voxelBox(center, it.getSize(), voxelId("octomap_occupied",
                                               Eigen::Vector3d(center.x(),
                                                               center.y(),
                                                               center.z())));
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

bool OctoMapEnvironment::collidesWithUnknown(
    const CollisionCapsule& capsule, CollisionContact* contact) const {
  const double res = tree_->getResolution();
  if (res <= 0.0 || !std::isfinite(res)) return false;

  const Eigen::Vector3d min =
      capsule.a.cwiseMin(capsule.b) - Eigen::Vector3d::Constant(capsule.radius);
  const Eigen::Vector3d max =
      capsule.a.cwiseMax(capsule.b) + Eigen::Vector3d::Constant(capsule.radius);

  for (double x = alignedVoxelCenter(min.x(), res); x <= max.x() + 1e-12;
       x += res) {
    for (double y = alignedVoxelCenter(min.y(), res); y <= max.y() + 1e-12;
         y += res) {
      for (double z = alignedVoxelCenter(min.z(), res); z <= max.z() + 1e-12;
           z += res) {
        const Eigen::Vector3d center(x, y, z);
        AxisAlignedBox box{"octomap_unknown", center,
                           Eigen::Vector3d::Constant(res)};
        if (!capsuleIntersectsBox(capsule, box)) continue;
        if (occupancyAt(center) == OccupancyStatus::Unknown) {
          if (contact) {
            contact->robot_geometry = capsule.id;
            contact->environment_geometry = voxelId("octomap_unknown", center);
            contact->kind = CollisionContactKind::Unknown;
            contact->robot_capsule = capsule;
            contact->environment_box = box;
          }
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace stretch_collision_planning
