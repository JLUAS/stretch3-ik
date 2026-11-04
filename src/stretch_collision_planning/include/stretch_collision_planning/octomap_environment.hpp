#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "stretch_collision_planning/environment.hpp"

namespace stretch_collision_planning {

enum class OccupancyStatus { Free, Occupied, Unknown };
enum class UnknownSpacePolicy { UnknownIsFree, UnknownIsCollision };
enum class SceneFreshness { Available, Stale, Unavailable };

const char* occupancyStatusName(OccupancyStatus status);
const char* sceneFreshnessName(SceneFreshness freshness);

struct OctoMapEnvironmentOptions {
  UnknownSpacePolicy unknown_policy = UnknownSpacePolicy::UnknownIsCollision;
  double freshness_timeout = 1.0;
  std::string default_map_frame = "base_link";
};

class OctoMapEnvironment : public CollisionEnvironment {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit OctoMapEnvironment(OctoMapEnvironmentOptions options = {});

  bool setMap(std::shared_ptr<const octomap::OcTree> tree,
              std::string map_frame, TimePoint stamp = Clock::now());
  bool setMapFromMessage(const octomap_msgs::msg::Octomap& msg,
                         TimePoint stamp = Clock::now());
  void clear();
  // Robot capsules are expressed in base_link; a map in another frame (odom)
  // is queried through this transform, taken while the base is still.
  void setMapFromBase(const Eigen::Isometry3d& map_from_base) {
    map_from_base_ = map_from_base;
  }
  // Occupied voxels inside these base_link capsules are ignored: they are
  // the robot's own body as the camera currently sees it, not obstacles.
  void setIgnoredCapsules(const std::vector<CollisionCapsule>& base_capsules) {
    ignored_.clear();
    for (auto c : base_capsules) {
      c.a = map_from_base_ * c.a;
      c.b = map_from_base_ * c.b;
      ignored_.push_back(c);
    }
  }

  OccupancyStatus occupancyAt(const Eigen::Vector3d& point) const;
  bool isOccupied(const Eigen::Vector3d& point) const;
  bool collides(const CollisionCapsule& capsule,
                CollisionContact* contact = nullptr) const override;

  SceneFreshness freshness(TimePoint now = Clock::now()) const;
  bool hasMap() const { return static_cast<bool>(tree_); }
  const std::string& mapFrame() const { return map_frame_; }
  double resolution() const;
  UnknownSpacePolicy unknownPolicy() const { return options_.unknown_policy; }
  double freshnessTimeout() const { return options_.freshness_timeout; }
  TimePoint stamp() const { return stamp_; }

 private:
  bool collidesWithOccupied(const CollisionCapsule& capsule,
                            CollisionContact* contact) const;
  bool collidesWithUnknown(const CollisionCapsule& capsule,
                           CollisionContact* contact) const;

  OctoMapEnvironmentOptions options_;
  std::shared_ptr<const octomap::OcTree> tree_;
  std::string map_frame_;
  TimePoint stamp_{};
  Eigen::Isometry3d map_from_base_ = Eigen::Isometry3d::Identity();
  std::vector<CollisionCapsule> ignored_;
};

}  // namespace stretch_collision_planning
