#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <octomap_msgs/msg/octomap.hpp>

#include "stretch_collision_planning/octomap_environment.hpp"

namespace stretch_collision_planning {

enum class CollisionEnvironmentBackend { Primitive, OctoMap };

const char* collisionEnvironmentBackendName(CollisionEnvironmentBackend backend);
CollisionEnvironmentBackend parseCollisionEnvironmentBackend(
    const std::string& value);

struct OctoMapSceneSnapshot {
  uint64_t id = 0;
  std::shared_ptr<const OctoMapEnvironment> environment;
  OctoMapEnvironment::TimePoint received_at{};
  SceneFreshness status = SceneFreshness::Unavailable;
  std::string reason = "no OctoMap received";

  bool available() const {
    return status == SceneFreshness::Available && environment;
  }
};

class OctoMapSceneStore {
 public:
  explicit OctoMapSceneStore(OctoMapEnvironmentOptions options = {},
                             std::string required_frame = "base_link");

  bool update(const octomap_msgs::msg::Octomap& message,
              OctoMapEnvironment::TimePoint received_at =
                  OctoMapEnvironment::Clock::now(),
              const Eigen::Isometry3d& map_from_base =
                  Eigen::Isometry3d::Identity());
  OctoMapSceneSnapshot snapshot(
      OctoMapEnvironment::TimePoint now =
          OctoMapEnvironment::Clock::now()) const;
  void clear();

  uint64_t latestId() const { return latest_.id; }
  const std::string& lastError() const { return last_error_; }

 private:
  OctoMapEnvironmentOptions options_;
  std::string required_frame_;
  OctoMapSceneSnapshot latest_;
  std::string last_error_ = "no OctoMap received";
  uint64_t next_id_ = 1;
};

}  // namespace stretch_collision_planning
