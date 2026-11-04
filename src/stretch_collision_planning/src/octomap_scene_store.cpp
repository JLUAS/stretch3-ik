#include "stretch_collision_planning/octomap_scene_store.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace stretch_collision_planning {

const char* collisionEnvironmentBackendName(
    CollisionEnvironmentBackend backend) {
  return backend == CollisionEnvironmentBackend::Primitive ? "PRIMITIVE"
                                                            : "OCTOMAP";
}

CollisionEnvironmentBackend parseCollisionEnvironmentBackend(
    const std::string& value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (normalized == "primitive") return CollisionEnvironmentBackend::Primitive;
  if (normalized == "octomap") return CollisionEnvironmentBackend::OctoMap;
  throw std::invalid_argument(
      "collision_environment must be 'primitive' or 'octomap'");
}

OctoMapSceneStore::OctoMapSceneStore(OctoMapEnvironmentOptions options,
                                     std::string required_frame)
    : options_(std::move(options)), required_frame_(std::move(required_frame)) {
  if (required_frame_.empty()) required_frame_ = options_.default_map_frame;
  if (required_frame_.empty()) required_frame_ = "base_link";
}

bool OctoMapSceneStore::update(
    const octomap_msgs::msg::Octomap& message,
    OctoMapEnvironment::TimePoint received_at,
    const Eigen::Isometry3d& map_from_base) {
  if (message.header.frame_id != required_frame_) {
    last_error_ = "map frame '" + message.header.frame_id +
                  "' does not match required frame '" + required_frame_ + "'";
    latest_ = {};
    return false;
  }
  auto environment = std::make_shared<OctoMapEnvironment>(options_);
  if (!environment->setMapFromMessage(message, received_at)) {
    last_error_ = "OctoMap message could not be decoded as OcTree";
    latest_ = {};
    return false;
  }
  environment->setMapFromBase(map_from_base);
  latest_.id = next_id_++;
  latest_.environment = std::move(environment);
  latest_.received_at = received_at;
  latest_.status = SceneFreshness::Available;
  latest_.reason = "scene available";
  last_error_.clear();
  return true;
}

OctoMapSceneSnapshot OctoMapSceneStore::snapshot(
    OctoMapEnvironment::TimePoint now) const {
  OctoMapSceneSnapshot result = latest_;
  if (!result.environment) {
    result.status = SceneFreshness::Unavailable;
    result.reason = last_error_.empty() ? "no OctoMap received" : last_error_;
    return result;
  }
  result.status = result.environment->freshness(now);
  result.reason = result.status == SceneFreshness::Stale
                      ? "latest OctoMap exceeds freshness timeout"
                      : "scene available";
  return result;
}

void OctoMapSceneStore::clear() {
  latest_ = {};
  last_error_ = "no OctoMap received";
}

}  // namespace stretch_collision_planning
