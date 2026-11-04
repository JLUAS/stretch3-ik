#include "stretch_guarded_executor/planned_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace stretch_guarded_executor {
namespace {
bool validOptions(const TimedTrajectoryOptions& options, std::string& reason) {
  if (!options.bounds.lower.allFinite() || !options.bounds.upper.allFinite() ||
      (options.bounds.lower.array() >= options.bounds.upper.array()).any()) {
    reason = "invalid trajectory bounds";
    return false;
  }
  if (!options.max_delta.allFinite() || !options.max_velocity.allFinite() ||
      !options.max_acceleration.allFinite() ||
      (options.max_delta.array() <= 0.0).any() ||
      (options.max_velocity.array() <= 0.0).any() ||
      (options.max_acceleration.array() <= 0.0).any()) {
    reason = "invalid trajectory limits";
    return false;
  }
  if (!std::isfinite(options.min_duration) ||
      !std::isfinite(options.max_duration) ||
      !std::isfinite(options.nominal_duration) ||
      options.min_duration <= 0.0 || options.max_duration <= 0.0 ||
      options.nominal_duration <= 0.0 ||
      options.min_duration > options.max_duration) {
    reason = "invalid trajectory timing options";
    return false;
  }
  return true;
}

double segmentDuration(const Vector& a, const Vector& b,
                       const TimedTrajectoryOptions& options) {
  const Vector delta = (b - a).cwiseAbs();
  double duration = 0.05;
  for (int axis = 0; axis < 5; ++axis) {
    if (delta[axis] < 1e-15) continue;
    duration =
        std::max(duration, 1.875 * delta[axis] / options.max_velocity[axis]);
    duration = std::max(
        duration,
        std::sqrt((10.0 / std::sqrt(3.0)) * delta[axis] /
                  options.max_acceleration[axis]));
  }
  return duration;
}

bool withinBounds(const Vector& q, const Bounds& bounds) {
  return q.allFinite() &&
         (q.array() >= bounds.lower.array() - 1e-8).all() &&
         (q.array() <= bounds.upper.array() + 1e-8).all();
}
}  // namespace

TimedTrajectoryResult buildTimedTrajectory(
    const std::vector<Vector>& geometric_path,
    const TimedTrajectoryOptions& options) {
  TimedTrajectoryResult result;
  std::string reason;
  if (!validOptions(options, reason)) {
    result.message = reason;
    return result;
  }
  if (geometric_path.empty()) {
    result.message = "geometric path is empty";
    return result;
  }
  if (geometric_path.size() == 1) {
    result.message = "geometric path has no executable motion";
    return result;
  }
  for (size_t i = 0; i < geometric_path.size(); ++i) {
    if (!withinBounds(geometric_path[i], options.bounds)) {
      result.message = "geometric path point " + std::to_string(i) +
                       " outside bounds";
      return result;
    }
  }

  // One minimum-time quintic per geometric edge, sampled every 0.1 s. The
  // robot only stops at edge corners, where the direction changes anyway.
  std::vector<Vector> states{geometric_path.front()};
  std::vector<double> times{0.0};
  double total = 0.0;
  for (size_t edge = 1; edge < geometric_path.size(); ++edge) {
    const Vector& a = geometric_path[edge - 1];
    const Vector& b = geometric_path[edge];
    const Vector delta = b - a;
    if (delta.cwiseAbs().maxCoeff() < 1e-12) continue;
    const double duration = segmentDuration(a, b, options);
    int steps = static_cast<int>(std::ceil(duration / 0.1));
    // Peak quintic step is 1.875 * delta / steps; keep it inside max_delta.
    for (int axis = 0; axis < 5; ++axis)
      steps = std::max(steps, static_cast<int>(std::ceil(
                                  1.875 * std::abs(delta[axis]) /
                                  options.max_delta[axis])));
    for (int k = 1; k <= steps; ++k) {
      const double u = static_cast<double>(k) / steps;
      const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      states.push_back(a + blend * delta);
      times.push_back(total + u * duration);
    }
    total += duration;
  }

  const double requested_min =
      std::max(options.min_duration, options.nominal_duration);
  if (total < requested_min && total > 0.0) {
    const double scale = requested_min / total;
    for (double& value : times) value *= scale;
    total = times.back();
  }

  if (!std::isfinite(total) || total < options.min_duration ||
      total > options.max_duration) {
    std::ostringstream message;
    message << "timed trajectory duration " << total
            << " outside allowed range [" << options.min_duration << ", "
            << options.max_duration << "]";
    result.message = message.str();
    return result;
  }

  result.success = true;
  result.message = "timed trajectory generated";
  result.trajectory.states = std::move(states);
  result.trajectory.times_from_start = std::move(times);
  return result;
}

}  // namespace stretch_guarded_executor
