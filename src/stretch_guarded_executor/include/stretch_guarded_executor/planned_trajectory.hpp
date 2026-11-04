#pragma once

#include <string>
#include <vector>

#include "stretch_guarded_executor/guard.hpp"

namespace stretch_guarded_executor {

struct TimedTrajectoryOptions {
  Bounds bounds;
  Vector max_delta = Vector::Ones();
  Vector max_velocity = Vector::Ones();
  Vector max_acceleration = Vector::Ones();
  double min_duration = 3.0;
  double max_duration = 10.0;
  double nominal_duration = 3.0;
};

struct TimedTrajectory {
  std::vector<Vector> states;
  std::vector<double> times_from_start;
};

struct TimedTrajectoryResult {
  bool success = false;
  std::string message;
  TimedTrajectory trajectory;
};

TimedTrajectoryResult buildTimedTrajectory(
    const std::vector<Vector>& geometric_path,
    const TimedTrajectoryOptions& options);

}  // namespace stretch_guarded_executor
