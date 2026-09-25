#include "stretch_guarded_executor/lift_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace stretch_guarded_executor {

std::vector<double> LiftSequence::nominalTargets(double current, double target,
                                                  double max_step,
                                                  double tolerance) {
  for (double value : {current, target, max_step, tolerance})
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite lift plan");
  if (max_step <= 0 || max_step > .030000000001 || tolerance <= 0 ||
      tolerance > .003000000001)
    throw std::invalid_argument("invalid lift segmentation limits");
  const double distance = std::abs(target - current);
  if (distance <= tolerance) return {};
  const uint32_t count =
      static_cast<uint32_t>(std::ceil((distance - 1e-12) / max_step));
  const double direction = target > current ? 1. : -1.;
  std::vector<double> result;
  result.reserve(count);
  for (uint32_t i = 1; i <= count; ++i) {
    const double traveled = std::min(distance, i * max_step);
    result.push_back(i == count ? target : current + direction * traveled);
  }
  return result;
}

LiftSequenceCheck LiftSequence::start(double current, double target,
                                       double lower, double upper,
                                       double max_step, double tolerance) {
  if (active_) return {false, "another lift execution is active"};
  for (double value : {current, target, lower, upper, max_step, tolerance})
    if (!std::isfinite(value)) return {false, "non-finite lift target/state"};
  if (lower >= upper || current < lower || current > upper || target < lower ||
      target > upper)
    return {false, "lift target/state outside physical limits"};
  try {
    const auto plan = nominalTargets(current, target, max_step, tolerance);
    target_ = target;
    lower_ = lower;
    upper_ = upper;
    max_step_ = max_step;
    tolerance_ = tolerance;
    completed_ = 0;
    total_ = static_cast<uint32_t>(plan.size());
    canceled_ = failed_ = false;
    active_ = !plan.empty();
    reason_ = active_ ? "absolute lift target accepted"
                      : "target already satisfied";
    return {true, reason_};
  } catch (const std::exception& e) {
    return {false, e.what()};
  }
}

bool LiftSequence::satisfied(double measured) const {
  return std::isfinite(measured) && std::abs(measured - target_) <= tolerance_;
}

std::optional<double> LiftSequence::next(double measured) {
  if (!active_ || canceled_ || failed_ || !std::isfinite(measured) ||
      measured < lower_ || measured > upper_)
    return std::nullopt;
  if (satisfied(measured)) {
    active_ = false;
    reason_ = "absolute lift target reached";
    return std::nullopt;
  }
  const double remaining = target_ - measured;
  const double step = std::clamp(remaining, -max_step_, max_step_);
  const double next_target =
      std::abs(remaining) <= max_step_ + 1e-12 ? target_ : measured + step;
  const uint32_t remaining_segments = static_cast<uint32_t>(
      std::ceil((std::abs(remaining) - 1e-12) / max_step_));
  total_ = std::max(total_, completed_ + remaining_segments);
  return std::clamp(next_target, lower_, upper_);
}

void LiftSequence::segmentComplete(double measured) {
  if (!active_) return;
  ++completed_;
  if (satisfied(measured)) {
    active_ = false;
    reason_ = "absolute lift target reached";
  } else {
    reason_ = "segment endpoint verified";
  }
}

void LiftSequence::fail(const std::string& reason) {
  active_ = false;
  failed_ = true;
  reason_ = reason;
}

void LiftSequence::cancel() {
  active_ = false;
  canceled_ = true;
  reason_ = "lift execution canceled";
}

}  // namespace stretch_guarded_executor
