#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace stretch_guarded_executor {

struct LiftSequenceCheck {
  bool ok = false;
  std::string reason;
};

// Pure, hardware-independent planner/state holder. Every next target is
// computed from fresh measured lift toward one immutable absolute endpoint.
class LiftSequence {
 public:
  LiftSequenceCheck start(double current, double target, double lower,
                          double upper, double max_step, double tolerance);
  std::optional<double> next(double measured);
  void segmentComplete(double measured);
  void fail(const std::string& reason);
  void cancel();

  bool active() const { return active_; }
  bool canceled() const { return canceled_; }
  bool failed() const { return failed_; }
  bool satisfied(double measured) const;
  double target() const { return target_; }
  double tolerance() const { return tolerance_; }
  uint32_t currentSegment() const { return completed_ + (active_ ? 1 : 0); }
  uint32_t completedSegments() const { return completed_; }
  uint32_t totalSegments() const { return total_; }
  const std::string& reason() const { return reason_; }

  static std::vector<double> nominalTargets(double current, double target,
                                             double max_step,
                                             double tolerance);

 private:
  bool active_ = false, canceled_ = false, failed_ = false;
  double target_ = 0, lower_ = 0, upper_ = 0, max_step_ = .03,
         tolerance_ = .0025;
  uint32_t completed_ = 0, total_ = 0;
  std::string reason_ = "idle";
};

}  // namespace stretch_guarded_executor
