#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <string>
#include <vector>

namespace stretch_guarded_executor {

struct DiagnosticDecision {
  uint8_t original_level;
  uint8_t effective_level;
  bool downgraded = false;
  std::string reason;

  bool blocking() const {
    return effective_level >=
           diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
};

// Fail-closed by default. Profiles can select only reviewed
// diagnostic/subsystem associations, not arbitrary names, prefixes or message
// patterns.
struct DiagnosticPolicy {
  bool base_motion_enabled = false;

  std::vector<std::string> error_to_warn_allowlist;

  // Bounded exception for the Pimu temperature diagnostic.
  // This is intentionally separate from the generic allowlist.
  bool pimu_temperature_error_can_warn = false;
  double pimu_temperature_warn_max_c = 50.0;

  // Bounded exception for calibrated cliff range readings. The Stretch driver
  // may report an ERROR using conservative defaults even when this robot's
  // calibrated closed/open readings are inside the supervised range.
  double cliff_range_min = -100.0;
  double cliff_range_max = 200.0;

  void validate() const;

  DiagnosticDecision classify(
      const diagnostic_msgs::msg::DiagnosticStatus& status) const;
};

}  // namespace stretch_guarded_executor
