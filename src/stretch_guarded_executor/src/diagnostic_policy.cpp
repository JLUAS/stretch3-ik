#include "stretch_guarded_executor/diagnostic_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace stretch_guarded_executor {
namespace {
constexpr char kBaseCliff[] = "stretch_driver: Pimu/Cliff";
constexpr char kPimuTemperature[] = "stretch_driver: Pimu/Temperature";
using Status = diagnostic_msgs::msg::DiagnosticStatus;

double parseNumberAfter(const std::string& text, const std::string& pattern) {
  const auto pos = text.find(pattern);
  if (pos == std::string::npos) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    size_t parsed = 0;
    const double value = std::stod(text.substr(pos + pattern.size()), &parsed);
    if (parsed > 0 && std::isfinite(value)) return value;
  } catch (...) {}
  return std::numeric_limits<double>::quiet_NaN();
}

double cliffRange(const Status& status) {
  for (const auto& kv : status.values) {
    if (kv.key == "cliff_range" || kv.key == "range") {
      try {
        size_t parsed = 0;
        const double value = std::stod(kv.value, &parsed);
        if (parsed > 0 && std::isfinite(value)) return value;
      } catch (...) {}
    }
  }
  return parseNumberAfter(status.message, "cliff_range=");
}
}  // namespace

void DiagnosticPolicy::validate() const {
  for (const auto& name : error_to_warn_allowlist) {
    if (name != kBaseCliff)
      throw std::invalid_argument(
          "Diagnostic has no reviewed downgrade policy: " + name);
  }
  if (!std::isfinite(pimu_temperature_warn_max_c) ||
      pimu_temperature_warn_max_c < 0.0 ||
      pimu_temperature_warn_max_c > 50.0) {
    throw std::invalid_argument(
        "pimu_temperature_warn_max_c must be finite and between 0.0 and 50.0 C");
  }
  if (!std::isfinite(cliff_range_min) || !std::isfinite(cliff_range_max) ||
      cliff_range_min >= cliff_range_max) {
    throw std::invalid_argument(
        "cliff range limits must be finite and ordered min < max");
  }
}

DiagnosticDecision DiagnosticPolicy::classify(const Status& status) const {
  DiagnosticDecision result{status.level, status.level, false,
                            "original severity retained; fail-closed default"};
  // STALE (including an allowlisted diagnostic) and unknown severity values
  // never qualify. Stream freshness is checked independently by Guard.
  if (status.level != Status::ERROR) return result;

  if (pimu_temperature_error_can_warn && status.name == kPimuTemperature) {
    double temp = std::numeric_limits<double>::quiet_NaN();
    // Try key-value pairs first
    for (const auto& kv : status.values) {
      if (kv.key.find("emperature") != std::string::npos ||
          kv.key.find("temp") != std::string::npos) {
        try {
          size_t pos = 0;
          double parsed = std::stod(kv.value, &pos);
          if (pos > 0 && std::isfinite(parsed)) temp = parsed;
        } catch (...) {}
      }
    }
    // Fallback: try parsing from message
    if (!std::isfinite(temp)) {
      // Try to find a number after common patterns
      for (const auto& pattern : {"temp=", "temperature=", "Temperature="}) {
        temp = parseNumberAfter(status.message, pattern);
        if (std::isfinite(temp)) break;
      }
    }
    if (std::isfinite(temp) && temp <= pimu_temperature_warn_max_c) {
      result.effective_level = Status::WARN;
      result.downgraded = true;
      result.reason = "bounded temperature exception: measured " + 
          std::to_string(temp) + " C <= supervised ceiling " +
          std::to_string(pimu_temperature_warn_max_c) + " C";
    }
  }
  const bool selected =
      std::find(error_to_warn_allowlist.begin(), error_to_warn_allowlist.end(),
                status.name) != error_to_warn_allowlist.end();
  if (selected && status.name == kBaseCliff && !base_motion_enabled) {
    const double range = cliffRange(status);
    if (std::isfinite(range) && range >= cliff_range_min &&
        range <= cliff_range_max) {
      result.effective_level = Status::WARN;
      result.downgraded = true;
      std::ostringstream reason;
      reason << "bounded cliff exception: measured " << range
             << " within supervised range [" << cliff_range_min << ","
             << cliff_range_max
             << "]; base motion disabled; lift-only executor prohibits all "
                "base commands";
      result.reason = reason.str();
    }
  }
  return result;
}
}  // namespace stretch_guarded_executor
