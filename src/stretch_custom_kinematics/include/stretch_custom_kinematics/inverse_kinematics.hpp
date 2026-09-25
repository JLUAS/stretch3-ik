#pragma once
#include <array>
#include <limits>

#include "stretch_custom_kinematics/forward_kinematics.hpp"
namespace stretch_custom_kinematics {
using JointVector = Eigen::Matrix<double, 5, 1>;
using RobotState = stretch_robot_interface::RobotState;
struct JointBounds {
  JointVector lower, upper;
  // Strict CSV: logical_name,lower,upper. No silent fallback limits.
  static JointBounds load(const std::string& path);
  JointBounds intersectModel(
      const stretch_robot_interface::RobotModel& model) const;
  bool contains(const JointVector& q) const;
};
JointVector coordinates(const RobotState& q);
RobotState withCoordinates(const RobotState& seed, const JointVector& q);
enum class IkFailure {
  None,
  UnreachableTarget,
  JointLimit,
  ConvergenceFailure,
  InvalidTarget,
  StaleState
};
const char* failureName(IkFailure reason);
struct IkOptions {
  double position_tolerance = 1e-6, direction_tolerance = 1e-5;
  double state_timeout = 1.0;
  int max_iterations = 160, max_attempts = 64;
};
struct IkResult {
  bool success = false;
  IkFailure reason = IkFailure::ConvergenceFailure;
  RobotState solution;
  double position_error = std::numeric_limits<double>::infinity();
  double direction_error = std::numeric_limits<double>::infinity();
  double orientation_error = std::numeric_limits<double>::infinity();
  double solve_time_ms = 0;
  int iterations = 0, attempts = 0;
};
// Task: position XYZ and TCP local-Z unit direction (3+2 constraints).
// Full rotation about target local Z is deliberately not constrained.
class InverseKinematics {
 public:
  InverseKinematics(
      std::shared_ptr<const stretch_robot_interface::RobotModel> model,
      const JointBounds& hardware, IkOptions options = {});
  IkResult solve(const Eigen::Isometry3d& target, const RobotState& current,
                 double state_age_seconds) const;
  IkResult solve(const geometry_msgs::msg::Pose& target,
                 const RobotState& current, double state_age_seconds) const;
  const JointBounds& bounds() const { return bounds_; }

 private:
  std::shared_ptr<const stretch_robot_interface::RobotModel> model_;
  ForwardKinematics fk_;
  JointBounds bounds_;
  IkOptions options_;
  double reach_bound_ = 0;
};
}  // namespace stretch_custom_kinematics
