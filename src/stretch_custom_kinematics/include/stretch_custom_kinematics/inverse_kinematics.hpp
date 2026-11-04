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
  // Grasp task acceptance. With a fixed base the 5 joints cannot match a full
  // pose exactly, so wrist yaw trades a small approach deviation for position.
  double grasp_position_tolerance = 1e-3, approach_tolerance = 0.12,
         finger_tolerance = 0.15, max_base_offset = 0.6;
};
struct IkResult {
  bool success = false;
  IkFailure reason = IkFailure::ConvergenceFailure;
  RobotState solution;
  double position_error = std::numeric_limits<double>::infinity();
  // Local Z error for the direction task, finger axis (local Y) for grasps.
  double direction_error = std::numeric_limits<double>::infinity();
  double orientation_error = std::numeric_limits<double>::infinity();
  double approach_error = std::numeric_limits<double>::infinity();
  double base_dx = 0;  // Grasp task with free base: forward base translation.
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
  // Grasp task: position, approach (TCP local X) and finger axis (TCP local
  // Y). With free_base, a translation along base X is a sixth variable; the
  // joint solution then belongs to the base moved by result.base_dx.
  IkResult solveGrasp(const Eigen::Isometry3d& target, const RobotState& current,
                      double state_age_seconds, bool free_base) const;
  const JointBounds& bounds() const { return bounds_; }

 private:
  IkResult solveImpl(const Eigen::Isometry3d& target, const RobotState& current,
                     double age, bool grasp, bool free_base) const;
  std::shared_ptr<const stretch_robot_interface::RobotModel> model_;
  ForwardKinematics fk_;
  JointBounds bounds_;
  IkOptions options_;
  double reach_bound_ = 0;
};
}  // namespace stretch_custom_kinematics
