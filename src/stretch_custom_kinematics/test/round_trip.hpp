#pragma once
#include <functional>

#include "stretch_custom_kinematics/forward_kinematics.hpp"
namespace test_support {
// Future IK supplies this callback; no inverse solver is implemented here.
using InverseSolver = std::function<stretch_robot_interface::RobotState(
    const Eigen::Isometry3d&, const stretch_robot_interface::RobotState&)>;
inline stretch_custom_kinematics::PoseError checkRoundTrip(
    const stretch_custom_kinematics::ForwardKinematics& fk,
    const stretch_robot_interface::RobotState& seed,
    const InverseSolver& solve) {
  const auto target = fk.computeTcpTransform(seed);
  return stretch_custom_kinematics::poseError(
      target, fk.computeTcpTransform(solve(target, seed)));
}
}  // namespace test_support
