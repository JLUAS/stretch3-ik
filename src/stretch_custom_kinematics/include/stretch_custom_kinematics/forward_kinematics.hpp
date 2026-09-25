#pragma once
#include <geometry_msgs/msg/pose.hpp>

#include "stretch_robot_interface/model.hpp"
namespace stretch_custom_kinematics {
class ForwardKinematics {
 public:
  explicit ForwardKinematics(
      std::shared_ptr<const stretch_robot_interface::RobotModel> model);
  Eigen::Isometry3d computeTcpTransform(
      const stretch_robot_interface::RobotState& state) const;
  geometry_msgs::msg::Pose computeTcpPose(
      const stretch_robot_interface::RobotState& state) const;

 private:
  std::shared_ptr<const stretch_robot_interface::RobotModel> model_;
};
struct PoseError {
  double position_m;
  double orientation_rad;
};
PoseError poseError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b);
}  // namespace stretch_custom_kinematics
