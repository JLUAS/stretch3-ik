#include "stretch_custom_kinematics/forward_kinematics.hpp"

#include <stdexcept>
namespace stretch_custom_kinematics {
ForwardKinematics::ForwardKinematics(
    std::shared_ptr<const stretch_robot_interface::RobotModel> model)
    : model_(std::move(model)) {
  if (!model_) throw std::invalid_argument("Null robot model");
}
Eigen::Isometry3d ForwardKinematics::computeTcpTransform(
    const stretch_robot_interface::RobotState& q) const {
  if (!q.valid) throw std::invalid_argument("Invalid logical state");
  const auto p = model_->physicalJoints(q);
  auto t = Eigen::Isometry3d::Identity();
  for (const auto& j : model_->chain()) {
    t = t * stretch_robot_interface::RobotModel::origin(
                j->parent_to_joint_origin_transform);
    if (j->type == urdf::Joint::FIXED) continue;
    const Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    if (j->type == urdf::Joint::PRISMATIC)
      motion.translation() = axis * p.at(j->name);
    else
      motion.linear() =
          Eigen::AngleAxisd(p.at(j->name), axis).toRotationMatrix();
    t = t * motion;
  }
  return t;
}
geometry_msgs::msg::Pose ForwardKinematics::computeTcpPose(
    const stretch_robot_interface::RobotState& state) const {
  const auto t = computeTcpTransform(state);
  Eigen::Quaterniond r(t.linear());
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  p.orientation.x = r.x();
  p.orientation.y = r.y();
  p.orientation.z = r.z();
  p.orientation.w = r.w();
  return p;
}
PoseError poseError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return {(a.translation() - b.translation()).norm(),
          Eigen::Quaterniond(a.linear())
              .angularDistance(Eigen::Quaterniond(b.linear()))};
}
}  // namespace stretch_custom_kinematics
