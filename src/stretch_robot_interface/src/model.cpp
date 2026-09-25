#include "stretch_robot_interface/model.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace stretch_robot_interface {
const std::array<std::string, 4> RobotModel::arm_names = {
    "joint_arm_l3", "joint_arm_l2", "joint_arm_l1", "joint_arm_l0"};
Eigen::Isometry3d RobotModel::origin(const urdf::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond r(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
  if (!t.matrix().allFinite() || !r.coeffs().allFinite() || r.norm() < 1e-12)
    throw std::invalid_argument("Nonfinite/invalid URDF origin");
  t.linear() = r.normalized().toRotationMatrix();
  return t;
}
RobotModel::RobotModel(const std::string& xml, const std::string& base,
                       const std::string& tcp)
    : base_(base), tcp_(tcp) {
  if (!model_.initString(xml)) throw std::invalid_argument("Malformed URDF");
  auto link = model_.getLink(tcp);
  if (!model_.getLink(base) || !link)
    throw std::invalid_argument("Missing base or TCP link");
  while (link->name != base) {
    if (!link->parent_joint)
      throw std::invalid_argument("Base is not a TCP ancestor");
    chain_.push_back(link->parent_joint);
    link = model_.getLink(link->parent_joint->parent_link_name);
  }
  std::reverse(chain_.begin(), chain_.end());
  const std::set<std::string> expected = {
      "joint_lift",        "joint_arm_l3",    "joint_arm_l2",
      "joint_arm_l1",      "joint_arm_l0",    "joint_wrist_yaw",
      "joint_wrist_pitch", "joint_wrist_roll"};
  std::set<std::string> found;
  Eigen::Isometry3d zero = Eigen::Isometry3d::Identity();
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  bool arm_started = false, arm_finished = false;
  for (const auto& j : chain_) {
    zero = zero * origin(j->parent_to_joint_origin_transform);
    if (j->type == urdf::Joint::FIXED) continue;
    if (!expected.count(j->name) || j->mimic)
      throw std::invalid_argument("Unsupported movable/mimic joint: " +
                                  j->name);
    found.insert(j->name);
    const bool arm = std::find(arm_names.begin(), arm_names.end(), j->name) !=
                     arm_names.end();
    if (j->type != (arm || j->name == "joint_lift" ? urdf::Joint::PRISMATIC
                                                   : urdf::Joint::REVOLUTE))
      throw std::invalid_argument("Wrong joint type: " + j->name);
    Eigen::Vector3d axis(j->axis.x, j->axis.y, j->axis.z);
    if (!axis.allFinite() || std::abs(axis.norm() - 1) > 1e-6 || !j->limits ||
        !std::isfinite(j->limits->lower) || !std::isfinite(j->limits->upper) ||
        j->limits->lower > j->limits->upper)
      throw std::invalid_argument("Invalid axis/limits: " + j->name);
    if (arm) {
      if (arm_finished)
        throw std::invalid_argument("Telescope interrupted by movable joint");
      auto d = zero.linear() * axis;
      if (arm_started && (d - direction).norm() > 1e-8)
        throw std::invalid_argument(
            "Telescope axes are not parallel and codirectional");
      direction = d;
      arm_started = true;
    } else if (arm_started)
      arm_finished = true;
  }
  if (found != expected)
    throw std::invalid_argument(
        "Required manipulation joint absent from TCP chain");
}
void RobotModel::check(const std::string& name, double value) const {
  auto j = model_.getJoint(name);
  if (!std::isfinite(value) || value < j->limits->lower - 1e-8 ||
      value > j->limits->upper + 1e-8)
    throw std::invalid_argument("Joint outside URDF limits: " + name);
}
std::map<std::string, double> RobotModel::physicalJoints(
    const RobotState& q) const {
  std::map<std::string, double> p = {{"joint_lift", q.lift},
                                     {"joint_wrist_yaw", q.wrist_yaw},
                                     {"joint_wrist_pitch", q.wrist_pitch},
                                     {"joint_wrist_roll", q.wrist_roll}};
  for (auto& name : arm_names) p[name] = q.arm_extension / 4.0;
  for (auto& [name, value] : p) check(name, value);
  return p;
}
RobotState RobotModel::decode(const sensor_msgs::msg::JointState& msg) const {
  if (msg.name.size() != msg.position.size())
    throw std::invalid_argument("Joint name/position size mismatch");
  std::map<std::string, double> p;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (!std::isfinite(msg.position[i]) ||
        !p.emplace(msg.name[i], msg.position[i]).second)
      throw std::invalid_argument("Duplicate/nonfinite joint state");
  }
  auto get = [&](const std::string& n) {
    if (!p.count(n)) throw std::invalid_argument("Missing joint: " + n);
    check(n, p.at(n));
    return p.at(n);
  };
  RobotState q;
  q.lift = get("joint_lift");
  q.wrist_yaw = get("joint_wrist_yaw");
  q.wrist_pitch = get("joint_wrist_pitch");
  q.wrist_roll = get("joint_wrist_roll");
  for (auto& n : arm_names) q.arm_extension += get(n);
  if (p.count("wrist_extension") &&
      std::abs(p.at("wrist_extension") - q.arm_extension) > 1e-6)
    throw std::invalid_argument(
        "wrist_extension disagrees with sum of arm stages");
  physicalJoints(q);
  q.timestamp = msg.header.stamp;
  q.valid = true;
  return q;
}
bool fresh(double receive_age, double stamp_age, double timeout,
           double future_tolerance) {
  return std::isfinite(receive_age) && std::isfinite(stamp_age) &&
         std::isfinite(future_tolerance) && timeout > 0 &&
         future_tolerance >= 0 && future_tolerance <= .2 &&
         receive_age >= 0 && receive_age <= timeout &&
         stamp_age >= -future_tolerance && stamp_age <= timeout;
}
}  // namespace stretch_robot_interface
