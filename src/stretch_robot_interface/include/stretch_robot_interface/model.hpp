#pragma once
#include <urdf/model.h>

#include <Eigen/Geometry>
#include <array>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
namespace stretch_robot_interface {
struct RobotState {
  double lift{}, arm_extension{}, wrist_yaw{}, wrist_pitch{}, wrist_roll{};
  builtin_interfaces::msg::Time timestamp;
  bool homed{}, runstopped{}, valid{};
};
class RobotModel {
 public:
  RobotModel(const std::string& xml, const std::string& base = "base_link",
             const std::string& tcp = "link_grasp_center");
  RobotState decode(const sensor_msgs::msg::JointState& msg) const;
  std::map<std::string, double> physicalJoints(const RobotState& q) const;
  const std::vector<urdf::JointConstSharedPtr>& chain() const { return chain_; }
  const std::string& base() const { return base_; }
  const std::string& tcp() const { return tcp_; }
  static Eigen::Isometry3d origin(const urdf::Pose& pose);
  static const std::array<std::string, 4> arm_names;

 private:
  void check(const std::string& name, double value) const;
  urdf::Model model_;
  std::vector<urdf::JointConstSharedPtr> chain_;
  std::string base_, tcp_;
};
bool fresh(double receive_age, double stamp_age, double timeout,
           double future_tolerance = 0.05);
}  // namespace stretch_robot_interface
