#include "stretch_custom_kinematics/inverse_kinematics.hpp"

#include <Eigen/Cholesky>
#include <chrono>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
namespace stretch_custom_kinematics {
namespace {
const std::array<std::string, 5> names = {"lift", "arm_extension", "wrist_yaw",
                                          "wrist_pitch", "wrist_roll"};
using Residual = Eigen::Matrix<double, 6, 1>;
Residual residual(const Eigen::Isometry3d& actual,
                  const Eigen::Isometry3d& target) {
  Residual r;
  r.head<3>() = target.translation() - actual.translation();
  // Three components on S2 have only two independent degrees of freedom.
  r.tail<3>() = 0.2 * (target.linear().col(2) - actual.linear().col(2));
  return r;
}
double directionError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  auto x = a.linear().col(2), y = b.linear().col(2);
  return std::atan2(x.cross(y).norm(), x.dot(y));
}
bool validTarget(const Eigen::Isometry3d& t) {
  return t.matrix().allFinite() &&
         (t.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9 &&
         (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity())
                 .norm() < 1e-6 &&
         std::abs(t.linear().determinant() - 1) < 1e-6;
}
double radicalInverse(unsigned n, unsigned base) {
  double f = 1., v = 0.;
  while (n) {
    f /= base;
    v += f * (n % base);
    n /= base;
  }
  return v;
}
}  // namespace
JointVector coordinates(const RobotState& q) {
  JointVector v;
  v << q.lift, q.arm_extension, q.wrist_yaw, q.wrist_pitch, q.wrist_roll;
  return v;
}
RobotState withCoordinates(const RobotState& seed, const JointVector& q) {
  auto s = seed;
  s.lift = q[0];
  s.arm_extension = q[1];
  s.wrist_yaw = q[2];
  s.wrist_pitch = q[3];
  s.wrist_roll = q[4];
  return s;
}
JointBounds JointBounds::load(const std::string& path) {
  std::ifstream f(path);
  if (!f)
    throw std::invalid_argument("Hardware limit file unavailable: " + path);
  JointBounds b;
  std::set<int> seen;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    std::string name, lo, hi, extra;
    if (!std::getline(s, name, ',') || !std::getline(s, lo, ',') ||
        !std::getline(s, hi, ',') || std::getline(s, extra, ','))
      throw std::invalid_argument("Malformed hardware limits row");
    auto found = std::find(names.begin(), names.end(), name);
    if (found == names.end())
      throw std::invalid_argument("Unknown logical limit: " + name);
    int i = std::distance(names.begin(), found);
    if (!seen.insert(i).second) throw std::invalid_argument("Duplicate limit");
    size_t a = 0, c = 0;
    b.lower[i] = std::stod(lo, &a);
    b.upper[i] = std::stod(hi, &c);
    if (a != lo.size() || c != hi.size())
      throw std::invalid_argument("Malformed limit number");
  }
  if (seen.size() != 5 || !b.lower.allFinite() || !b.upper.allFinite() ||
      (b.lower.array() >= b.upper.array()).any())
    throw std::invalid_argument("Missing or invalid hardware limits");
  return b;
}
JointBounds JointBounds::intersectModel(
    const stretch_robot_interface::RobotModel& m) const {
  if (!lower.allFinite() || !upper.allFinite() ||
      (lower.array() >= upper.array()).any())
    throw std::invalid_argument("Invalid hardware bounds");
  auto b = *this;
  for (auto j : m.chain()) {
    int i = -1;
    double scale = 1;
    if (j->name == "joint_lift")
      i = 0;
    else if (std::find(stretch_robot_interface::RobotModel::arm_names.begin(),
                       stretch_robot_interface::RobotModel::arm_names.end(),
                       j->name) !=
             stretch_robot_interface::RobotModel::arm_names.end()) {
      i = 1;
      scale = 4;
    } else if (j->name == "joint_wrist_yaw")
      i = 2;
    else if (j->name == "joint_wrist_pitch")
      i = 3;
    else if (j->name == "joint_wrist_roll")
      i = 4;
    if (i >= 0) {
      b.lower[i] = std::max(b.lower[i], scale * j->limits->lower);
      b.upper[i] = std::min(b.upper[i], scale * j->limits->upper);
    }
  }
  if ((b.lower.array() >= b.upper.array()).any())
    throw std::invalid_argument("Empty hardware/model limit intersection");
  return b;
}
bool JointBounds::contains(const JointVector& q) const {
  return q.allFinite() && (q.array() >= lower.array()).all() &&
         (q.array() <= upper.array()).all();
}
const char* failureName(IkFailure r) {
  switch (r) {
    case IkFailure::None:
      return "success";
    case IkFailure::UnreachableTarget:
      return "unreachable target";
    case IkFailure::JointLimit:
      return "joint limit";
    case IkFailure::ConvergenceFailure:
      return "convergence failure";
    case IkFailure::InvalidTarget:
      return "invalid target";
    case IkFailure::StaleState:
      return "stale state";
  }
  return "unknown";
}
InverseKinematics::InverseKinematics(
    std::shared_ptr<const stretch_robot_interface::RobotModel> m,
    const JointBounds& h, IkOptions o)
    : model_(m), fk_(m), bounds_(h.intersectModel(*m)), options_(o) {
  if (o.position_tolerance <= 0 || o.direction_tolerance <= 0 ||
      o.state_timeout <= 0 || !std::isfinite(o.position_tolerance) ||
      !std::isfinite(o.direction_tolerance) ||
      !std::isfinite(o.state_timeout) || o.max_iterations < 0 ||
      o.max_attempts < 1)
    throw std::invalid_argument("Invalid IK options");
  // Conservative URDF-derived sphere: triangle inequality for every origin
  // plus every bounded prismatic translation. Only this certificate returns
  // unreachable.
  for (auto j : m->chain()) {
    reach_bound_ += stretch_robot_interface::RobotModel::origin(
                        j->parent_to_joint_origin_transform)
                        .translation()
                        .norm();
    if (j->type == urdf::Joint::PRISMATIC)
      reach_bound_ +=
          std::max(std::abs(j->limits->lower), std::abs(j->limits->upper));
  }
}
IkResult InverseKinematics::solve(const geometry_msgs::msg::Pose& p,
                                  const RobotState& q, double age) const {
  Eigen::Quaterniond r(p.orientation.w, p.orientation.x, p.orientation.y,
                       p.orientation.z);
  if (!r.coeffs().allFinite() || std::abs(r.norm() - 1) > 1e-3) {
    IkResult result;
    result.reason = IkFailure::InvalidTarget;
    return result;
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  t.linear() = r.normalized().toRotationMatrix();
  return solve(t, q, age);
}
IkResult InverseKinematics::solve(const Eigen::Isometry3d& target,
                                  const RobotState& current, double age) const {
  const auto begin = std::chrono::steady_clock::now();
  IkResult result;
  result.solution = current;
  auto finish = [&](IkFailure reason) {
    result.reason = reason;
    result.success = reason == IkFailure::None;
    result.solution.valid = result.success;
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - begin)
                               .count();
    return result;
  };
  if (!validTarget(target)) return finish(IkFailure::InvalidTarget);
  if (!current.valid || !std::isfinite(age) || age < -.05 ||
      age > options_.state_timeout)
    return finish(IkFailure::StaleState);
  if (!bounds_.contains(coordinates(current)))
    return finish(IkFailure::JointLimit);
  if (target.translation().norm() > reach_bound_ + options_.position_tolerance)
    return finish(IkFailure::UnreachableTarget);
  JointVector best = coordinates(current);
  double best_cost = std::numeric_limits<double>::infinity();
  for (int attempt = 0; attempt < options_.max_attempts; ++attempt) {
    ++result.attempts;
    JointVector q = coordinates(current);
    // Always start with current measured state. Deterministic bounded restarts
    // escape local minima; none is the known target-generating configuration.
    if (attempt > 0) {
      const unsigned primes[5] = {2, 3, 5, 7, 11};
      for (int j = 0; j < 5; ++j)
        q[j] = bounds_.lower[j] + (bounds_.upper[j] - bounds_.lower[j]) *
                                      radicalInverse(attempt, primes[j]);
    }
    double damping = 1e-7;
    for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
      ++result.iterations;
      auto pose = fk_.computeTcpTransform(withCoordinates(current, q));
      auto r = residual(pose, target);
      double cost = r.squaredNorm();
      if (cost < best_cost) {
        best_cost = cost;
        best = q;
        result.position_error =
            (pose.translation() - target.translation()).norm();
        result.direction_error = directionError(pose, target);
        result.orientation_error = poseError(pose, target).orientation_rad;
        result.solution = withCoordinates(current, q);
      }
      if (result.position_error <= options_.position_tolerance &&
          result.direction_error <= options_.direction_tolerance)
        return finish(IkFailure::None);
      Eigen::Matrix<double, 6, 5> jac;
      for (int j = 0; j < 5; ++j) {
        auto plus = q, minus = q;
        plus[j] = std::min(q[j] + 1e-6, bounds_.upper[j]);
        minus[j] = std::max(q[j] - 1e-6, bounds_.lower[j]);
        jac.col(j) =
            (residual(fk_.computeTcpTransform(withCoordinates(current, plus)),
                      target) -
             residual(fk_.computeTcpTransform(withCoordinates(current, minus)),
                      target)) /
            (plus[j] - minus[j]);
      }
      JointVector delta = -(jac.transpose() * jac +
                            damping * Eigen::Matrix<double, 5, 5>::Identity())
                               .ldlt()
                               .solve(jac.transpose() * r);
      const double scale =
          std::max({1., std::abs(delta[0]) / .15, std::abs(delta[1]) / .1,
                    std::abs(delta[2]) / .5, std::abs(delta[3]) / .5,
                    std::abs(delta[4]) / .5});
      delta /= scale;
      bool accepted = false;
      for (int ls = 0; ls < 10; ++ls) {
        auto next =
            (q + delta).cwiseMax(bounds_.lower).cwiseMin(bounds_.upper).eval();
        double c =
            residual(fk_.computeTcpTransform(withCoordinates(current, next)),
                     target)
                .squaredNorm();
        if (c < cost - 1e-18) {
          q = next;
          accepted = true;
          damping = std::max(1e-10, damping * .3);
          break;
        }
        delta *= .5;
      }
      if (!accepted) {
        damping *= 10;
        if (damping > 1e2) break;
      }
    }
  }
  // A bound-active best candidate is a constrained failure, not a proof that
  // no alternate branch exists. Interior exhausted searches are convergence
  // failures.
  bool bound = false;
  for (int j = 0; j < 5; ++j)
    bound = bound || std::abs(best[j] - bounds_.lower[j]) < 1e-7 ||
            std::abs(best[j] - bounds_.upper[j]) < 1e-7;
  return finish(bound ? IkFailure::JointLimit : IkFailure::ConvergenceFailure);
}
}  // namespace stretch_custom_kinematics
