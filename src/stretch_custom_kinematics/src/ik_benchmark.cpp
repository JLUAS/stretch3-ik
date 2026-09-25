#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>

#include "stretch_custom_kinematics/inverse_kinematics.hpp"
using namespace stretch_custom_kinematics;
int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: ik_benchmark MODEL_URDF HARDWARE_LIMITS_CSV "
                 "RESULTS_CSV [count>=1000]\n";
    return 2;
  }
  std::ifstream f(argv[1]);
  std::stringstream xml;
  xml << f.rdbuf();
  auto m = std::make_shared<stretch_robot_interface::RobotModel>(xml.str());
  InverseKinematics ik(m, JointBounds::load(argv[2]));
  ForwardKinematics fk(m);
  int count = argc > 4 ? std::stoi(argv[4]) : 1000;
  if (count < 1000) return 2;
  // Fixed current-state snapshot from the read-only capture, not the target.
  RobotState current;
  current.valid = true;
  current.lift = .5998399625969705;
  current.arm_extension = .10000246696812026;
  current.wrist_yaw = .0051132692929521375;
  current.wrist_pitch = -.6381360077604268;
  current.wrist_roll = -.0015339807878856412;
  std::mt19937 generator(20260924);
  std::uniform_real_distribution<double> uniform(0, 1);
  std::ofstream csv(argv[3]);
  if (!csv) return 2;
  csv << "sample,success,reason,position_m,direction_rad,orientation_rad,solve_"
         "ms,iterations,attempts,target_lift,target_arm,target_yaw,target_"
         "pitch,target_roll,solution_lift,solution_arm,solution_yaw,solution_"
         "pitch,solution_roll\n";
  int successes = 0;
  double sums[5] = {}, maxima[5] = {};
  int max_iterations = 0;
  double sum_iterations = 0;
  for (int i = 0; i < count; ++i) {
    JointVector q;
    for (int j = 0; j < 5; ++j)
      q[j] = ik.bounds().lower[j] +
             uniform(generator) * (ik.bounds().upper[j] - ik.bounds().lower[j]);
    auto target = fk.computeTcpTransform(withCoordinates(current, q));
    auto r = ik.solve(target, current, 0);
    csv << std::setprecision(17) << i << "," << r.success << ","
        << failureName(r.reason) << "," << r.position_error << ","
        << r.direction_error << "," << r.orientation_error << ","
        << r.solve_time_ms << "," << r.iterations << "," << r.attempts;
    for (int j = 0; j < 5; ++j) csv << "," << q[j];
    for (int j = 0; j < 5; ++j) csv << "," << coordinates(r.solution)[j];
    csv << "\n";
    sums[3] += r.solve_time_ms;
    maxima[3] = std::max(maxima[3], r.solve_time_ms);
    sum_iterations += r.iterations;
    max_iterations = std::max(max_iterations, r.iterations);
    if (r.success) {
      ++successes;
      double v[] = {r.position_error, r.direction_error, r.orientation_error};
      for (int j = 0; j < 3; ++j) {
        sums[j] += v[j];
        maxima[j] = std::max(maxima[j], v[j]);
      }
      if (!ik.bounds().contains(coordinates(r.solution))) return 3;
    }
  }
  std::cout << std::setprecision(12) << "samples=" << count
            << " successes=" << successes
            << " success_rate=" << double(successes) / count << "\n";
  const char* labels[] = {"position_m", "direction_rad", "full_orientation_rad",
                          "solve_ms"};
  for (int j = 0; j < 4; ++j)
    std::cout << labels[j]
              << " mean=" << sums[j] / (j == 3 ? count : std::max(successes, 1))
              << " max=" << maxima[j] << "\n";
  std::cout << "iterations mean=" << sum_iterations / count
            << " max=" << max_iterations << "\n";
  return successes == count ? 0 : 1;
}
