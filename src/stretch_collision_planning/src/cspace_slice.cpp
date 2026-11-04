#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/state_validity.hpp"

using namespace stretch_collision_planning;

namespace {
std::shared_ptr<RobotModel> loadModel(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open URDF: " + path);
  std::stringstream xml;
  xml << file.rdbuf();
  return std::make_shared<RobotModel>(xml.str());
}

int nearestIndex(double value, double lower, double upper, int count) {
  if (count <= 1) return 0;
  const double u = (value - lower) / (upper - lower);
  return std::max(0, std::min(count - 1, static_cast<int>(std::llround(u * (count - 1)))));
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr
        << "Usage: cspace_slice MODEL_URDF HARDWARE_LIMITS_CSV OUT_CSV "
           "[OUT_PPM] [lift_samples=80] [arm_samples=80] "
           "[yaw=0] [pitch=-0.6] [roll=0]\n";
    return 2;
  }

  try {
    const auto model = loadModel(argv[1]);
    const auto bounds = JointBounds::load(argv[2]);
    const auto robot = std::make_shared<PrimitiveRobotCollisionModel>(
        PrimitiveRobotCollisionModel::makeConservativeStretchArm());
    const auto scene = std::make_shared<PrimitiveEnvironment>(
        PrimitiveEnvironment::makeDeskBenchmark());
    StateValidityChecker checker(model, bounds, robot, scene);

    const std::string csv_path = argv[3];
    const std::string ppm_path = argc > 4 ? argv[4] : "";
    const int lift_count = argc > 5 ? std::stoi(argv[5]) : 80;
    const int arm_count = argc > 6 ? std::stoi(argv[6]) : 80;
    const double yaw = argc > 7 ? std::stod(argv[7]) : 0.0;
    const double pitch = argc > 8 ? std::stod(argv[8]) : -0.6;
    const double roll = argc > 9 ? std::stod(argv[9]) : 0.0;
    if (lift_count < 2 || arm_count < 2) {
      std::cerr << "slice dimensions must be at least 2x2\n";
      return 2;
    }

    std::ofstream csv(csv_path);
    if (!csv) throw std::runtime_error("Cannot open CSV: " + csv_path);
    csv << "lift,arm_extension,wrist_yaw,wrist_pitch,wrist_roll,valid,code,reason\n";

    const auto& b = checker.bounds();
    std::vector<unsigned char> image;
    if (!ppm_path.empty()) image.resize(lift_count * arm_count * 3, 255);

    for (int row = 0; row < lift_count; ++row) {
      const double lift =
          b.lower[0] + (b.upper[0] - b.lower[0]) * row / (lift_count - 1);
      for (int col = 0; col < arm_count; ++col) {
        const double arm =
            b.lower[1] + (b.upper[1] - b.lower[1]) * col / (arm_count - 1);
        JointVector q;
        q << lift, arm, yaw, pitch, roll;
        const auto result = checker.isStateValid(q);
        csv << std::setprecision(17) << lift << "," << arm << "," << yaw << ","
            << pitch << "," << roll << "," << result.valid << ","
            << validityCodeName(result.code) << ",\"" << result.reason << "\"\n";
        if (!ppm_path.empty()) {
          const int y = lift_count - 1 - row;
          const size_t index = 3 * (y * arm_count + col);
          image[index + 0] = result.valid ? 245 : 35;
          image[index + 1] = result.valid ? 245 : 35;
          image[index + 2] = result.valid ? 245 : 35;
        }
      }
    }

    if (!ppm_path.empty()) {
      // Mark the deterministic benchmark edge endpoints used by the tests.
      const JointVector q_start = (JointVector() << 0.35, 0.06, yaw, pitch, roll).finished();
      const JointVector q_goal = (JointVector() << 0.75, 0.44, yaw, pitch, roll).finished();
      for (const auto& mark : {std::pair<JointVector, Eigen::Vector3i>(
               q_start, Eigen::Vector3i(0, 180, 60)),
               {q_goal, Eigen::Vector3i(220, 40, 40)}}) {
        const int row = nearestIndex(mark.first[0], b.lower[0], b.upper[0], lift_count);
        const int col = nearestIndex(mark.first[1], b.lower[1], b.upper[1], arm_count);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int rr = std::max(0, std::min(lift_count - 1, lift_count - 1 - row + dy));
            const int cc = std::max(0, std::min(arm_count - 1, col + dx));
            const size_t index = 3 * (rr * arm_count + cc);
            image[index + 0] = static_cast<unsigned char>(mark.second[0]);
            image[index + 1] = static_cast<unsigned char>(mark.second[1]);
            image[index + 2] = static_cast<unsigned char>(mark.second[2]);
          }
        }
      }
      std::ofstream ppm(ppm_path, std::ios::binary);
      if (!ppm) throw std::runtime_error("Cannot open PPM: " + ppm_path);
      ppm << "P6\n" << arm_count << " " << lift_count << "\n255\n";
      ppm.write(reinterpret_cast<const char*>(image.data()),
                static_cast<std::streamsize>(image.size()));
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
