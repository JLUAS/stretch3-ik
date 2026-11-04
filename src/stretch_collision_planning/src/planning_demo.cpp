#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

#include "stretch_collision_planning/primitive_environment.hpp"
#include "stretch_collision_planning/robot_collision_model.hpp"
#include "stretch_collision_planning/rrt_connect.hpp"

using namespace stretch_collision_planning;

namespace {
std::shared_ptr<RobotModel> loadModel(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open URDF: " + path);
  std::stringstream xml;
  xml << file.rdbuf();
  return std::make_shared<RobotModel>(xml.str());
}

JointVector q(double lift, double arm, double yaw = 0.0, double pitch = -0.6,
              double roll = 0.0) {
  JointVector value;
  value << lift, arm, yaw, pitch, roll;
  return value;
}

void writePathCsv(const std::string& path, const std::vector<JointVector>& raw,
                  const std::vector<JointVector>& simplified) {
  std::ofstream csv(path);
  if (!csv) throw std::runtime_error("Cannot open path CSV: " + path);
  csv << "path,index,lift,arm_extension,wrist_yaw,wrist_pitch,wrist_roll\n";
  auto write = [&](const char* name, const std::vector<JointVector>& points) {
    for (size_t i = 0; i < points.size(); ++i) {
      csv << name << "," << i;
      for (int axis = 0; axis < 5; ++axis)
        csv << "," << std::setprecision(17) << points[i][axis];
      csv << "\n";
    }
  };
  write("raw", raw);
  write("simplified", simplified);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const auto kinematics_share =
        ament_index_cpp::get_package_share_directory("stretch_custom_kinematics");
    const std::string urdf = argc > 1 ? argv[1]
                                      : kinematics_share + "/config/captured_robot.urdf";
    const std::string limits =
        argc > 2 ? argv[2] : kinematics_share + "/config/hardware_limits.csv";
    const std::string out_csv = argc > 3 ? argv[3] : "";

    auto model = loadModel(urdf);
    auto robot = std::make_shared<PrimitiveRobotCollisionModel>(
        PrimitiveRobotCollisionModel::makeConservativeStretchArm());
    auto scene = std::make_shared<PrimitiveEnvironment>(
        PrimitiveEnvironment::makeDeskBenchmark());
    StateValidityChecker checker(model, JointBounds::load(limits), robot, scene);

    const JointVector start = q(0.35, 0.06);
    const JointVector goal = q(0.75, 0.44);
    const double edge_resolution = 0.01;

    const auto start_valid = checker.isStateValid(start);
    const auto goal_valid = checker.isStateValid(goal);
    const auto direct = checker.isEdgeValid(start, goal, edge_resolution);

    std::cout << std::boolalpha;
    std::cout << "START VALID: " << start_valid.valid << "\n";
    std::cout << "GOAL VALID: " << goal_valid.valid << "\n";
    std::cout << "DIRECT EDGE: " << (direct.valid ? "FREE" : "COLLISION") << "\n\n";

    PlannerOptions options;
    options.edge_resolution = edge_resolution;
    options.random_seed = 29;
    options.max_iterations = 4000;
    options.planning_timeout = 3.0;
    options.max_extension_distance = 0.12;
    options.goal_bias = 0.10;
    options.shortcut_attempts = 200;

    RrtConnectPlanner planner(checker);
    const auto result = planner.plan(start, goal, options);

    std::cout << "RRT-CONNECT:\n";
    std::cout << (result.success ? "SUCCESS" : "FAILURE") << "\n\n";
    std::cout << "reason: " << result.message << "\n";
    std::cout << "iterations: " << result.iterations << "\n";
    std::cout << "samples: " << result.sampled_states << "\n";
    std::cout << "invalid samples: " << result.invalid_samples << "\n";
    std::cout << "rejected states: " << result.rejected_states << "\n";
    std::cout << "rejected edges: " << result.rejected_edges << "\n";
    std::cout << "tree_start_size: " << result.tree_start_size << "\n";
    std::cout << "tree_goal_size: " << result.tree_goal_size << "\n";
    std::cout << "raw waypoints: " << result.raw_path.size() << "\n";
    std::cout << "simplified waypoints: " << result.simplified_path.size() << "\n";
    std::cout << "planning time ms: " << result.planning_time_ms << "\n";
    std::cout << "FINAL PATH VALID: " << result.final_path_valid << "\n";

    if (!out_csv.empty() && result.success) {
      writePathCsv(out_csv, result.raw_path, result.simplified_path);
      std::cout << "PATH CSV: " << out_csv << "\n";
    }
    return result.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
}
