#pragma once

#include <functional>
#include <string>
#include <vector>

#include "stretch_collision_planning/state_validity.hpp"

namespace stretch_collision_planning {

enum class PlanningFailure {
  None,
  InvalidOptions,
  InvalidStart,
  InvalidGoal,
  Timeout,
  IterationLimit,
  PathValidationFailed
};

const char* planningFailureName(PlanningFailure failure);

struct PlannerOptions {
  int max_iterations = 2000;
  double planning_timeout = 2.0;
  // Measured in the planner's weighted C-space metric.
  double max_extension_distance = 0.12;
  double edge_resolution = 0.01;
  double goal_bias = 0.05;
  unsigned random_seed = 7;
  bool simplify = true;
  int shortcut_attempts = 120;
  // If all entries are <= 0, the default metric is joint-range normalization:
  // distance^2 = sum(((q_a[i] - q_b[i]) / range[i])^2).
  // Otherwise every entry must be positive and distance^2 =
  // sum(metric_weights[i] * delta[i]^2).
  JointVector metric_weights = JointVector::Zero();
};

struct PathValidationResult {
  bool valid = false;
  int invalid_state_index = -1;
  int invalid_edge_index = -1;
  std::string reason;
};

struct PlanningResult {
  bool success = false;
  PlanningFailure failure = PlanningFailure::IterationLimit;
  std::string message;
  std::vector<JointVector> raw_path;
  std::vector<JointVector> simplified_path;
  int iterations = 0;
  int tree_start_size = 0;
  int tree_goal_size = 0;
  int sampled_states = 0;
  int invalid_samples = 0;
  int rejected_states = 0;
  int rejected_edges = 0;
  double planning_time_ms = 0.0;
  bool direct_path = false;
  bool final_path_valid = false;

  const std::vector<JointVector>& path() const {
    return simplified_path.empty() ? raw_path : simplified_path;
  }
};

class RrtConnectPlanner {
 public:
  using StateValidityFn = std::function<StateValidityResult(const JointVector&)>;
  using EdgeValidityFn =
      std::function<EdgeValidityResult(const JointVector&, const JointVector&,
                                       double)>;

  RrtConnectPlanner(JointBounds bounds, StateValidityFn state_validity,
                    EdgeValidityFn edge_validity);
  explicit RrtConnectPlanner(const StateValidityChecker& checker);

  PlanningResult plan(const JointVector& start, const JointVector& goal,
                      const PlannerOptions& options = {}) const;

  PathValidationResult validatePath(const std::vector<JointVector>& path,
                                    double edge_resolution) const;

 private:
  struct Node {
    JointVector q = JointVector::Zero();
    int parent = -1;
  };
  enum class ExtendStatus { Trapped, Advanced, Reached };
  struct ExtendResult {
    ExtendStatus status = ExtendStatus::Trapped;
    int index = -1;
  };

  using Tree = std::vector<Node>;

  JointVector metricWeights(const PlannerOptions& options) const;
  double distance(const JointVector& a, const JointVector& b,
                  const JointVector& weights) const;
  JointVector steer(const JointVector& from, const JointVector& to,
                    double max_distance,
                    const JointVector& weights) const;
  int nearest(const Tree& tree, const JointVector& target,
              const JointVector& weights) const;
  ExtendResult extend(Tree& tree, const JointVector& target,
                      const PlannerOptions& options,
                      const JointVector& weights,
                      PlanningResult& result) const;
  ExtendResult connect(Tree& tree, const JointVector& target,
                       const PlannerOptions& options,
                       const JointVector& weights,
                       PlanningResult& result) const;
  std::vector<JointVector> reconstruct(const Tree& active_tree,
                                       int active_index,
                                       bool active_tree_is_start,
                                       const Tree& other_tree,
                                       int other_index) const;
  std::vector<JointVector> shortcut(const std::vector<JointVector>& path,
                                    const PlannerOptions& options,
                                    PlanningResult& result) const;

  JointBounds bounds_;
  StateValidityFn state_validity_;
  EdgeValidityFn edge_validity_;
};

}  // namespace stretch_collision_planning
