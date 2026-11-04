#include "stretch_collision_planning/rrt_connect.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace stretch_collision_planning {
namespace {
using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool sameConfiguration(const JointVector& a, const JointVector& b) {
  return (a - b).cwiseAbs().maxCoeff() <= 1e-12;
}
}  // namespace

const char* planningFailureName(PlanningFailure failure) {
  switch (failure) {
    case PlanningFailure::None:
      return "SUCCESS";
    case PlanningFailure::InvalidOptions:
      return "INVALID_OPTIONS";
    case PlanningFailure::InvalidStart:
      return "INVALID_START";
    case PlanningFailure::InvalidGoal:
      return "INVALID_GOAL";
    case PlanningFailure::Timeout:
      return "TIMEOUT";
    case PlanningFailure::IterationLimit:
      return "ITERATION_LIMIT";
    case PlanningFailure::PathValidationFailed:
      return "PATH_VALIDATION_FAILED";
  }
  return "UNKNOWN";
}

RrtConnectPlanner::RrtConnectPlanner(JointBounds bounds,
                                     StateValidityFn state_validity,
                                     EdgeValidityFn edge_validity)
    : bounds_(std::move(bounds)),
      state_validity_(std::move(state_validity)),
      edge_validity_(std::move(edge_validity)) {
  if (!bounds_.lower.allFinite() || !bounds_.upper.allFinite() ||
      (bounds_.lower.array() >= bounds_.upper.array()).any())
    throw std::invalid_argument("Planner bounds are invalid");
  if (!state_validity_ || !edge_validity_)
    throw std::invalid_argument("Planner validity callbacks are required");
}

RrtConnectPlanner::RrtConnectPlanner(const StateValidityChecker& checker)
    : RrtConnectPlanner(
          checker.bounds(),
          [&checker](const JointVector& q) { return checker.isStateValid(q); },
          [&checker](const JointVector& a, const JointVector& b,
                     double resolution) {
            return checker.isEdgeValid(a, b, resolution);
          }) {}

JointVector RrtConnectPlanner::metricWeights(
    const PlannerOptions& options) const {
  JointVector weights = options.metric_weights;
  if ((weights.array() <= 0.0).all()) {
    for (int i = 0; i < 5; ++i) {
      const double range = bounds_.upper[i] - bounds_.lower[i];
      weights[i] = 1.0 / (range * range);
    }
    return weights;
  }
  if (!weights.allFinite() || (weights.array() <= 0.0).any())
    throw std::invalid_argument(
        "metric_weights must be all positive or all non-positive");
  return weights;
}

double RrtConnectPlanner::distance(const JointVector& a, const JointVector& b,
                                   const JointVector& weights) const {
  const JointVector delta = a - b;
  return std::sqrt((weights.array() * delta.array().square()).sum());
}

JointVector RrtConnectPlanner::steer(const JointVector& from,
                                     const JointVector& to,
                                     double max_distance,
                                     const JointVector& weights) const {
  const double d = distance(from, to, weights);
  if (d <= max_distance || d <= 1e-15) return to;
  return from + (max_distance / d) * (to - from);
}

int RrtConnectPlanner::nearest(const Tree& tree, const JointVector& target,
                               const JointVector& weights) const {
  int best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double d = distance(tree[i].q, target, weights);
    if (d < best_distance) {
      best_distance = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

RrtConnectPlanner::ExtendResult RrtConnectPlanner::extend(
    Tree& tree, const JointVector& target, const PlannerOptions& options,
    const JointVector& weights, PlanningResult& result) const {
  const int near_index = nearest(tree, target, weights);
  const JointVector& near = tree.at(static_cast<size_t>(near_index)).q;
  if (sameConfiguration(near, target)) {
    return {ExtendStatus::Reached, near_index};
  }

  const JointVector next =
      steer(near, target, options.max_extension_distance, weights);
  const auto state = state_validity_(next);
  if (!state.valid) {
    ++result.rejected_states;
    return {ExtendStatus::Trapped, -1};
  }
  const auto edge = edge_validity_(near, next, options.edge_resolution);
  if (!edge.valid) {
    ++result.rejected_edges;
    return {ExtendStatus::Trapped, -1};
  }

  tree.push_back({next, near_index});
  const int index = static_cast<int>(tree.size()) - 1;
  return {sameConfiguration(next, target) ? ExtendStatus::Reached
                                          : ExtendStatus::Advanced,
          index};
}

RrtConnectPlanner::ExtendResult RrtConnectPlanner::connect(
    Tree& tree, const JointVector& target, const PlannerOptions& options,
    const JointVector& weights, PlanningResult& result) const {
  ExtendResult last;
  do {
    last = extend(tree, target, options, weights, result);
  } while (last.status == ExtendStatus::Advanced);
  return last;
}

std::vector<JointVector> RrtConnectPlanner::reconstruct(
    const Tree& active_tree, int active_index, bool active_tree_is_start,
    const Tree& other_tree, int other_index) const {
  auto path_from_root = [](const Tree& tree, int index) {
    std::vector<JointVector> path;
    while (index >= 0) {
      path.push_back(tree.at(static_cast<size_t>(index)).q);
      index = tree.at(static_cast<size_t>(index)).parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  };
  const auto active = path_from_root(active_tree, active_index);
  const auto other = path_from_root(other_tree, other_index);
  std::vector<JointVector> path;

  if (active_tree_is_start) {
    path = active;
    for (int i = static_cast<int>(other.size()) - 2; i >= 0; --i)
      path.push_back(other[static_cast<size_t>(i)]);
  } else {
    path = other;
    for (int i = static_cast<int>(active.size()) - 2; i >= 0; --i)
      path.push_back(active[static_cast<size_t>(i)]);
  }
  return path;
}

PathValidationResult RrtConnectPlanner::validatePath(
    const std::vector<JointVector>& path, double edge_resolution) const {
  if (path.empty()) return {false, -1, -1, "path is empty"};
  for (size_t i = 0; i < path.size(); ++i) {
    const auto state = state_validity_(path[i]);
    if (!state.valid) {
      return {false, static_cast<int>(i), -1,
              "invalid state " + std::to_string(i) + ": " + state.reason};
    }
  }
  for (size_t i = 1; i < path.size(); ++i) {
    const auto edge = edge_validity_(path[i - 1], path[i], edge_resolution);
    if (!edge.valid) {
      return {false, -1, static_cast<int>(i - 1),
              "invalid edge " + std::to_string(i - 1) + "->" +
                  std::to_string(i) + ": " + edge.first_invalid.reason};
    }
  }
  return {true, -1, -1, "path valid"};
}

std::vector<JointVector> RrtConnectPlanner::shortcut(
    const std::vector<JointVector>& path, const PlannerOptions& options,
    PlanningResult& result) const {
  if (path.size() <= 2 || options.shortcut_attempts <= 0) return path;
  std::vector<JointVector> simplified = path;
  std::mt19937 rng(options.random_seed ^ 0x9e3779b9U);
  for (int attempt = 0;
       attempt < options.shortcut_attempts && simplified.size() > 2;
       ++attempt) {
    std::uniform_int_distribution<int> pick(
        0, static_cast<int>(simplified.size()) - 1);
    int a = pick(rng);
    int b = pick(rng);
    if (a > b) std::swap(a, b);
    if (b - a <= 1) continue;
    const auto edge =
        edge_validity_(simplified[static_cast<size_t>(a)],
                       simplified[static_cast<size_t>(b)],
                       options.edge_resolution);
    if (!edge.valid) {
      ++result.rejected_edges;
      continue;
    }
    simplified.erase(simplified.begin() + a + 1, simplified.begin() + b);
  }

  // Remove gratuitous motion in coordinates whose endpoints are identical.
  // This is still collision-aware: every accepted waypoint projection must be
  // a valid state and keep both adjacent edges valid.
  const JointVector start = simplified.front();
  const JointVector goal = simplified.back();
  for (size_t i = 1; i + 1 < simplified.size(); ++i) {
    for (int axis = 0; axis < 5; ++axis) {
      if (std::abs(start[axis] - goal[axis]) > 1e-12) continue;
      JointVector candidate = simplified[i];
      candidate[axis] = start[axis];
      if ((candidate - simplified[i]).cwiseAbs().maxCoeff() <= 1e-12)
        continue;
      const auto state = state_validity_(candidate);
      if (!state.valid) {
        ++result.rejected_states;
        continue;
      }
      const auto incoming = edge_validity_(simplified[i - 1], candidate,
                                           options.edge_resolution);
      if (!incoming.valid) {
        ++result.rejected_edges;
        continue;
      }
      const auto outgoing = edge_validity_(candidate, simplified[i + 1],
                                           options.edge_resolution);
      if (!outgoing.valid) {
        ++result.rejected_edges;
        continue;
      }
      simplified[i] = candidate;
    }
  }
  return simplified;
}

PlanningResult RrtConnectPlanner::plan(const JointVector& start,
                                       const JointVector& goal,
                                       const PlannerOptions& options) const {
  const auto begin = Clock::now();
  PlanningResult result;
  auto fail = [&](PlanningFailure failure, std::string message) {
    result.success = false;
    result.failure = failure;
    result.message = std::move(message);
    result.planning_time_ms = elapsedMs(begin);
    return result;
  };
  auto succeed = [&](std::vector<JointVector> raw, bool direct) {
    result.raw_path = std::move(raw);
    result.direct_path = direct;
    const auto raw_validation =
        validatePath(result.raw_path, options.edge_resolution);
    if (!raw_validation.valid) {
      return fail(PlanningFailure::PathValidationFailed,
                  "raw path validation failed: " + raw_validation.reason);
    }
    if (options.simplify)
      result.simplified_path = shortcut(result.raw_path, options, result);
    else
      result.simplified_path = result.raw_path;
    const auto final_validation =
        validatePath(result.simplified_path, options.edge_resolution);
    if (!final_validation.valid) {
      return fail(PlanningFailure::PathValidationFailed,
                  "simplified path validation failed: " +
                      final_validation.reason);
    }
    result.success = true;
    result.failure = PlanningFailure::None;
    result.message = planningFailureName(PlanningFailure::None);
    result.final_path_valid = true;
    result.planning_time_ms = elapsedMs(begin);
    return result;
  };

  if (options.max_iterations < 0 ||
      !std::isfinite(options.planning_timeout) ||
      options.planning_timeout <= 0.0 ||
      !std::isfinite(options.max_extension_distance) ||
      options.max_extension_distance <= 0.0 ||
      !std::isfinite(options.edge_resolution) ||
      options.edge_resolution <= 0.0 || !std::isfinite(options.goal_bias) ||
      options.goal_bias < 0.0 || options.goal_bias > 1.0 ||
      options.shortcut_attempts < 0) {
    return fail(PlanningFailure::InvalidOptions, "invalid planner options");
  }

  JointVector weights;
  try {
    weights = metricWeights(options);
  } catch (const std::exception& e) {
    return fail(PlanningFailure::InvalidOptions, e.what());
  }

  const auto start_state = state_validity_(start);
  if (!start_state.valid)
    return fail(PlanningFailure::InvalidStart, start_state.reason);
  const auto goal_state = state_validity_(goal);
  if (!goal_state.valid)
    return fail(PlanningFailure::InvalidGoal, goal_state.reason);

  if (sameConfiguration(start, goal)) {
    result.tree_start_size = 1;
    return succeed({start}, true);
  }

  const auto direct = edge_validity_(start, goal, options.edge_resolution);
  if (direct.valid) {
    result.tree_start_size = 1;
    result.tree_goal_size = 1;
    return succeed({start, goal}, true);
  }

  Tree start_tree{{start, -1}};
  Tree goal_tree{{goal, -1}};
  Tree* active = &start_tree;
  Tree* other = &goal_tree;
  bool active_is_start = true;

  std::mt19937 rng(options.random_seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::array<std::uniform_real_distribution<double>, 5> joint_distributions = {
      std::uniform_real_distribution<double>(bounds_.lower[0], bounds_.upper[0]),
      std::uniform_real_distribution<double>(bounds_.lower[1], bounds_.upper[1]),
      std::uniform_real_distribution<double>(bounds_.lower[2], bounds_.upper[2]),
      std::uniform_real_distribution<double>(bounds_.lower[3], bounds_.upper[3]),
      std::uniform_real_distribution<double>(bounds_.lower[4], bounds_.upper[4])};

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    if (std::chrono::duration<double>(Clock::now() - begin).count() >
        options.planning_timeout) {
      result.tree_start_size = static_cast<int>(start_tree.size());
      result.tree_goal_size = static_cast<int>(goal_tree.size());
      result.iterations = iteration;
      return fail(PlanningFailure::Timeout, planningFailureName(PlanningFailure::Timeout));
    }
    result.iterations = iteration + 1;

    JointVector sample;
    if (unit(rng) < options.goal_bias) {
      sample = active_is_start ? goal : start;
    } else {
      for (int i = 0; i < 5; ++i) sample[i] = joint_distributions[i](rng);
    }
    ++result.sampled_states;
    const auto sample_state = state_validity_(sample);
    if (!sample_state.valid) {
      ++result.invalid_samples;
      std::swap(active, other);
      active_is_start = !active_is_start;
      continue;
    }

    const auto extended = extend(*active, sample, options, weights, result);
    if (extended.status != ExtendStatus::Trapped) {
      const JointVector target = active->at(static_cast<size_t>(extended.index)).q;
      const auto connected = connect(*other, target, options, weights, result);
      if (connected.status == ExtendStatus::Reached) {
        result.tree_start_size = static_cast<int>(start_tree.size());
        result.tree_goal_size = static_cast<int>(goal_tree.size());
        return succeed(reconstruct(*active, extended.index, active_is_start,
                                   *other, connected.index),
                       false);
      }
    }
    std::swap(active, other);
    active_is_start = !active_is_start;
  }

  result.tree_start_size = static_cast<int>(start_tree.size());
  result.tree_goal_size = static_cast<int>(goal_tree.size());
  return fail(PlanningFailure::IterationLimit,
              planningFailureName(PlanningFailure::IterationLimit));
}

}  // namespace stretch_collision_planning
