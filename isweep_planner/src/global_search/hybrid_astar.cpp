#include "isweep_planner/global_search/hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace isweep_planner {

namespace {

struct Node {
  SE2State state;
  double g = kInf;
  double f = kInf;
  int parent = -1;
  int dir = 0;
  int steer_idx = 0;
};

struct OpenItem {
  double f = kInf;
  int idx = -1;

  bool operator>(const OpenItem& other) const {
    if (f != other.f) {
      return f > other.f;
    }
    return idx > other.idx;
  }
};

struct SearchResult {
  bool ok = false;
  bool reached_goal = false;
  int expansions = 0;
  double nearest_h = kInf;
};

}  // namespace

HybridAStarPlanner::HybridAStarPlanner() {}

void HybridAStarPlanner::init(const GridMap& map, const CollisionChecker& checker,
                              const HybridAStarParams& params) {
  map_ = &map;
  checker_ = &checker;
  params_ = params;
}

bool HybridAStarPlanner::inBoundsAndFree(const SE2State& state) const {
  const GridIndex index = map_->worldToGrid(state.x, state.y);
  if (!map_->isInside(index.x, index.y)) {
    return false;
  }
  return checker_->isFree(state);
}

double HybridAStarPlanner::heuristic(const SE2State& lhs, const SE2State& rhs) const {
  const double distance = std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
  const double yaw_delta = std::abs(normalizeAngle(lhs.yaw - rhs.yaw));
  return distance + 0.2 * yaw_delta;
}

uint64_t HybridAStarPlanner::keyOf(const SE2State& state) const {
  const GridIndex index = map_->worldToGrid(state.x, state.y);
  const int yaw_bin = checker_->binFromYaw(state.yaw);
  const uint64_t gx = static_cast<uint64_t>(std::max(0, index.x));
  const uint64_t gy = static_cast<uint64_t>(std::max(0, index.y));
  const uint64_t bins = static_cast<uint64_t>(std::max(1, checker_->numYawBins()));
  return ((gx * static_cast<uint64_t>(std::max(1, map_->height()))) + gy) * bins +
         static_cast<uint64_t>(std::max(0, yaw_bin));
}

bool HybridAStarPlanner::plan(const SE2State& start, const SE2State& goal,
                              std::vector<SE2State>& path) {
  path.clear();
  if (!map_ || !checker_) {
    return false;
  }

  SE2State start_search = start;
  SE2State goal_search = goal;

  const auto choose_nearest_safe_yaw_at =
      [&](double wx, double wy, double ref_yaw, SE2State* out) -> bool {
    SE2State probe(wx, wy, ref_yaw);
    if (inBoundsAndFree(probe)) {
      *out = probe;
      return true;
    }

    const std::vector<int> safe_bins = checker_->safeYawIndices(wx, wy);
    if (safe_bins.empty()) {
      return false;
    }

    int best_bin = safe_bins.front();
    double best_err =
        std::abs(normalizeAngle(checker_->yawFromBin(best_bin) - ref_yaw));
    for (int bin : safe_bins) {
      const double yaw = checker_->yawFromBin(bin);
      const double err = std::abs(normalizeAngle(yaw - ref_yaw));
      if (err < best_err) {
        best_err = err;
        best_bin = bin;
      }
    }

    *out = SE2State(wx, wy, checker_->yawFromBin(best_bin));
    return true;
  };

  const auto recover_pose = [&](const SE2State& ref, SE2State* out) -> bool {
    if (choose_nearest_safe_yaw_at(ref.x, ref.y, ref.yaw, out)) {
      return true;
    }

    const double radial_step = std::max(
        map_->resolution(),
        (params_.pose_recovery_radial_step > 1e-9)
            ? params_.pose_recovery_radial_step
            : map_->resolution());
    const double max_radius = std::max(radial_step, params_.pose_recovery_max_radius);
    const int ang_samples = std::max(8, params_.pose_recovery_angular_samples);

    bool found = false;
    SE2State best_pose = ref;
    double best_cost = kInf;

    for (double radius = radial_step; radius <= max_radius + 1e-9; radius += radial_step) {
      for (int i = 0; i < ang_samples; ++i) {
        const double angle =
            (kTwoPi * static_cast<double>(i)) / static_cast<double>(ang_samples);
        const double wx = ref.x + radius * std::cos(angle);
        const double wy = ref.y + radius * std::sin(angle);

        SE2State candidate;
        if (!choose_nearest_safe_yaw_at(wx, wy, ref.yaw, &candidate)) {
          continue;
        }

        const double pos_cost = std::hypot(candidate.x - ref.x, candidate.y - ref.y);
        const double yaw_cost = 0.1 * std::abs(normalizeAngle(candidate.yaw - ref.yaw));
        const double cost = pos_cost + yaw_cost;
        if (cost < best_cost) {
          best_cost = cost;
          best_pose = candidate;
          found = true;
        }
      }
    }

    if (found) {
      *out = best_pose;
    }
    return found;
  };

  if (!recover_pose(start, &start_search) || !recover_pose(goal, &goal_search)) {
    return false;
  }

  const auto can_connect_to_goal = [&](const SE2State& from, const SE2State& to) -> bool {
    const double dist = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(
        2, static_cast<int>(std::ceil(dist / std::max(0.05, map_->resolution() * 0.5))));
    for (int i = 1; i <= steps; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      SE2State sample;
      sample.x = from.x + (to.x - from.x) * ratio;
      sample.y = from.y + (to.y - from.y) * ratio;
      sample.yaw =
          normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * ratio);
      if (!inBoundsAndFree(sample)) {
        return false;
      }
    }
    return true;
  };

  struct SearchTuning {
    int max_expansions = 0;
    int steer_samples = 0;
  };

  const auto run_search =
      [&](const SearchTuning& tuning, std::vector<SE2State>* out_path) -> SearchResult {
    SearchResult result;
    out_path->clear();

    const int steer_samples = std::max(3, tuning.steer_samples | 1);
    std::vector<double> steer_values;
    steer_values.reserve(static_cast<size_t>(steer_samples));
    for (int i = 0; i < steer_samples; ++i) {
      const double ratio = (steer_samples == 1)
                               ? 0.0
                               : static_cast<double>(i) /
                                     static_cast<double>(steer_samples - 1);
      steer_values.push_back(-params_.max_steer + 2.0 * params_.max_steer * ratio);
    }

    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(std::max(32, tuning.max_expansions)) + 8);
    std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open;
    std::unordered_map<uint64_t, double> best_g;
    best_g.reserve(static_cast<size_t>(std::max(64, tuning.max_expansions / 2)));

    Node start_node;
    start_node.state = start_search;
    start_node.g = 0.0;
    start_node.f = heuristic(start_search, goal_search);
    start_node.parent = -1;
    start_node.dir = 0;
    start_node.steer_idx = steer_samples / 2;
    nodes.push_back(start_node);
    open.push({start_node.f, 0});
    best_g[keyOf(start_search)] = 0.0;

    int best_goal_idx = -1;
    int nearest_idx = 0;
    int connect_goal_idx = -1;
    double nearest_h = heuristic(start_search, goal_search);
    int expansions = 0;
    const double connect_trigger_dist =
        std::max(1.0, 4.0 * params_.goal_tolerance_pos);

    while (!open.empty() && expansions < tuning.max_expansions) {
      const OpenItem item = open.top();
      open.pop();
      if (item.idx < 0 || item.idx >= static_cast<int>(nodes.size())) {
        continue;
      }
      const Node current = nodes[item.idx];
      if (item.f > current.f + 1e-9) {
        continue;
      }
      ++expansions;

      const double current_h = heuristic(current.state, goal_search);
      if (current_h < nearest_h) {
        nearest_h = current_h;
        nearest_idx = item.idx;
      }

      const double pos_err =
          std::hypot(current.state.x - goal_search.x, current.state.y - goal_search.y);
      const double yaw_err =
          std::abs(normalizeAngle(current.state.yaw - goal_search.yaw));
      if (pos_err <= params_.goal_tolerance_pos &&
          yaw_err <= params_.goal_tolerance_yaw) {
        best_goal_idx = item.idx;
        break;
      }

      if (current_h <= connect_trigger_dist &&
          can_connect_to_goal(current.state, goal_search)) {
        connect_goal_idx = item.idx;
        break;
      }

      for (int dir : {1, -1}) {
        for (int steer_idx = 0; steer_idx < steer_samples; ++steer_idx) {
          const double steer = steer_values[steer_idx];
          SE2State next;
          next.x =
              current.state.x + dir * params_.step_size * std::cos(current.state.yaw);
          next.y =
              current.state.y + dir * params_.step_size * std::sin(current.state.yaw);
          next.yaw = normalizeAngle(
              current.state.yaw + dir * params_.step_size / params_.wheel_base *
                                      std::tan(steer));

          if (!inBoundsAndFree(next)) {
            continue;
          }

          double step_cost = params_.step_size;
          if (dir < 0) {
            step_cost *= params_.reverse_penalty;
          }

          const double steer_norm =
              (params_.max_steer > 1e-9) ? (std::abs(steer) / params_.max_steer) : 0.0;
          step_cost += params_.steer_penalty * steer_norm;

          if (current.dir != 0 && dir != current.dir) {
            step_cost += params_.switch_penalty;
          }
          step_cost += params_.steer_change_penalty *
                       static_cast<double>(std::abs(steer_idx - current.steer_idx));

          const double g2 = current.g + step_cost;
          const uint64_t key = keyOf(next);
          std::unordered_map<uint64_t, double>::const_iterator it = best_g.find(key);
          if (it != best_g.end() && it->second <= g2) {
            continue;
          }
          best_g[key] = g2;

          Node child;
          child.state = next;
          child.g = g2;
          child.f = g2 + heuristic(next, goal_search);
          child.parent = item.idx;
          child.dir = dir;
          child.steer_idx = steer_idx;
          const int child_idx = static_cast<int>(nodes.size());
          nodes.push_back(child);
          open.push({child.f, child_idx});
        }
      }
    }

    int target_idx = -1;
    bool add_goal_after_reconstruct = false;
    if (best_goal_idx >= 0) {
      target_idx = best_goal_idx;
      result.reached_goal = true;
    } else if (connect_goal_idx >= 0) {
      target_idx = connect_goal_idx;
      add_goal_after_reconstruct = true;
    } else if (nearest_idx >= 0) {
      target_idx = nearest_idx;
      add_goal_after_reconstruct = true;
    }
    if (target_idx < 0) {
      return result;
    }

    std::vector<SE2State> reversed;
    for (int idx = target_idx; idx >= 0; idx = nodes[idx].parent) {
      reversed.push_back(nodes[idx].state);
      if (nodes[idx].parent < 0) {
        break;
      }
    }
    if (reversed.empty()) {
      return result;
    }

    out_path->assign(reversed.rbegin(), reversed.rend());
    if (add_goal_after_reconstruct && out_path->size() >= 2 &&
        can_connect_to_goal(out_path->back(), goal_search)) {
      out_path->push_back(goal_search);
    }

    result.expansions = expansions;
    result.nearest_h = nearest_h;
    result.ok = out_path->size() >= 2;
    return result;
  };

  const int full_expansions = std::max(100, params_.max_expansions);
  const int full_steer = std::max(3, params_.steer_samples | 1);
  int fast_expansions = params_.fast_pass_max_expansions;
  if (fast_expansions <= 0) {
    fast_expansions = full_expansions;
  }
  fast_expansions = std::max(100, std::min(fast_expansions, full_expansions));

  int fast_steer = params_.fast_pass_steer_samples;
  if (fast_steer <= 0) {
    fast_steer = full_steer;
  }
  fast_steer = std::max(3, fast_steer | 1);

  SearchResult fast =
      run_search({fast_expansions, fast_steer}, &path);
  if (fast.ok) {
    return true;
  }
  if (fast_expansions == full_expansions && fast_steer == full_steer) {
    return false;
  }

  SearchResult full =
      run_search({full_expansions, full_steer}, &path);
  return full.ok;
}

}  // namespace isweep_planner
