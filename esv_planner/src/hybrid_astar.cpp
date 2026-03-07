#include "esv_planner/hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace esv_planner {

namespace {

struct Node {
  SE2State state;
  double g = kInf;
  double f = kInf;
  int parent = -1;
  int dir = 0;        // +1 forward, -1 reverse
  int steer_idx = 0;  // primitive index
};

struct OpenItem {
  double f = kInf;
  int idx = -1;
  bool operator>(const OpenItem& o) const {
    if (f != o.f) return f > o.f;
    return idx > o.idx;
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

bool HybridAStarPlanner::inBoundsAndFree(const SE2State& s) const {
  GridIndex gi = map_->worldToGrid(s.x, s.y);
  if (!map_->isInside(gi.x, gi.y)) return false;
  return checker_->isFree(s);
}

double HybridAStarPlanner::heuristic(const SE2State& a, const SE2State& b) const {
  double d = std::hypot(a.x - b.x, a.y - b.y);
  double dyaw = std::abs(normalizeAngle(a.yaw - b.yaw));
  return d + 0.2 * dyaw;
}

uint64_t HybridAStarPlanner::keyOf(const SE2State& s) const {
  GridIndex gi = map_->worldToGrid(s.x, s.y);
  int ybin = checker_->binFromYaw(s.yaw);
  const uint64_t gx = static_cast<uint64_t>(std::max(0, gi.x));
  const uint64_t gy = static_cast<uint64_t>(std::max(0, gi.y));
  const uint64_t bins = static_cast<uint64_t>(std::max(1, checker_->numYawBins()));
  return ((gx * static_cast<uint64_t>(std::max(1, map_->height()))) + gy) * bins +
         static_cast<uint64_t>(std::max(0, ybin));
}

bool HybridAStarPlanner::plan(const SE2State& start, const SE2State& goal,
                              std::vector<SE2State>& path) {
  path.clear();
  if (!map_ || !checker_) return false;

  SE2State start_search = start;
  SE2State goal_search = goal;

  auto chooseNearestSafeYawAt = [&](double wx, double wy, double ref_yaw, SE2State* out) -> bool {
    SE2State probe(wx, wy, ref_yaw);
    if (inBoundsAndFree(probe)) {
      *out = probe;
      return true;
    }

    auto safe_bins = checker_->safeYawIndices(wx, wy);
    if (safe_bins.empty()) return false;

    int best_bin = safe_bins.front();
    double best_err = std::abs(normalizeAngle(checker_->yawFromBin(best_bin) - ref_yaw));
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

  auto recoverPose = [&](const SE2State& ref, SE2State* out) -> bool {
    if (chooseNearestSafeYawAt(ref.x, ref.y, ref.yaw, out)) {
      return true;
    }

    const double radial_step = std::max(
        map_->resolution(),
        (params_.pose_recovery_radial_step > 1e-9) ?
            params_.pose_recovery_radial_step : map_->resolution());
    const double max_radius = std::max(radial_step, params_.pose_recovery_max_radius);
    const int ang_samples = std::max(8, params_.pose_recovery_angular_samples);

    bool found = false;
    SE2State best_pose = ref;
    double best_cost = kInf;

    for (double r = radial_step; r <= max_radius + 1e-9; r += radial_step) {
      for (int ai = 0; ai < ang_samples; ++ai) {
        const double a = (kTwoPi * static_cast<double>(ai)) / static_cast<double>(ang_samples);
        const double wx = ref.x + r * std::cos(a);
        const double wy = ref.y + r * std::sin(a);

        SE2State cand;
        if (!chooseNearestSafeYawAt(wx, wy, ref.yaw, &cand)) continue;

        const double pos_cost = std::hypot(cand.x - ref.x, cand.y - ref.y);
        const double yaw_cost = 0.1 * std::abs(normalizeAngle(cand.yaw - ref.yaw));
        const double cost = pos_cost + yaw_cost;
        if (cost < best_cost) {
          best_cost = cost;
          best_pose = cand;
          found = true;
        }
      }
    }

    if (found) {
      *out = best_pose;
    }
    return found;
  };

  if (!recoverPose(start, &start_search)) return false;
  if (!recoverPose(goal, &goal_search)) return false;

  auto canConnectToGoal = [&](const SE2State& from, const SE2State& to) -> bool {
    const double dist = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(
        2, static_cast<int>(std::ceil(dist / std::max(0.05, map_->resolution() * 0.5))));
    for (int i = 1; i <= steps; ++i) {
      const double r = static_cast<double>(i) / static_cast<double>(steps);
      SE2State s;
      s.x = from.x + (to.x - from.x) * r;
      s.y = from.y + (to.y - from.y) * r;
      s.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * r);
      if (!inBoundsAndFree(s)) return false;
    }
    return true;
  };

  struct SearchTuning {
    int max_expansions = 0;
    int steer_samples = 0;
  };

  auto runSearch = [&](const SearchTuning& tuning, std::vector<SE2State>* out_path) -> SearchResult {
    SearchResult result;
    out_path->clear();

    const int steer_samples = std::max(3, tuning.steer_samples | 1);
    std::vector<double> steer_values;
    steer_values.reserve(static_cast<size_t>(steer_samples));
    for (int i = 0; i < steer_samples; ++i) {
      const double r = (steer_samples == 1) ? 0.0 :
          (static_cast<double>(i) / static_cast<double>(steer_samples - 1));
      const double steer = -params_.max_steer + 2.0 * params_.max_steer * r;
      steer_values.push_back(steer);
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
    const double connect_trigger_dist = std::max(1.0, 4.0 * params_.goal_tolerance_pos);

    while (!open.empty() && expansions < tuning.max_expansions) {
      OpenItem item = open.top();
      open.pop();
      if (item.idx < 0 || item.idx >= static_cast<int>(nodes.size())) continue;
      const Node cur = nodes[item.idx];
      if (item.f > cur.f + 1e-9) continue;
      ++expansions;

      const double cur_h = heuristic(cur.state, goal_search);
      if (cur_h < nearest_h) {
        nearest_h = cur_h;
        nearest_idx = item.idx;
      }

      const double pos_err = std::hypot(cur.state.x - goal_search.x, cur.state.y - goal_search.y);
      const double yaw_err = std::abs(normalizeAngle(cur.state.yaw - goal_search.yaw));
      if (pos_err <= params_.goal_tolerance_pos && yaw_err <= params_.goal_tolerance_yaw) {
        best_goal_idx = item.idx;
        break;
      }

      if (cur_h <= connect_trigger_dist && canConnectToGoal(cur.state, goal_search)) {
        connect_goal_idx = item.idx;
        break;
      }

      for (int dir : {1, -1}) {
        for (int si = 0; si < steer_samples; ++si) {
          const double steer = steer_values[si];
          SE2State nxt;
          nxt.x = cur.state.x + dir * params_.step_size * std::cos(cur.state.yaw);
          nxt.y = cur.state.y + dir * params_.step_size * std::sin(cur.state.yaw);
          nxt.yaw = normalizeAngle(cur.state.yaw +
                                   dir * params_.step_size / params_.wheel_base *
                                   std::tan(steer));

          if (!inBoundsAndFree(nxt)) continue;

          double step_cost = params_.step_size;
          if (dir < 0) step_cost *= params_.reverse_penalty;

          const double steer_norm = (params_.max_steer > 1e-9) ?
              (std::abs(steer) / params_.max_steer) : 0.0;
          step_cost += params_.steer_penalty * steer_norm;

          if (cur.dir != 0 && dir != cur.dir) {
            step_cost += params_.switch_penalty;
          }
          step_cost += params_.steer_change_penalty *
              static_cast<double>(std::abs(si - cur.steer_idx));

          const double g2 = cur.g + step_cost;
          const uint64_t key = keyOf(nxt);
          auto it = best_g.find(key);
          if (it != best_g.end() && it->second <= g2) continue;
          best_g[key] = g2;

          Node child;
          child.state = nxt;
          child.g = g2;
          child.f = g2 + heuristic(nxt, goal_search);
          child.parent = item.idx;
          child.dir = dir;
          child.steer_idx = si;
          const int cidx = static_cast<int>(nodes.size());
          nodes.push_back(child);
          open.push({child.f, cidx});
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
    if (target_idx < 0) return result;

    std::vector<SE2State> rev;
    for (int idx = target_idx; idx >= 0; idx = nodes[idx].parent) {
      rev.push_back(nodes[idx].state);
      if (nodes[idx].parent < 0) break;
    }
    if (rev.empty()) return result;

    out_path->assign(rev.rbegin(), rev.rend());
    if (add_goal_after_reconstruct && out_path->size() >= 2 &&
        canConnectToGoal(out_path->back(), goal_search)) {
      out_path->push_back(goal_search);
    }

    result.expansions = expansions;
    result.nearest_h = nearest_h;
    result.ok = out_path->size() >= 2;
    return result;
  };

  const int full_exp = std::max(100, params_.max_expansions);
  const int full_steer = std::max(3, params_.steer_samples | 1);
  int fast_exp = params_.fast_pass_max_expansions;
  if (fast_exp <= 0) fast_exp = full_exp;
  fast_exp = std::max(100, std::min(fast_exp, full_exp));

  int fast_steer = params_.fast_pass_steer_samples;
  if (fast_steer <= 0) fast_steer = full_steer;
  fast_steer = std::max(3, fast_steer | 1);

  SearchResult fast = runSearch({fast_exp, fast_steer}, &path);
  if (fast.ok) return true;

  // If fast pass already equals full settings, avoid duplicate work.
  if (fast_exp == full_exp && fast_steer == full_steer) {
    return false;
  }

  SearchResult full = runSearch({full_exp, full_steer}, &path);
  return full.ok;
}

}  // namespace esv_planner
