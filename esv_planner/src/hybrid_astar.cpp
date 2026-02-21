#include "esv_planner/hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <string>
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

std::string HybridAStarPlanner::keyOf(const SE2State& s) const {
  GridIndex gi = map_->worldToGrid(s.x, s.y);
  int ybin = checker_->binFromYaw(s.yaw);
  return std::to_string(gi.x) + "," + std::to_string(gi.y) + "," + std::to_string(ybin);
}

bool HybridAStarPlanner::plan(const SE2State& start, const SE2State& goal,
                              std::vector<SE2State>& path) {
  path.clear();
  if (!map_ || !checker_) return false;

  SE2State start_search = start;
  SE2State goal_search = goal;

  auto chooseNearestSafeYaw = [&](const SE2State& ref, SE2State* out) -> bool {
    if (inBoundsAndFree(ref)) {
      *out = ref;
      return true;
    }
    auto safe_bins = checker_->safeYawIndices(ref.x, ref.y);
    if (safe_bins.empty()) return false;
    int best_bin = safe_bins.front();
    double best_err = std::abs(normalizeAngle(checker_->yawFromBin(best_bin) - ref.yaw));
    for (int bin : safe_bins) {
      double yaw = checker_->yawFromBin(bin);
      double err = std::abs(normalizeAngle(yaw - ref.yaw));
      if (err < best_err) {
        best_err = err;
        best_bin = bin;
      }
    }
    *out = ref;
    out->yaw = checker_->yawFromBin(best_bin);
    return true;
  };

  if (!chooseNearestSafeYaw(start, &start_search)) return false;
  if (!chooseNearestSafeYaw(goal, &goal_search)) return false;

  int steer_samples = std::max(3, params_.steer_samples | 1);
  std::vector<double> steer_values;
  steer_values.reserve(static_cast<size_t>(steer_samples));
  for (int i = 0; i < steer_samples; ++i) {
    double r = (steer_samples == 1) ? 0.0 :
        (static_cast<double>(i) / static_cast<double>(steer_samples - 1));
    double steer = -params_.max_steer + 2.0 * params_.max_steer * r;
    steer_values.push_back(steer);
  }

  std::vector<Node> nodes;
  nodes.reserve(static_cast<size_t>(params_.max_expansions) + 8);
  std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open;
  std::unordered_map<std::string, double> best_g;

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
  double best_goal_cost = kInf;
  int nearest_idx = 0;
  double nearest_h = heuristic(start_search, goal_search);
  int expansions = 0;

  while (!open.empty() && expansions < params_.max_expansions) {
    OpenItem item = open.top();
    open.pop();
    if (item.idx < 0 || item.idx >= static_cast<int>(nodes.size())) continue;
    const Node cur = nodes[item.idx];
    if (item.f > cur.f + 1e-9) continue;
    ++expansions;

    double cur_h = heuristic(cur.state, goal_search);
    if (cur_h < nearest_h) {
      nearest_h = cur_h;
      nearest_idx = item.idx;
    }

    double pos_err = std::hypot(cur.state.x - goal_search.x, cur.state.y - goal_search.y);
    double yaw_err = std::abs(normalizeAngle(cur.state.yaw - goal_search.yaw));
    if (pos_err <= params_.goal_tolerance_pos && yaw_err <= params_.goal_tolerance_yaw) {
      best_goal_idx = item.idx;
      best_goal_cost = cur.g;
      break;
    }

    for (int dir : {1, -1}) {
      for (int si = 0; si < steer_samples; ++si) {
        double steer = steer_values[si];
        SE2State nxt;
        nxt.x = cur.state.x + dir * params_.step_size * std::cos(cur.state.yaw);
        nxt.y = cur.state.y + dir * params_.step_size * std::sin(cur.state.yaw);
        nxt.yaw = normalizeAngle(cur.state.yaw +
                                 dir * params_.step_size / params_.wheel_base *
                                 std::tan(steer));

        if (!inBoundsAndFree(nxt)) continue;

        double step_cost = params_.step_size;
        if (dir < 0) step_cost *= params_.reverse_penalty;

        double steer_norm = (params_.max_steer > 1e-9) ?
            (std::abs(steer) / params_.max_steer) : 0.0;
        step_cost += params_.steer_penalty * steer_norm;

        if (cur.dir != 0 && dir != cur.dir) {
          step_cost += params_.switch_penalty;
        }
        step_cost += params_.steer_change_penalty *
            static_cast<double>(std::abs(si - cur.steer_idx));

        double g2 = cur.g + step_cost;
        std::string key = keyOf(nxt);
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
        int cidx = static_cast<int>(nodes.size());
        nodes.push_back(child);
        open.push({child.f, cidx});
      }
    }
  }

  int target_idx = -1;
  if (best_goal_idx >= 0 && std::isfinite(best_goal_cost)) {
    target_idx = best_goal_idx;
  } else if (nearest_idx >= 0) {
    // Fallback: return best-effort path to the nearest explored state.
    target_idx = nearest_idx;
  } else {
    return false;
  }

  std::vector<SE2State> rev;
  for (int idx = target_idx; idx >= 0; idx = nodes[idx].parent) {
    rev.push_back(nodes[idx].state);
    if (nodes[idx].parent < 0) break;
  }
  if (rev.empty()) return false;

  path.assign(rev.rbegin(), rev.rend());
  if (best_goal_idx < 0 && path.size() >= 2) {
    // Try to connect the tail to the goal with straight interpolation.
    const SE2State tail = path.back();
    double dist = std::hypot(goal_search.x - tail.x, goal_search.y - tail.y);
    int steps = std::max(2, static_cast<int>(
        std::ceil(dist / std::max(0.05, map_->resolution() * 0.5))));
    bool connect_ok = true;
    for (int i = 1; i <= steps; ++i) {
      double r = static_cast<double>(i) / static_cast<double>(steps);
      SE2State s;
      s.x = tail.x + (goal_search.x - tail.x) * r;
      s.y = tail.y + (goal_search.y - tail.y) * r;
      s.yaw = normalizeAngle(tail.yaw +
                             normalizeAngle(goal_search.yaw - tail.yaw) * r);
      if (!inBoundsAndFree(s)) {
        connect_ok = false;
        break;
      }
    }
    if (connect_ok) {
      path.push_back(goal_search);
    }
  }

  return path.size() >= 2;
}

}  // namespace esv_planner
