#include "esv_planner/swept_astar.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace esv_planner {

namespace {

struct OpenNode {
  int cell = -1;
  double f = 0.0;

  bool operator<(const OpenNode& other) const {
    return f > other.f;
  }
};

double DistanceToBlockedCenters(const Eigen::Vector2d& point,
                                const std::vector<Eigen::Vector2d>& blocked_centers) {
  double min_distance = kInf;
  for (const Eigen::Vector2d& center : blocked_centers) {
    min_distance = std::min(min_distance, (point - center).norm());
  }
  return min_distance;
}

}  // namespace

void SweptAstar::init(const GridMap& map, double required_clearance) {
  map_ = &map;
  checker_ = nullptr;
  required_clearance_ = required_clearance;
}

void SweptAstar::init(const GridMap& map, const CollisionChecker& checker,
                      double required_clearance) {
  map_ = &map;
  checker_ = &checker;
  required_clearance_ = required_clearance;
}

AstarSearchResult SweptAstar::search(const Eigen::Vector2d& start,
                                     const Eigen::Vector2d& goal) const {
  return search(start, goal, SweptAstarSearchOptions());
}

AstarSearchResult SweptAstar::search(const Eigen::Vector2d& start,
                                     const Eigen::Vector2d& goal,
                                     const SweptAstarSearchOptions& options) const {
  AstarSearchResult result;
  if (!map_ || !map_->ready()) {
    return result;
  }

  const GridIndex start_idx = map_->worldToGrid(start.x(), start.y());
  const GridIndex goal_idx = map_->worldToGrid(goal.x(), goal.y());
  if (!map_->isInside(start_idx.x, start_idx.y) ||
      !map_->isInside(goal_idx.x, goal_idx.y) ||
      !isTraversable(start_idx.x, start_idx.y, options) ||
      !isTraversable(goal_idx.x, goal_idx.y, options)) {
    return result;
  }

  const int cell_count = map_->width() * map_->height();
  std::vector<double> g_score(static_cast<size_t>(cell_count), kInf);
  std::vector<int> parent(static_cast<size_t>(cell_count), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(cell_count), 0U);
  std::priority_queue<OpenNode> open;

  const int start_cell = map_->linearIndex(start_idx.x, start_idx.y);
  const int goal_cell = map_->linearIndex(goal_idx.x, goal_idx.y);
  g_score[static_cast<size_t>(start_cell)] = 0.0;
  open.push(OpenNode{start_cell, heuristic(start_idx.x, start_idx.y, goal_idx.x, goal_idx.y)});

  const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  const double dd8[8] = {
      1.41421356237, 1.0, 1.41421356237, 1.0,
      1.0, 1.41421356237, 1.0, 1.41421356237};

  while (!open.empty()) {
    const OpenNode current = open.top();
    open.pop();
    if (closed[static_cast<size_t>(current.cell)] != 0U) {
      continue;
    }
    closed[static_cast<size_t>(current.cell)] = 1U;
    ++result.expanded_nodes;

    if (current.cell == goal_cell) {
      break;
    }

    const int cx = current.cell % map_->width();
    const int cy = current.cell / map_->width();
    for (int dir = 0; dir < 8; ++dir) {
      const int nx = cx + dx8[dir];
      const int ny = cy + dy8[dir];
      if (!map_->isInside(nx, ny) || !isTraversable(nx, ny, options)) {
        continue;
      }

      const int next_cell = map_->linearIndex(nx, ny);
      const double tentative =
          g_score[static_cast<size_t>(current.cell)] +
          dd8[dir] * map_->resolution() + traversalPenalty(nx, ny, options);
      if (tentative >= g_score[static_cast<size_t>(next_cell)]) {
        continue;
      }

      g_score[static_cast<size_t>(next_cell)] = tentative;
      parent[static_cast<size_t>(next_cell)] = current.cell;
      const double f_score = tentative + heuristic(nx, ny, goal_idx.x, goal_idx.y);
      open.push(OpenNode{next_cell, f_score});
    }
  }

  if (parent[static_cast<size_t>(goal_cell)] == -1 && goal_cell != start_cell) {
    return result;
  }

  std::vector<Eigen::Vector2d> reversed;
  for (int cell = goal_cell; cell != -1; cell = parent[static_cast<size_t>(cell)]) {
    const int gx = cell % map_->width();
    const int gy = cell / map_->width();
    reversed.push_back(map_->gridToWorld(gx, gy));
    if (cell == start_cell) {
      break;
    }
  }

  result.path.assign(reversed.rbegin(), reversed.rend());
  if (!result.path.empty()) {
    result.path.front() = start;
    result.path.back() = goal;
    result.success = true;
  }
  return result;
}

bool SweptAstar::isTraversable(int gx, int gy) const {
  return isTraversable(gx, gy, SweptAstarSearchOptions());
}

bool SweptAstar::isTraversable(int gx, int gy,
                               const SweptAstarSearchOptions& options) const {
  if (!map_ || !withinBounds(gx, gy, options)) {
    return false;
  }
  if (map_->getEsdfCell(gx, gy) <= required_clearance_) {
    return false;
  }

  if (checker_ != nullptr && options.min_safe_yaw_count > 0) {
    const Eigen::Vector2d world = map_->gridToWorld(gx, gy);
    if (static_cast<int>(checker_->safeYawIndices(world.x(), world.y()).size()) <
        options.min_safe_yaw_count) {
      return false;
    }
  }

  if (options.blocked_radius > 0.0 && !options.blocked_centers.empty()) {
    const Eigen::Vector2d world = map_->gridToWorld(gx, gy);
    if (DistanceToBlockedCenters(world, options.blocked_centers) <
        options.blocked_radius) {
      return false;
    }
  }
  return true;
}

bool SweptAstar::withinBounds(int gx, int gy,
                              const SweptAstarSearchOptions& options) const {
  if (options.max_x < options.min_x || options.max_y < options.min_y) {
    return true;
  }
  return gx >= options.min_x && gx <= options.max_x &&
         gy >= options.min_y && gy <= options.max_y;
}

double SweptAstar::traversalPenalty(int gx, int gy,
                                    const SweptAstarSearchOptions& options) const {
  if (!map_) {
    return 0.0;
  }

  double penalty = 0.0;
  const double clearance = map_->getEsdfCell(gx, gy);
  if (options.clearance_penalty > 0.0 &&
      options.preferred_clearance > required_clearance_) {
    penalty += options.clearance_penalty *
               std::max(0.0, options.preferred_clearance - clearance);
  }

  if (options.blocked_center_penalty > 0.0 &&
      options.blocked_radius > 0.0 &&
      !options.blocked_centers.empty()) {
    const Eigen::Vector2d world = map_->gridToWorld(gx, gy);
    const double distance = DistanceToBlockedCenters(world, options.blocked_centers);
    const double soft_band = 2.0 * options.blocked_radius;
    if (distance < soft_band) {
      penalty += options.blocked_center_penalty *
                 (soft_band - distance) / std::max(1e-3, soft_band);
    }
  }
  return penalty;
}

double SweptAstar::heuristic(int gx, int gy, int goal_x, int goal_y) const {
  const double dx = static_cast<double>(goal_x - gx);
  const double dy = static_cast<double>(goal_y - gy);
  return std::sqrt(dx * dx + dy * dy) * map_->resolution();
}

}  // namespace esv_planner
