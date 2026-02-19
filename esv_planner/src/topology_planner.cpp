#include "esv_planner/topology_planner.h"
#include <algorithm>
#include <random>
#include <queue>
#include <set>
#include <cmath>

namespace esv_planner {

TopologyPlanner::TopologyPlanner() {}

void TopologyPlanner::init(const GridMap& map, const CollisionChecker& checker,
                           int num_samples, int knn, int max_paths) {
  map_ = &map;
  checker_ = &checker;
  num_samples_ = num_samples;
  knn_ = knn;
  max_paths_ = max_paths;
}

void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
  nodes_.clear();
  adjacency_.clear();

  // Add start and goal as first two nodes
  nodes_.push_back(start);
  nodes_.push_back(goal);

  // Random sampling in free space
  std::mt19937 rng(42);
  double x_min = map_->originX();
  double y_min = map_->originY();
  double x_max = x_min + map_->width() * map_->resolution();
  double y_max = y_min + map_->height() * map_->resolution();

  std::uniform_real_distribution<double> dist_x(x_min, x_max);
  std::uniform_real_distribution<double> dist_y(y_min, y_max);

  for (int i = 0; i < num_samples_; ++i) {
    double x = dist_x(rng);
    double y = dist_y(rng);
    GridIndex gi = map_->worldToGrid(x, y);
    if (map_->isInside(gi.x, gi.y) && !map_->isOccupied(gi.x, gi.y)) {
      // Check ESDF > inscribed radius for safety
      if (map_->getEsdf(x, y) > 0.0) {
        nodes_.push_back(Eigen::Vector2d(x, y));
      }
    }
  }

  int n = static_cast<int>(nodes_.size());
  adjacency_.resize(n);

  // KNN connection
  for (int i = 0; i < n; ++i) {
    // Compute distances to all other nodes
    std::vector<std::pair<double, int>> dists;
    dists.reserve(n);
    for (int j = 0; j < n; ++j) {
      if (i == j) continue;
      double d = (nodes_[i] - nodes_[j]).norm();
      dists.emplace_back(d, j);
    }
    std::sort(dists.begin(), dists.end());

    int connections = 0;
    for (const auto& p : dists) {
      if (connections >= knn_) break;
      int j = p.second;
      if (lineCollisionFree(nodes_[i], nodes_[j])) {
        adjacency_[i].push_back(j);
        connections++;
      }
    }
  }
}

std::vector<TopoPath> TopologyPlanner::searchPaths() {
  // DFS-based multi-path search from node 0 (start) to node 1 (goal)
  std::vector<TopoPath> result;
  if (nodes_.size() < 2) return result;

  struct DFSState {
    int node;
    std::vector<int> path;
  };

  std::vector<DFSState> stack;
  stack.push_back({0, {0}});

  int max_iterations = 50000;
  int iter = 0;

  while (!stack.empty() && iter < max_iterations &&
         static_cast<int>(result.size()) < max_paths_) {
    ++iter;
    DFSState cur = stack.back();
    stack.pop_back();

    if (cur.node == 1) {
      // Found a path to goal
      TopoPath tp;
      for (int idx : cur.path) {
        tp.points.push_back(nodes_[idx]);
      }
      tp.computeLength();

      if (isTopologicallyDistinct(tp, result)) {
        result.push_back(tp);
      }
      continue;
    }

    // Limit path length to avoid infinite loops
    if (static_cast<int>(cur.path.size()) > static_cast<int>(nodes_.size()) / 2) continue;

    for (int next : adjacency_[cur.node]) {
      // Check not already visited
      bool visited = false;
      for (int v : cur.path) {
        if (v == next) { visited = true; break; }
      }
      if (visited) continue;

      std::vector<int> new_path = cur.path;
      new_path.push_back(next);
      stack.push_back({next, new_path});
    }
  }

  return result;
}

void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  // Algorithm 1: Geometry-aware path shortcut
  for (auto& path : paths) {
    if (path.points.size() <= 2) continue;

    std::vector<Eigen::Vector2d> shortened;
    shortened.push_back(path.points.front());

    size_t i = 0;
    while (i < path.points.size() - 1) {
      // Find farthest visible point
      size_t farthest = i + 1;
      for (size_t j = path.points.size() - 1; j > i + 1; --j) {
        if (lineCollisionFree(path.points[i], path.points[j])) {
          farthest = j;
          break;
        }
      }
      shortened.push_back(path.points[farthest]);
      i = farthest;
    }

    path.points = shortened;
    path.computeLength();
  }
}

bool TopologyPlanner::lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  double dist = (b - a).norm();
  double step = map_->resolution() * 0.5;
  int n_steps = static_cast<int>(std::ceil(dist / step));
  if (n_steps == 0) return true;

  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / n_steps;
    Eigen::Vector2d p = a + t * (b - a);
    GridIndex gi = map_->worldToGrid(p.x(), p.y());
    if (map_->isOccupied(gi.x, gi.y)) return false;
  }
  return true;
}

bool TopologyPlanner::isTopologicallyDistinct(const TopoPath& path,
                                               const std::vector<TopoPath>& existing) const {
  // UVD-based topological equivalence check (simplified)
  // Two paths are topologically equivalent if the region between them contains no obstacles
  for (const auto& ep : existing) {
    // Sample points along both paths and check if the "band" between them is obstacle-free
    bool equivalent = true;
    int n_samples = 20;

    for (int i = 0; i <= n_samples; ++i) {
      double t = static_cast<double>(i) / n_samples;

      // Interpolate along each path by arc length fraction
      auto interpolate = [&](const TopoPath& tp) -> Eigen::Vector2d {
        double target = t * tp.length;
        double acc = 0.0;
        for (size_t j = 1; j < tp.points.size(); ++j) {
          double seg = (tp.points[j] - tp.points[j - 1]).norm();
          if (acc + seg >= target) {
            double frac = (seg > 1e-9) ? (target - acc) / seg : 0.0;
            return tp.points[j - 1] + frac * (tp.points[j] - tp.points[j - 1]);
          }
          acc += seg;
        }
        return tp.points.back();
      };

      Eigen::Vector2d p1 = interpolate(path);
      Eigen::Vector2d p2 = interpolate(ep);

      if (!lineCollisionFree(p1, p2)) {
        equivalent = false;
        break;
      }
    }

    if (equivalent) return false;  // Not distinct
  }
  return true;
}

}  // namespace esv_planner
