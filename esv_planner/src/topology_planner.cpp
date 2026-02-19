#include "esv_planner/topology_planner.h"
#include <algorithm>
#include <random>
#include <queue>
#include <set>
#include <cmath>
#include <functional>

namespace esv_planner {

TopologyPlanner::TopologyPlanner() {}

void TopologyPlanner::init(const GridMap& map, const CollisionChecker& checker,
                           int num_samples, int knn, int max_paths,
                           double inscribed_radius) {
  map_ = &map;
  checker_ = &checker;
  num_samples_ = num_samples;
  knn_ = knn;
  max_paths_ = max_paths;
  inscribed_radius_ = inscribed_radius;
}

void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
  nodes_.clear();
  adjacency_.clear();

  // Node 0 = start, Node 1 = goal
  nodes_.push_back(start);
  nodes_.push_back(goal);

  // Random sampling in free space (inflated by inscribed radius)
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
    // Only accept samples with ESDF > inscribed_radius (safe for point robot)
    double esdf = map_->getEsdf(x, y);
    if (esdf > inscribed_radius_) {
      nodes_.push_back(Eigen::Vector2d(x, y));
    }
  }

  int n = static_cast<int>(nodes_.size());
  adjacency_.resize(n);

  // KNN connection with collision-free edges
  for (int i = 0; i < n; ++i) {
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
        adjacency_[i].emplace_back(j, p.first);
        connections++;
      }
    }
  }
}

TopoPath TopologyPlanner::dijkstra(int src, int dst,
                                    const std::vector<double>& edge_penalty) const {
  int n = static_cast<int>(nodes_.size());
  std::vector<double> dist(n, kInf);
  std::vector<int> prev(n, -1);
  dist[src] = 0.0;

  // min-heap: (distance, node)
  using PII = std::pair<double, int>;
  std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;
  pq.push({0.0, src});

  while (!pq.empty()) {
    auto top = pq.top();
    pq.pop();
    double d = top.first;
    int u = top.second;

    if (d > dist[u]) continue;
    if (u == dst) break;

    for (const auto& edge : adjacency_[u]) {
      int v = edge.first;
      double w = edge.second;

      // Apply edge penalty if provided (for finding diverse paths)
      if (!edge_penalty.empty()) {
        // Penalty index: encode edge as u*n + v
        // We use a simpler approach: penalize edges near already-found paths
        w += edge_penalty[v];
      }

      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  TopoPath result;
  if (dist[dst] >= kInf) return result;

  // Reconstruct path
  std::vector<int> path_indices;
  for (int v = dst; v != -1; v = prev[v]) {
    path_indices.push_back(v);
  }
  std::reverse(path_indices.begin(), path_indices.end());

  for (int idx : path_indices) {
    result.points.push_back(nodes_[idx]);
  }
  result.computeLength();
  return result;
}

std::vector<TopoPath> TopologyPlanner::searchPaths() {
  std::vector<TopoPath> result;
  if (nodes_.size() < 2) return result;

  int n = static_cast<int>(nodes_.size());

  // Find multiple topologically distinct paths using Dijkstra + node penalty
  std::vector<double> node_penalty(n, 0.0);
  double penalty_increment = 2.0;  // penalize nodes used by previous paths

  for (int attempt = 0; attempt < max_paths_ * 3 &&
       static_cast<int>(result.size()) < max_paths_; ++attempt) {

    TopoPath path = dijkstra(0, 1, node_penalty);
    if (path.points.empty()) break;

    if (isTopologicallyDistinct(path, result)) {
      result.push_back(path);
    }

    // Penalize nodes along this path to encourage diverse routes
    // Map path points back to node indices
    for (const auto& pt : path.points) {
      for (int i = 0; i < n; ++i) {
        if ((nodes_[i] - pt).norm() < 1e-6) {
          node_penalty[i] += penalty_increment;
          break;
        }
      }
    }
  }

  // Sort by path length
  std::sort(result.begin(), result.end(),
            [](const TopoPath& a, const TopoPath& b) { return a.length < b.length; });

  return result;
}

void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  // Algorithm 1: Geometry-aware path shortcut with ESDF push-off
  double safe_dist = inscribed_radius_;

  for (auto& path : paths) {
    if (path.points.size() <= 2) continue;

    // Iterative shortcutting
    for (int round = 0; round < 3; ++round) {
      std::vector<Eigen::Vector2d> shortened;
      shortened.push_back(path.points.front());

      size_t i = 0;
      while (i < path.points.size() - 1) {
        // Find farthest visible point from current
        size_t farthest = i + 1;
        for (size_t j = path.points.size() - 1; j > i + 1; --j) {
          if (lineCollisionFree(path.points[i], path.points[j])) {
            farthest = j;
            break;
          }
        }

        // If we can't skip any points, check if the collision point
        // can be pushed away from obstacles (Algorithm 1 push-off)
        if (farthest == i + 1 && i + 2 < path.points.size()) {
          // Try to push the intermediate point away from obstacle
          Eigen::Vector2d mid = 0.5 * (path.points[i] + path.points[i + 2]);
          if (pushPointFromObstacle(mid, safe_dist)) {
            // Check if the shortcut via pushed point is collision-free
            if (lineCollisionFree(path.points[i], mid) &&
                lineCollisionFree(mid, path.points[i + 2])) {
              shortened.push_back(mid);
              i = i + 2;
              continue;
            }
          }
        }

        shortened.push_back(path.points[farthest]);
        i = farthest;
      }

      path.points = shortened;
    }

    // Final pass: push all waypoints to safe distance from obstacles
    for (size_t i = 1; i + 1 < path.points.size(); ++i) {
      double esdf = map_->getEsdf(path.points[i].x(), path.points[i].y());
      if (esdf < safe_dist) {
        pushPointFromObstacle(path.points[i], safe_dist);
      }
    }

    path.computeLength();
  }
}

bool TopologyPlanner::pushPointFromObstacle(Eigen::Vector2d& pt, double safe_dist) const {
  // Push point away from nearest obstacle using ESDF gradient
  double esdf = map_->getEsdf(pt.x(), pt.y());
  if (esdf >= safe_dist) return true;  // already safe

  double eps = map_->resolution();
  for (int attempt = 0; attempt < 10; ++attempt) {
    // Compute ESDF gradient via finite difference
    double dx = map_->getEsdf(pt.x() + eps, pt.y()) - map_->getEsdf(pt.x() - eps, pt.y());
    double dy = map_->getEsdf(pt.x(), pt.y() + eps) - map_->getEsdf(pt.x(), pt.y() - eps);
    double grad_norm = std::sqrt(dx * dx + dy * dy);

    if (grad_norm < 1e-9) return false;

    // Push in gradient direction
    double push = std::max(eps, safe_dist - esdf);
    pt.x() += push * dx / grad_norm;
    pt.y() += push * dy / grad_norm;

    esdf = map_->getEsdf(pt.x(), pt.y());
    if (esdf >= safe_dist) return true;
  }
  return esdf > 0.0;  // at least not inside obstacle
}

bool TopologyPlanner::lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  // Check line segment against inflated occupancy grid
  double dist = (b - a).norm();
  double step = map_->resolution() * 0.5;
  int n_steps = static_cast<int>(std::ceil(dist / step));
  if (n_steps == 0) return true;

  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / n_steps;
    Eigen::Vector2d p = a + t * (b - a);
    // Use ESDF for more accurate check with inscribed radius
    if (map_->getEsdf(p.x(), p.y()) < inscribed_radius_) return false;
  }
  return true;
}

bool TopologyPlanner::isTopologicallyDistinct(const TopoPath& path,
                                               const std::vector<TopoPath>& existing) const {
  // UVD-based topological equivalence check
  // Two paths are topologically equivalent if the region between them
  // (connecting corresponding points) contains no obstacles
  for (const auto& ep : existing) {
    bool equivalent = true;
    int n_samples = 30;

    for (int i = 0; i <= n_samples; ++i) {
      double t = static_cast<double>(i) / n_samples;

      // Interpolate along each path by arc length fraction
      auto interpolate = [](const TopoPath& tp, double frac) -> Eigen::Vector2d {
        if (tp.points.empty()) return Eigen::Vector2d::Zero();
        if (tp.length < 1e-9) return tp.points.front();
        double target = frac * tp.length;
        double acc = 0.0;
        for (size_t j = 1; j < tp.points.size(); ++j) {
          double seg = (tp.points[j] - tp.points[j - 1]).norm();
          if (acc + seg >= target) {
            double f = (seg > 1e-9) ? (target - acc) / seg : 0.0;
            return tp.points[j - 1] + f * (tp.points[j] - tp.points[j - 1]);
          }
          acc += seg;
        }
        return tp.points.back();
      };

      Eigen::Vector2d p1 = interpolate(path, t);
      Eigen::Vector2d p2 = interpolate(ep, t);

      // Check if the line between corresponding points crosses an obstacle
      double seg_dist = (p2 - p1).norm();
      if (seg_dist < 1e-6) continue;

      int check_steps = std::max(2, static_cast<int>(seg_dist / map_->resolution()));
      for (int s = 0; s <= check_steps; ++s) {
        double st = static_cast<double>(s) / check_steps;
        Eigen::Vector2d p = p1 + st * (p2 - p1);
        GridIndex gi = map_->worldToGrid(p.x(), p.y());
        if (map_->isOccupied(gi.x, gi.y)) {
          equivalent = false;
          break;
        }
      }
      if (!equivalent) break;
    }

    if (equivalent) return false;
  }
  return true;
}

}  // namespace esv_planner
