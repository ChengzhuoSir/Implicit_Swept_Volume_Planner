#include "esv_planner/topology_planner.h"
#include <algorithm>
#include <queue>
#include <set>
#include <cmath>
#include <functional>
#include <numeric>
#include <unordered_set>
#include <cassert>
#include <string>

namespace esv_planner {

// ---------------------------------------------------------------------------
// Halton sequence helper (base 2 and base 3 for 2-D quasi-random sampling)
// ---------------------------------------------------------------------------
static double haltonSequence(int index, int base) {
  double result = 0.0;
  double f = 1.0 / base;
  int i = index;
  while (i > 0) {
    result += f * (i % base);
    i /= base;
    f /= base;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Construction / initialisation
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// buildRoadmap  --  Halton quasi-random PRM
// ---------------------------------------------------------------------------
void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal) {
  nodes_.clear();
  adjacency_.clear();

  // Node 0 = start, Node 1 = goal
  nodes_.push_back(start);
  nodes_.push_back(goal);

  double x_min = map_->originX();
  double y_min = map_->originY();
  double x_max = x_min + map_->width() * map_->resolution();
  double y_max = y_min + map_->height() * map_->resolution();

  // Halton sequence (bases 2 and 3) for low-discrepancy sampling
  int accepted = 0;
  for (int i = 1; accepted < num_samples_; ++i) {
    double hx = haltonSequence(i, 2);
    double hy = haltonSequence(i, 3);
    double x = x_min + hx * (x_max - x_min);
    double y = y_min + hy * (y_max - y_min);

    double esdf = map_->getEsdf(x, y);
    if (esdf > inscribed_radius_) {
      nodes_.push_back(Eigen::Vector2d(x, y));
      ++accepted;
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

// ---------------------------------------------------------------------------
// Dijkstra  --  single-source shortest path on the PRM graph
// ---------------------------------------------------------------------------
TopoPath TopologyPlanner::dijkstra(int src, int dst,
                                    const std::vector<double>& edge_penalty) const {
  int n = static_cast<int>(nodes_.size());
  std::vector<double> dist(n, kInf);
  std::vector<int> prev(n, -1);
  dist[src] = 0.0;

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

      if (!edge_penalty.empty()) {
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

// ---------------------------------------------------------------------------
// dijkstraFull  --  returns (cost, prev-array) so Yen's can splice paths
// ---------------------------------------------------------------------------
static std::pair<double, std::vector<int>>
dijkstraFull(int src, int dst, int n,
             const std::vector<std::vector<std::pair<int, double>>>& adj,
             const std::set<std::pair<int,int>>& blocked_edges,
             const std::unordered_set<int>& blocked_nodes) {
  std::vector<double> dist(n, kInf);
  std::vector<int> prev(n, -1);
  dist[src] = 0.0;

  using PII = std::pair<double, int>;
  std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;
  pq.push({0.0, src});

  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();

    if (d > dist[u]) continue;
    if (u == dst) break;

    for (const auto& edge : adj[u]) {
      int v = edge.first;
      double w = edge.second;

      if (blocked_nodes.count(v) && v != dst) continue;
      if (blocked_edges.count({u, v})) continue;

      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  return {dist[dst], prev};
}

static std::vector<int> reconstructIndices(const std::vector<int>& prev, int src, int dst) {
  if (src < 0 || dst < 0 || src >= static_cast<int>(prev.size()) ||
      dst >= static_cast<int>(prev.size())) {
    return {};
  }
  if (src == dst) return {src};
  if (prev[dst] == -1) return {};  // unreachable

  std::vector<int> path;
  for (int v = dst; v != -1; v = prev[v]) {
    path.push_back(v);
    if (v == src) break;
  }
  if (path.empty() || path.back() != src) return {};
  std::reverse(path.begin(), path.end());
  return path;
}

static double pathCostFromAdj(const std::vector<int>& idx_path,
                              const std::vector<std::vector<std::pair<int, double>>>& adj) {
  if (idx_path.size() <= 1) return 0.0;
  double total = 0.0;
  for (size_t i = 1; i < idx_path.size(); ++i) {
    int u = idx_path[i - 1];
    int v = idx_path[i];
    double w = kInf;
    for (const auto& e : adj[u]) {
      if (e.first == v) {
        w = e.second;
        break;
      }
    }
    if (!std::isfinite(w)) return kInf;
    total += w;
  }
  return total;
}

// ---------------------------------------------------------------------------
// searchPaths  --  Dijkstra + progressive node penalty for homotopy diversity
// ---------------------------------------------------------------------------
std::vector<TopoPath> TopologyPlanner::searchPaths() {
  std::vector<TopoPath> result;
  if (nodes_.size() < 2) return result;

  const int n = static_cast<int>(nodes_.size());
  const int src = 0;
  const int dst = 1;

  struct CandidatePath {
    std::vector<int> indices;
    double cost = kInf;
    bool operator>(const CandidatePath& o) const {
      if (cost != o.cost) return cost > o.cost;
      return indices.size() > o.indices.size();
    }
  };

  std::vector<CandidatePath> A;  // accepted K-shortest index paths
  std::priority_queue<CandidatePath, std::vector<CandidatePath>,
                      std::greater<CandidatePath>> B;  // candidates
  std::unordered_set<std::string> seen;

  auto keyOf = [](const std::vector<int>& p) {
    std::string k;
    k.reserve(p.size() * 6);
    for (size_t i = 0; i < p.size(); ++i) {
      if (i > 0) k.push_back('-');
      k += std::to_string(p[i]);
    }
    return k;
  };

  auto pushCandidate = [&](const std::vector<int>& idx_path) {
    if (idx_path.size() < 2) return;
    double c = pathCostFromAdj(idx_path, adjacency_);
    if (!std::isfinite(c)) return;
    std::string key = keyOf(idx_path);
    if (!seen.insert(key).second) return;
    B.push(CandidatePath{idx_path, c});
  };

  // 1) First shortest path
  {
    const std::set<std::pair<int, int>> blocked_edges;
    const std::unordered_set<int> blocked_nodes;
    auto d = dijkstraFull(src, dst, n, adjacency_, blocked_edges, blocked_nodes);
    if (d.first >= kInf) return result;
    std::vector<int> shortest = reconstructIndices(d.second, src, dst);
    if (shortest.size() < 2) return result;
    A.push_back(CandidatePath{shortest, d.first});
  }

  // 2) Yen's algorithm
  for (int k = 1; k < max_paths_; ++k) {
    const std::vector<int>& prev_path = A[k - 1].indices;
    if (prev_path.size() < 2) break;

    for (size_t i = 0; i + 1 < prev_path.size(); ++i) {
      int spur_node = prev_path[i];
      std::vector<int> root_path(prev_path.begin(), prev_path.begin() + i + 1);

      std::set<std::pair<int, int>> blocked_edges;
      std::unordered_set<int> blocked_nodes;

      // Block edges that would reproduce previous paths sharing this root.
      for (const auto& accepted : A) {
        if (accepted.indices.size() <= i) continue;
        bool same_root = true;
        for (size_t r = 0; r <= i; ++r) {
          if (accepted.indices[r] != root_path[r]) {
            same_root = false;
            break;
          }
        }
        if (same_root && i + 1 < accepted.indices.size()) {
          blocked_edges.insert({accepted.indices[i], accepted.indices[i + 1]});
        }
      }

      // Block root nodes (except spur node) to avoid loops.
      for (size_t r = 0; r + 1 < root_path.size(); ++r) {
        blocked_nodes.insert(root_path[r]);
      }

      auto spur = dijkstraFull(spur_node, dst, n, adjacency_, blocked_edges, blocked_nodes);
      if (spur.first >= kInf) continue;

      std::vector<int> spur_path = reconstructIndices(spur.second, spur_node, dst);
      if (spur_path.size() < 2) continue;

      std::vector<int> total = root_path;
      total.insert(total.end(), spur_path.begin() + 1, spur_path.end());
      pushCandidate(total);
    }

    if (B.empty()) break;
    A.push_back(B.top());
    B.pop();
  }

  // 3) Convert to geometric paths and keep topologically distinct ones.
  for (const auto& cand : A) {
    TopoPath path;
    path.points.reserve(cand.indices.size());
    for (int idx : cand.indices) {
      if (idx < 0 || idx >= n) {
        path.points.clear();
        break;
      }
      path.points.push_back(nodes_[idx]);
    }
    if (path.points.size() < 2) continue;
    path.computeLength();
    if (isTopologicallyDistinct(path, result)) {
      result.push_back(path);
    }
    if (static_cast<int>(result.size()) >= max_paths_) break;
  }

  return result;
}

// ---------------------------------------------------------------------------
// lineCollisionFree  --  half-resolution ESDF check
// ---------------------------------------------------------------------------
bool TopologyPlanner::lineCollisionFree(const Eigen::Vector2d& a,
                                         const Eigen::Vector2d& b) const {
  double dist = (b - a).norm();
  double step = map_->resolution() * 0.5;
  int n_steps = static_cast<int>(std::ceil(dist / step));
  if (n_steps == 0) return true;

  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / n_steps;
    Eigen::Vector2d p = a + t * (b - a);
    if (map_->getEsdf(p.x(), p.y()) < inscribed_radius_) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// findCollisionPoint  --  binary-search for the first collision along a→b
// Returns the parametric t in [0,1] where ESDF first drops below threshold.
// ---------------------------------------------------------------------------
static double findCollisionT(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                              const GridMap& map, double threshold) {
  double dist = (b - a).norm();
  double step = map.resolution() * 0.5;
  int n_steps = std::max(1, static_cast<int>(std::ceil(dist / step)));

  // Linear scan to find the first colliding sample
  double t_col = -1.0;
  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / n_steps;
    Eigen::Vector2d p = a + t * (b - a);
    if (map.getEsdf(p.x(), p.y()) < threshold) {
      t_col = t;
      break;
    }
  }
  if (t_col < 0.0) return -1.0;  // no collision

  // Refine with binary search between previous safe sample and t_col
  double t_lo = std::max(0.0, t_col - 1.0 / n_steps);
  double t_hi = t_col;
  for (int iter = 0; iter < 16; ++iter) {
    double t_mid = 0.5 * (t_lo + t_hi);
    Eigen::Vector2d p = a + t_mid * (b - a);
    if (map.getEsdf(p.x(), p.y()) < threshold) {
      t_hi = t_mid;
    } else {
      t_lo = t_mid;
    }
  }
  return 0.5 * (t_lo + t_hi);
}

// ---------------------------------------------------------------------------
// pushPointFromObstacle  --  ESDF gradient ascent until safe
// ---------------------------------------------------------------------------
bool TopologyPlanner::pushPointFromObstacle(Eigen::Vector2d& pt,
                                             double safe_dist) const {
  double esdf = map_->getEsdf(pt.x(), pt.y());
  if (esdf >= safe_dist) return true;

  double eps = map_->resolution();
  for (int attempt = 0; attempt < 20; ++attempt) {
    double ex_p = map_->getEsdf(pt.x() + eps, pt.y());
    double ex_n = map_->getEsdf(pt.x() - eps, pt.y());
    double ey_p = map_->getEsdf(pt.x(), pt.y() + eps);
    double ey_n = map_->getEsdf(pt.x(), pt.y() - eps);
    double dx = ex_p - ex_n;
    double dy = ey_p - ey_n;
    double grad_norm = std::sqrt(dx * dx + dy * dy);

    if (grad_norm < 1e-9) return false;

    // Step size: remaining distance to safe boundary, clamped
    double deficit = safe_dist - esdf;
    double push = std::min(std::max(eps, deficit), 3.0 * eps);
    pt.x() += push * dx / grad_norm;
    pt.y() += push * dy / grad_norm;

    esdf = map_->getEsdf(pt.x(), pt.y());
    if (esdf >= safe_dist) return true;
  }
  return esdf > 0.0;
}

// ---------------------------------------------------------------------------
// discretizePath  --  resample a path at uniform spacing
// ---------------------------------------------------------------------------
static std::vector<Eigen::Vector2d> discretizePath(
    const std::vector<Eigen::Vector2d>& pts, double resolution) {
  if (pts.size() <= 1) return pts;

  std::vector<Eigen::Vector2d> out;
  out.push_back(pts.front());

  double accum = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    Eigen::Vector2d dir = pts[i] - pts[i - 1];
    double seg_len = dir.norm();
    if (seg_len < 1e-12) continue;
    Eigen::Vector2d unit = dir / seg_len;

    double remaining = seg_len;
    Eigen::Vector2d cursor = pts[i - 1];

    // If there is leftover from the previous segment, consume it first
    if (accum > 0.0) {
      double needed = resolution - accum;
      if (needed <= remaining) {
        cursor = cursor + needed * unit;
        remaining -= needed;
        out.push_back(cursor);
        accum = 0.0;
      } else {
        accum += remaining;
        continue;
      }
    }

    while (remaining >= resolution) {
      cursor = cursor + resolution * unit;
      remaining -= resolution;
      out.push_back(cursor);
    }
    accum = remaining;
  }

  // Always include the last point
  if ((out.back() - pts.back()).norm() > 1e-9) {
    out.push_back(pts.back());
  }
  return out;
}

// ---------------------------------------------------------------------------
// shortenPaths  --  Algorithm 1: Path Shortcut (precise paper version)
//
//  1. Discretize path into waypoints at resolution spacing
//  2. For each pair (p_i, p_j) where j > i+1, check visibility
//  3. If visible, shortcut: remove intermediate points
//  4. If NOT visible, find collision point p_c on line p_i -> p_j
//  5. Push p_c away from obstacle using ESDF gradient until safe
//  6. Replace segment with p_i -> p_c_pushed -> p_j
//  7. Iterate until no more shortcuts possible
// ---------------------------------------------------------------------------
void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  double safe_dist = inscribed_radius_;
  double res = map_->resolution();

  for (auto& path : paths) {
    if (path.points.size() <= 2) continue;

    // Step 1: discretize at map resolution
    path.points = discretizePath(path.points, res);

    // Iterate shortcutting until convergence
    bool changed = true;
    int max_iters = 20;
    for (int iter = 0; iter < max_iters && changed; ++iter) {
      changed = false;

      std::vector<Eigen::Vector2d> result;
      result.push_back(path.points.front());

      size_t i = 0;
      while (i < path.points.size() - 1) {
        // Step 2: try to find the farthest visible point from p_i
        bool shortcut_found = false;
        for (size_t j = path.points.size() - 1; j > i + 1; --j) {
          if (lineCollisionFree(path.points[i], path.points[j])) {
            // Step 3: visible -- shortcut, skip intermediate points
            result.push_back(path.points[j]);
            i = j;
            shortcut_found = true;
            changed = true;
            break;
          } else {
            // Step 4: find collision point on line p_i -> p_j
            double t_col = findCollisionT(path.points[i], path.points[j],
                                           *map_, safe_dist);
            if (t_col < 0.0) continue;  // shouldn't happen, but guard

            Eigen::Vector2d p_c = path.points[i] +
                                  t_col * (path.points[j] - path.points[i]);

            // Step 5: push p_c away from obstacle
            Eigen::Vector2d p_c_pushed = p_c;
            bool pushed = pushPointFromObstacle(p_c_pushed, safe_dist);
            if (!pushed) continue;

            // Step 6: verify both sub-segments are collision-free
            if (lineCollisionFree(path.points[i], p_c_pushed) &&
                lineCollisionFree(p_c_pushed, path.points[j])) {
              result.push_back(p_c_pushed);
              result.push_back(path.points[j]);
              i = j;
              shortcut_found = true;
              changed = true;
              break;
            }
            // If the pushed shortcut doesn't work for this j, try smaller j
          }
        }

        if (!shortcut_found) {
          // No shortcut from p_i -- advance one step
          ++i;
          if (i < path.points.size()) {
            result.push_back(path.points[i]);
          }
        }
      }

      path.points = result;
    }

    // Final safety pass: push any waypoint that is too close to obstacles
    for (size_t i = 1; i + 1 < path.points.size(); ++i) {
      double esdf = map_->getEsdf(path.points[i].x(), path.points[i].y());
      if (esdf < safe_dist) {
        pushPointFromObstacle(path.points[i], safe_dist);
      }
    }

    path.computeLength();
  }
}

// ---------------------------------------------------------------------------
// isTopologicallyDistinct  --  UVD with 40 sample points
//
// Two paths are topologically equivalent iff the connecting segments between
// corresponding arc-length-parameterised points are all collision-free.
// ---------------------------------------------------------------------------
bool TopologyPlanner::isTopologicallyDistinct(
    const TopoPath& path, const std::vector<TopoPath>& existing) const {
  const int n_samples = 40;

  // Arc-length interpolation lambda
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

  for (const auto& ep : existing) {
    bool equivalent = true;

    for (int i = 0; i <= n_samples; ++i) {
      double t = static_cast<double>(i) / n_samples;

      Eigen::Vector2d p1 = interpolate(path, t);
      Eigen::Vector2d p2 = interpolate(ep, t);

      double seg_dist = (p2 - p1).norm();
      if (seg_dist < 1e-6) continue;

      // Check the connecting segment for obstacle intersection
      int check_steps = std::max(2, static_cast<int>(
          std::ceil(seg_dist / (map_->resolution() * 0.5))));
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

    if (equivalent) return false;  // same homotopy class as an existing path
  }
  return true;
}

}  // namespace esv_planner
