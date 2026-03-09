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

namespace {

const Eigen::Vector2d& waypointPos(const TopoWaypoint& wp) {
  return wp.pos;
}

void assignTangentYaw(std::vector<TopoWaypoint>& waypoints) {
  if (waypoints.empty()) return;
  if (waypoints.size() == 1) {
    waypoints.front().yaw = 0.0;
    waypoints.front().has_yaw = true;
    return;
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    Eigen::Vector2d dir = (i == 0)
        ? (waypoints[1].pos - waypoints[0].pos)
        : (waypoints[i].pos - waypoints[i - 1].pos);
    if (dir.norm() < 1e-9) {
      waypoints[i].yaw = (i > 0) ? waypoints[i - 1].yaw : 0.0;
    } else {
      waypoints[i].yaw = std::atan2(dir.y(), dir.x());
    }
    waypoints[i].has_yaw = true;
  }
}

double interpolateWaypointYaw(const TopoWaypoint& a, const TopoWaypoint& b, double t) {
  double yaw_a = a.has_yaw ? a.yaw : std::atan2((b.pos - a.pos).y(), (b.pos - a.pos).x());
  double yaw_b = b.has_yaw ? b.yaw : yaw_a;
  return normalizeAngle(yaw_a + normalizeAngle(yaw_b - yaw_a) * t);
}

TopoWaypoint withIncomingYaw(const TopoWaypoint& from, const TopoWaypoint& to) {
  TopoWaypoint out = to;
  Eigen::Vector2d dir = out.pos - from.pos;
  if (dir.norm() < 1e-9) {
    out.yaw = from.has_yaw ? from.yaw : 0.0;
  } else {
    out.yaw = std::atan2(dir.y(), dir.x());
  }
  out.has_yaw = true;
  return out;
}

Eigen::Vector2d esdfAscentDirection(const GridMap& map,
                                    const Eigen::Vector2d& world,
                                    double eps) {
  const double ex_p = map.getEsdf(world.x() + eps, world.y());
  const double ex_n = map.getEsdf(world.x() - eps, world.y());
  const double ey_p = map.getEsdf(world.x(), world.y() + eps);
  const double ey_n = map.getEsdf(world.x(), world.y() - eps);
  Eigen::Vector2d grad(ex_p - ex_n, ey_p - ey_n);
  if (grad.norm() > 1e-9) {
    return grad.normalized();
  }

  const double base = map.getEsdf(world.x(), world.y());
  Eigen::Vector2d best = Eigen::Vector2d::Zero();
  double best_gain = 0.0;
  for (int k = 0; k < 16; ++k) {
    const double theta = kTwoPi * static_cast<double>(k) / 16.0;
    const Eigen::Vector2d dir(std::cos(theta), std::sin(theta));
    for (double scale : {1.0, 2.0, 3.0}) {
      const double value = map.getEsdf(world.x() + scale * eps * dir.x(),
                                       world.y() + scale * eps * dir.y());
      const double gain = value - base;
      if (gain > best_gain) {
        best_gain = gain;
        best = dir;
      }
    }
  }
  return best;
}

}  // namespace

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
    result.waypoints.emplace_back(nodes_[idx]);
  }
  assignTangentYaw(result.waypoints);
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
  std::vector<double> node_penalty(static_cast<size_t>(n), 0.0);
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

  auto indicesFromPath = [&](const TopoPath& path) {
    std::vector<int> indices;
    indices.reserve(path.waypoints.size());
    for (const auto& wp : path.waypoints) {
      int best_idx = -1;
      double best_dist = kInf;
      for (int ni = 0; ni < n; ++ni) {
        double dist = (nodes_[ni] - wp.pos).norm();
        if (dist < best_dist) {
          best_dist = dist;
          best_idx = ni;
        }
      }
      if (best_idx >= 0 && best_dist < 1e-6) {
        indices.push_back(best_idx);
      } else {
        indices.clear();
        break;
      }
    }
    return indices;
  };

  for (int iter = 0; iter < max_paths_ * 6; ++iter) {
    TopoPath path = dijkstra(src, dst, node_penalty);
    if (path.waypoints.size() < 2) break;

    std::vector<int> indices = indicesFromPath(path);
    if (indices.size() < 2) break;

    if (seen.insert(keyOf(indices)).second) {
      result.push_back(path);
      if (static_cast<int>(result.size()) >= max_paths_) break;
    }

    for (size_t i = 1; i + 1 < indices.size(); ++i) {
      node_penalty[indices[i]] += std::max(0.5, path.length * 0.1);
    }
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
bool TopologyPlanner::pushPointFromObstacle(TopoWaypoint& wp,
                                             double safe_dist) const {
  const auto boundary = checker_->footprint().sampleBoundary(wp.yaw, map_->resolution() * 0.5);
  if (boundary.empty()) return false;

  double eps = map_->resolution();
  for (int attempt = 0; attempt < 24; ++attempt) {
    double min_dist = kInf;
    Eigen::Vector2d worst_world = wp.pos;

    for (const auto& sample : boundary) {
      Eigen::Vector2d world = wp.pos + sample;
      double dist = map_->getEsdf(world.x(), world.y());
      if (dist < min_dist) {
        min_dist = dist;
        worst_world = world;
      }
    }

    if (min_dist >= safe_dist && checker_->isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw))) {
      wp.has_yaw = true;
      return true;
    }

    Eigen::Vector2d grad = esdfAscentDirection(*map_, worst_world, eps);
    double grad_norm = grad.norm();
    if (grad_norm < 1e-9) {
      grad = esdfAscentDirection(*map_, wp.pos, eps);
      grad_norm = grad.norm();
    }
    if (grad_norm < 1e-9) return false;

    double deficit = safe_dist - min_dist;
    double push = std::min(std::max(eps, deficit + eps), 4.0 * eps);
    wp.pos += push * grad / grad_norm;
    wp.has_yaw = true;
  }

  return checker_->isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
}

bool TopologyPlanner::configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const {
  Eigen::Vector2d delta = b.pos - a.pos;
  double dist = delta.norm();
  int n_steps = std::max(2, static_cast<int>(std::ceil(dist / (map_->resolution() * 0.5))) + 1);
  double yaw = b.has_yaw ? b.yaw : std::atan2(delta.y(), delta.x());
  for (int i = 0; i < n_steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(n_steps - 1);
    Eigen::Vector2d p = a.pos + t * delta;
    if (!checker_->isFree(SE2State(p.x(), p.y(), yaw))) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// discretizePath  --  resample a path at uniform spacing
// ---------------------------------------------------------------------------
static std::vector<TopoWaypoint> discretizePath(
    const std::vector<TopoWaypoint>& pts, double resolution) {
  if (pts.size() <= 1) return pts;

  std::vector<TopoWaypoint> out;
  out.push_back(pts.front());

  double accum = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    Eigen::Vector2d dir = pts[i].pos - pts[i - 1].pos;
    double seg_len = dir.norm();
    if (seg_len < 1e-12) continue;
    Eigen::Vector2d unit = dir / seg_len;

    double remaining = seg_len;
    Eigen::Vector2d cursor = pts[i - 1].pos;

    // If there is leftover from the previous segment, consume it first
    if (accum > 0.0) {
      double needed = resolution - accum;
      if (needed <= remaining) {
        cursor = cursor + needed * unit;
        remaining -= needed;
        out.emplace_back(cursor);
        accum = 0.0;
      } else {
        accum += remaining;
        continue;
      }
    }

    while (remaining >= resolution) {
      cursor = cursor + resolution * unit;
      remaining -= resolution;
      out.emplace_back(cursor);
    }
    accum = remaining;
  }

  // Always include the last point
  if ((out.back().pos - pts.back().pos).norm() > 1e-9) {
    out.push_back(pts.back());
  }
  assignTangentYaw(out);
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
    if (path.waypoints.size() <= 2) continue;

    // Densify first so obstacle-adjacent segments are repaired at resolution scale.
    std::vector<TopoWaypoint> discrete = discretizePath(path.waypoints, res);
    assignTangentYaw(discrete);
    for (size_t k = 1; k + 1 < discrete.size(); ++k) {
      TopoWaypoint adjusted = withIncomingYaw(discrete[k - 1], discrete[k]);
      if (pushPointFromObstacle(adjusted, safe_dist)) {
        discrete[k] = withIncomingYaw(discrete[k - 1], adjusted);
      }
    }
    assignTangentYaw(discrete);

    std::vector<TopoWaypoint> result;
    result.push_back(discrete.front());

    size_t i = 0;
    int inserted_pushes = 0;
    const int max_inserted_pushes = static_cast<int>(discrete.size()) * 2;

    while (i + 1 < discrete.size()) {
      const TopoWaypoint anchor = result.back();

      size_t best_j = i + 1;
      bool found_long_shortcut = false;
      for (size_t j = discrete.size() - 1; j > i + 1; --j) {
        TopoWaypoint candidate = withIncomingYaw(anchor, discrete[j]);
        if (configurationLineFree(anchor, candidate)) {
          best_j = j;
          found_long_shortcut = true;
          break;
        }
      }

      if (found_long_shortcut) {
        result.push_back(withIncomingYaw(anchor, discrete[best_j]));
        i = best_j;
        continue;
      }

      const TopoWaypoint next = withIncomingYaw(anchor, discrete[i + 1]);
      if (configurationLineFree(anchor, next)) {
        result.push_back(next);
        i += 1;
        continue;
      }

      if (inserted_pushes >= max_inserted_pushes) {
        result.push_back(next);
        i += 1;
        continue;
      }

      const size_t target_j = std::min(discrete.size() - 1, i + 3);
      TopoWaypoint target = withIncomingYaw(anchor, discrete[target_j]);
      double t_col = findCollisionT(anchor.pos, target.pos, *map_, safe_dist);
      if (t_col < 0.0) {
        t_col = 0.5;
      }

      TopoWaypoint pushed_wp(anchor.pos + t_col * (target.pos - anchor.pos));
      pushed_wp = withIncomingYaw(anchor, pushed_wp);
      if (pushPointFromObstacle(pushed_wp, safe_dist) &&
          (pushed_wp = withIncomingYaw(anchor, pushed_wp), true) &&
          configurationLineFree(anchor, pushed_wp) &&
          (pushed_wp.pos - anchor.pos).norm() > res * 0.75) {
        result.push_back(pushed_wp);
        ++inserted_pushes;
        continue;
      }

      result.push_back(next);
      i += 1;
    }

    if ((result.back().pos - discrete.back().pos).norm() > 1e-6) {
      result.push_back(withIncomingYaw(result.back(), discrete.back()));
    }

    for (size_t k = 1; k + 1 < result.size(); ++k) {
      TopoWaypoint adjusted = withIncomingYaw(result[k - 1], result[k]);
      if (pushPointFromObstacle(adjusted, safe_dist)) {
        result[k] = withIncomingYaw(result[k - 1], adjusted);
      }
    }
    assignTangentYaw(result);

    // Final pruning with the segment-yaw semantics used downstream.
    for (size_t k = 1; k + 1 < result.size();) {
      TopoWaypoint merged = withIncomingYaw(result[k - 1], result[k + 1]);
      if (configurationLineFree(result[k - 1], merged)) {
        result.erase(result.begin() + static_cast<long>(k));
        assignTangentYaw(result);
        continue;
      }
      ++k;
    }

    path.waypoints = result;
    path.computeLength();
  }

  std::vector<TopoPath> filtered;
  filtered.reserve(paths.size());
  for (const auto& path : paths) {
    if (path.waypoints.size() < 2) continue;
    if (isTopologicallyDistinct(path, filtered)) {
      filtered.push_back(path);
    }
    if (static_cast<int>(filtered.size()) >= max_paths_) break;
  }
  paths = filtered;
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
    if (tp.waypoints.empty()) return Eigen::Vector2d::Zero();
    if (tp.length < 1e-9) return tp.waypoints.front().pos;
    double target = frac * tp.length;
    double acc = 0.0;
    for (size_t j = 1; j < tp.waypoints.size(); ++j) {
      double seg = (tp.waypoints[j].pos - tp.waypoints[j - 1].pos).norm();
      if (acc + seg >= target) {
        double f = (seg > 1e-9) ? (target - acc) / seg : 0.0;
        return tp.waypoints[j - 1].pos +
               f * (tp.waypoints[j].pos - tp.waypoints[j - 1].pos);
      }
      acc += seg;
    }
    return tp.waypoints.back().pos;
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
