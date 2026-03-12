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

void assignMissingYaw(std::vector<TopoWaypoint>& waypoints) {
  if (waypoints.empty()) return;
  if (waypoints.size() == 1) {
    if (!waypoints.front().has_yaw) {
      waypoints.front().yaw = 0.0;
      waypoints.front().has_yaw = true;
    }
    return;
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (waypoints[i].has_yaw) continue;
    Eigen::Vector2d dir = (i + 1 < waypoints.size())
        ? (waypoints[i + 1].pos - waypoints[i].pos)
        : (waypoints[i].pos - waypoints[i - 1].pos);
    if (dir.norm() < 1e-9) {
      waypoints[i].yaw = (i > 0 && waypoints[i - 1].has_yaw) ? waypoints[i - 1].yaw : 0.0;
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

TopoWaypoint withExistingOrIncomingYaw(const TopoWaypoint& from, const TopoWaypoint& to) {
  return to.has_yaw ? to : withIncomingYaw(from, to);
}

bool isOccupiedValue(int8_t val) {
  return val > 50 || val < 0;
}

Eigen::Vector2d rotateIntoBody(const Eigen::Vector2d& world_delta, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * world_delta.x() + s * world_delta.y(),
                         -s * world_delta.x() + c * world_delta.y());
}

Eigen::Vector2d rotateIntoWorld(const Eigen::Vector2d& body_vec, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * body_vec.x() - s * body_vec.y(),
                         s * body_vec.x() + c * body_vec.y());
}

std::vector<Eigen::Vector2d> collectLocalObstacleSamples(
    const GeometryMap& geometry_map,
    const Eigen::Vector2d& center,
    double radius) {
  const auto local_segments = geometry_map.queryLocalSegments(center, radius, 32);
  std::vector<std::pair<double, Eigen::Vector2d>> ranked;
  const double r2 = radius * radius;
  ranked.reserve(local_segments.size() * 3);

  auto pushCandidate = [&](const Eigen::Vector2d& world) {
    const double dist2 = (world - center).squaredNorm();
    if (dist2 <= r2) {
      ranked.emplace_back(dist2, world);
    }
  };

  for (const auto& segment : local_segments) {
    pushCandidate(geometry_map_detail::projectToSegment(center, segment));
    pushCandidate(segment.a);
    pushCandidate(segment.b);
  }

  std::sort(ranked.begin(), ranked.end(),
            [](const std::pair<double, Eigen::Vector2d>& lhs,
               const std::pair<double, Eigen::Vector2d>& rhs) {
              return lhs.first < rhs.first;
            });
  const size_t max_points = 96;
  std::vector<Eigen::Vector2d> samples;
  samples.reserve(std::min(max_points, ranked.size()));
  const double dedup_tol2 = 1e-8;
  for (size_t i = 0; i < ranked.size() && i < max_points; ++i) {
    bool duplicate = false;
    for (const auto& existing : samples) {
      if ((existing - ranked[i].second).squaredNorm() <= dedup_tol2) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      samples.push_back(ranked[i].second);
    }
  }
  return samples;
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
  evaluator_.initGridEsdf(map, checker.footprint());
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
  const FootprintModel& footprint = checker_->footprint();
  const double eps = map_->resolution();
  const double obstacle_extent = 0.5 * std::sqrt(2.0) * map_->resolution();
  const double obstacle_radius =
      footprint.circumscribedRadius() + safe_dist + 2.5 * map_->resolution();
  const double yaw_eps = 0.05;
  const double center_clearance = map_->getEsdf(wp.pos.x(), wp.pos.y());
  auto continuousFree = [&](const TopoWaypoint& candidate) {
    const GridIndex gi = map_->worldToGrid(candidate.pos.x(), candidate.pos.y());
    if (!map_->isInside(gi.x, gi.y)) {
      return false;
    }
    return evaluator_.evaluate(
               SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw)) >= 0.0;
  };
  if (center_clearance >= obstacle_radius &&
      continuousFree(wp)) {
    wp.has_yaw = true;
    return true;
  }

  struct ObstacleInfo {
    Eigen::Vector2d world = Eigen::Vector2d::Zero();
    BodyFrameQuery query;
    double effective_dist = kInf;
  };

  auto queryObstacleInfos = [&](const TopoWaypoint& candidate,
                                const std::vector<Eigen::Vector2d>& candidate_obstacles) {
    std::vector<ObstacleInfo> infos;
    infos.reserve(candidate_obstacles.size());
    if (candidate_obstacles.empty()) {
      return infos;
    }

    Eigen::MatrixXd body_points(static_cast<Eigen::Index>(candidate_obstacles.size()), 2);
    for (size_t i = 0; i < candidate_obstacles.size(); ++i) {
      body_points.row(static_cast<Eigen::Index>(i)) =
          rotateIntoBody(candidate_obstacles[i] - candidate.pos, candidate.yaw).transpose();
    }
    const auto queries = footprint.bodyFrameSdfModel().queryBatch(body_points);
    infos.resize(candidate_obstacles.size());
    for (size_t i = 0; i < candidate_obstacles.size(); ++i) {
      infos[i].world = candidate_obstacles[i];
      infos[i].query = queries[i];
      infos[i].effective_dist = queries[i].signed_distance - obstacle_extent;
    }
    return infos;
  };

  auto tryRepairYaw = [&](TopoWaypoint& candidate,
                          const std::vector<Eigen::Vector2d>& candidate_obstacles) {
    (void)candidate_obstacles;
    double best_clearance = -kInf;
    double best_yaw = candidate.yaw;
    bool found = false;
    constexpr int kYawSamples = 12;
    for (int k = 0; k < kYawSamples; ++k) {
      const double yaw =
          normalizeAngle(candidate.yaw + kTwoPi * static_cast<double>(k) /
                                             static_cast<double>(kYawSamples));
      const double clearance =
          evaluator_.evaluate(SE2State(candidate.pos.x(), candidate.pos.y(), yaw));
      if (!found || clearance > best_clearance) {
        found = true;
        best_clearance = clearance;
        best_yaw = yaw;
      }
      if (clearance >= 0.0) {
        best_yaw = yaw;
        best_clearance = clearance;
        found = true;
        break;
      }
    }
    if (found) {
      candidate.yaw = best_yaw;
      candidate.has_yaw = true;
    }
    return found;
  };

  auto evaluateWorstDistance = [&](const TopoWaypoint& candidate,
                                   const std::vector<Eigen::Vector2d>& candidate_obstacles) {
    double candidate_worst = kInf;
    const auto infos = queryObstacleInfos(candidate, candidate_obstacles);
    for (const auto& info : infos) {
      candidate_worst = std::min(candidate_worst, info.effective_dist);
    }
    return candidate_worst;
  };

  for (int attempt = 0; attempt < 24; ++attempt) {
    const auto obstacles =
        collectLocalObstacleSamples(map_->geometryMap(), wp.pos, obstacle_radius);
    if (obstacles.empty()) {
      return continuousFree(wp);
    }

    std::vector<ObstacleInfo> violating;
    violating.reserve(obstacles.size());
    double worst_dist = kInf;
    const auto infos = queryObstacleInfos(wp, obstacles);
    for (const auto& info : infos) {
      worst_dist = std::min(worst_dist, info.effective_dist);
      if (info.effective_dist < safe_dist) {
        violating.push_back(info);
      }
    }

    if (worst_dist >= safe_dist &&
        continuousFree(wp)) {
      wp.has_yaw = true;
      return true;
    }

    if (violating.empty()) {
      return continuousFree(wp);
    }

    std::sort(violating.begin(), violating.end(),
              [](const ObstacleInfo& lhs, const ObstacleInfo& rhs) {
                return lhs.effective_dist < rhs.effective_dist;
              });

    double objective = 0.0;
    Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
    double grad_yaw = 0.0;
    const size_t max_active = 16;
    const double active_band = worst_dist + 0.05;
    for (size_t i = 0; i < violating.size() && i < max_active; ++i) {
      const auto& info = violating[i];
      if (info.effective_dist > active_band) break;

      const double pen = safe_dist - info.effective_dist;
      objective += pen * pen;
      grad_pos += 2.0 * pen * rotateIntoWorld(info.query.gradient, wp.yaw);

      const Eigen::Vector2d body_plus =
          rotateIntoBody(info.world - wp.pos, wp.yaw + yaw_eps);
      const Eigen::Vector2d body_minus =
          rotateIntoBody(info.world - wp.pos, wp.yaw - yaw_eps);
      const double d_plus = footprint.bodyFrameSdf(body_plus);
      const double d_minus = footprint.bodyFrameSdf(body_minus);
      const double dsd_yaw = (d_plus - d_minus) / (2.0 * yaw_eps);
      grad_yaw += -2.0 * pen * dsd_yaw;
    }

    if (objective < 1e-12) {
      return continuousFree(wp);
    }

    Eigen::Vector2d strongest_grad = Eigen::Vector2d::Zero();
    if (!violating.empty()) {
      strongest_grad = rotateIntoWorld(violating.front().query.gradient, wp.yaw);
    }
    if (grad_pos.norm() < 1e-9 && strongest_grad.norm() > 1e-9) {
      grad_pos = strongest_grad;
    }
    if (grad_pos.norm() < 1e-9) {
      return false;
    }

    const Eigen::Vector2d dir = -grad_pos.normalized();
    const double step_pos =
        std::min(std::max(eps, 0.5 * (safe_dist - std::min(worst_dist, safe_dist))), 4.0 * eps);
    const double yaw_step =
        std::max(-0.12, std::min(0.12, -0.05 * grad_yaw));

    bool accepted = false;
    for (double scale : {1.0, 0.5, 0.25}) {
      TopoWaypoint candidate = wp;
      candidate.pos += scale * step_pos * dir;
      candidate.yaw = normalizeAngle(candidate.yaw + scale * yaw_step);
      candidate.has_yaw = true;

      const auto candidate_obstacles =
          collectLocalObstacleSamples(map_->geometryMap(), candidate.pos, obstacle_radius);
      const double candidate_worst =
          evaluateWorstDistance(candidate, candidate_obstacles);

      const bool candidate_free =
          checker_->isFree(SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw)) ||
          tryRepairYaw(candidate, candidate_obstacles);

      if (candidate_worst + 1e-6 >= worst_dist && candidate_free) {
        wp = candidate;
        accepted = true;
        break;
      }
    }

    if (!accepted) {
      return false;
    }
  }

  return continuousFree(wp);
}

bool TopologyPlanner::configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const {
  return configurationClearance(a, b) >= 0.0;
}

double TopologyPlanner::configurationClearance(const TopoWaypoint& a,
                                               const TopoWaypoint& b) const {
  const double yaw_a =
      a.has_yaw ? a.yaw : std::atan2((b.pos - a.pos).y(), (b.pos - a.pos).x());
  const double yaw_b =
      b.has_yaw ? b.yaw : std::atan2((b.pos - a.pos).y(), (b.pos - a.pos).x());
  const Eigen::Vector2d delta = b.pos - a.pos;
  const double len = delta.norm();
  const double yaw_delta = std::abs(normalizeAngle(yaw_b - yaw_a));
  const int linear_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(map_->resolution(), 0.10))));
  const int yaw_steps = std::max(
      1, static_cast<int>(std::ceil(yaw_delta / 0.10)));
  const int n_steps = std::max(linear_steps, yaw_steps);
  double min_clearance = kInf;
  for (int i = 0; i <= n_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_steps);
    SE2State sample;
    sample.x = a.pos.x() + delta.x() * t;
    sample.y = a.pos.y() + delta.y() * t;
    sample.yaw = normalizeAngle(yaw_a + normalizeAngle(yaw_b - yaw_a) * t);
    min_clearance = std::min(min_clearance, evaluator_.evaluate(sample));
  }
  return min_clearance;
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
  const double shortcut_res = std::max(res, 0.10);

  const size_t shorten_budget = 6;
  if (paths.size() > shorten_budget) {
    std::stable_sort(paths.begin(), paths.end(),
                     [](const TopoPath& lhs, const TopoPath& rhs) {
                       return lhs.length < rhs.length;
                     });
    paths.resize(shorten_budget);
  }

  for (auto& path : paths) {
    if (path.waypoints.size() <= 2) continue;
    const bool have_graph_endpoints = nodes_.size() >= 2;

    std::vector<TopoWaypoint> discrete = path.waypoints;
    bool chain_repaired = false;
    assignTangentYaw(discrete);
    for (size_t k = 1; k + 1 < discrete.size(); ++k) {
      TopoWaypoint adjusted = withIncomingYaw(discrete[k - 1], discrete[k]);
      const Eigen::Vector2d original_pos = adjusted.pos;
      const double original_yaw = adjusted.yaw;
      const double center_clearance = map_->getEsdf(adjusted.pos.x(), adjusted.pos.y());
      const SE2State adjusted_state(adjusted.pos.x(), adjusted.pos.y(), adjusted.yaw);
      const double body_clearance = evaluator_.evaluate(adjusted_state);
      const double coarse_threshold =
          checker_->footprint().circumscribedRadius() + safe_dist + map_->resolution();
      if ((center_clearance < coarse_threshold ||
           body_clearance < 0.0 ||
           !checker_->isFree(adjusted_state))) {
        for (int attempt = 0; attempt < 3; ++attempt) {
          if (!pushPointFromObstacle(adjusted, safe_dist)) {
            break;
          }
          const SE2State repaired_state(adjusted.pos.x(), adjusted.pos.y(), adjusted.yaw);
          if (checker_->isFree(repaired_state) &&
              evaluator_.evaluate(repaired_state) >= 0.0) {
            break;
          }
        }
        discrete[k] = adjusted;
        if ((adjusted.pos - original_pos).norm() > 1e-6 ||
            std::abs(normalizeAngle(adjusted.yaw - original_yaw)) > 1e-6) {
          chain_repaired = true;
        }
      }
    }

    std::vector<TopoWaypoint> dense_safe;
    dense_safe.reserve(discrete.size() * 2);
    dense_safe.push_back(discrete.front());

    std::function<bool(const TopoWaypoint&, const TopoWaypoint&, int)> append_bridge =
        [&](const TopoWaypoint& from, const TopoWaypoint& raw_target, int depth) -> bool {
      TopoWaypoint target = withExistingOrIncomingYaw(from, raw_target);
      if (configurationLineFree(from, target)) {
        dense_safe.push_back(target);
        return true;
      }

      TopoWaypoint pushed = target;
      if (pushPointFromObstacle(pushed, safe_dist) &&
          configurationLineFree(from, pushed)) {
        dense_safe.push_back(pushed);
        if ((pushed.pos - target.pos).norm() > 1e-6 ||
            std::abs(normalizeAngle(pushed.yaw - target.yaw)) > 1e-6) {
          chain_repaired = true;
        }
        return true;
      }

      if (depth >= 4 || (target.pos - from.pos).norm() < shortcut_res * 0.75) {
        return false;
      }

      TopoWaypoint mid(from.pos + 0.5 * (target.pos - from.pos));
      mid.yaw = interpolateWaypointYaw(from, target, 0.5);
      mid.has_yaw = true;
      if (!pushPointFromObstacle(mid, safe_dist)) {
        return false;
      }
      if (!configurationLineFree(from, mid)) {
        return false;
      }

      dense_safe.push_back(mid);
      chain_repaired = true;
      return append_bridge(mid, target, depth + 1);
    };

    for (size_t k = 1; k < discrete.size(); ++k) {
      if (!append_bridge(dense_safe.back(), discrete[k], 0)) {
        dense_safe.clear();
        break;
      }
    }

    if (dense_safe.size() >= 2) {
      discrete = std::move(dense_safe);
      if (shortcut_res > res + 1e-9) {
        discrete = discretizePath(discrete, shortcut_res);
        assignMissingYaw(discrete);
      }
    }

    auto finalizeChain = [&](std::vector<TopoWaypoint> result,
                             const std::vector<TopoWaypoint>& reference_tail) {
      std::function<bool(const TopoWaypoint&, const TopoWaypoint&, int,
                         std::vector<TopoWaypoint>&)> buildBridgeChain =
          [&](const TopoWaypoint& from, const TopoWaypoint& raw_target, int depth,
              std::vector<TopoWaypoint>& bridge) -> bool {
        TopoWaypoint target = withExistingOrIncomingYaw(from, raw_target);
        if (configurationLineFree(from, target)) {
          bridge.push_back(target);
          return true;
        }

        if (depth >= 3 || (target.pos - from.pos).norm() < shortcut_res * 0.75) {
          return false;
        }

        TopoWaypoint pushed = target;
        if (pushPointFromObstacle(pushed, safe_dist) &&
            configurationLineFree(from, pushed)) {
          bridge.push_back(pushed);
          return true;
        }

        TopoWaypoint mid(from.pos + 0.5 * (target.pos - from.pos));
        mid.yaw = interpolateWaypointYaw(from, target, 0.5);
        mid.has_yaw = true;
        if (!pushPointFromObstacle(mid, safe_dist) ||
            !configurationLineFree(from, mid)) {
          return false;
        }
        bridge.push_back(mid);
        return buildBridgeChain(mid, target, depth + 1, bridge);
      };

      auto tryRepairEdgeWindow =
          [&](std::vector<TopoWaypoint>& chain, size_t edge_end_idx) -> bool {
        if (edge_end_idx == 0 || edge_end_idx >= chain.size()) {
          return false;
        }
        const TopoWaypoint from = chain[edge_end_idx - 1];
        TopoWaypoint target = withExistingOrIncomingYaw(from, chain[edge_end_idx]);
        if (configurationLineFree(from, target)) {
          chain[edge_end_idx] = target;
          return true;
        }
        if ((target.pos - from.pos).norm() < shortcut_res * 0.75) {
          return false;
        }

        TopoWaypoint mid(from.pos + 0.5 * (target.pos - from.pos));
        mid.yaw = interpolateWaypointYaw(from, target, 0.5);
        mid.has_yaw = true;
        if (!pushPointFromObstacle(mid, safe_dist)) {
          return false;
        }
        if (!configurationLineFree(from, mid) ||
            !configurationLineFree(mid, target)) {
          return false;
        }

        chain.insert(chain.begin() + static_cast<long>(edge_end_idx), mid);
        return true;
      };

      auto tryBypassWaypoint =
          [&](std::vector<TopoWaypoint>& chain, size_t idx) -> bool {
        if (idx == 0 || idx + 1 >= chain.size()) {
          return false;
        }
        const TopoWaypoint from = chain[idx - 1];
        const size_t max_span = std::min(chain.size() - idx - 1, static_cast<size_t>(3));
        for (size_t span = 1; span <= max_span; ++span) {
          const TopoWaypoint to = chain[idx + span];
          std::vector<TopoWaypoint> bridge;
          if (!buildBridgeChain(from, to, 0, bridge) || bridge.empty()) {
            continue;
          }
          chain.erase(chain.begin() + static_cast<long>(idx),
                      chain.begin() + static_cast<long>(idx + span));
          if (bridge.size() > 1) {
            chain.insert(chain.begin() + static_cast<long>(idx),
                         bridge.begin(), bridge.end() - 1);
          }
          return true;
        }
        return false;
      };

      auto tryResidualWindowSearch =
          [&](std::vector<TopoWaypoint>& chain, size_t idx) -> bool {
        if (idx == 0 || idx + 1 >= chain.size()) {
          return false;
        }
        const TopoWaypoint prev = chain[idx - 1];
        const TopoWaypoint next = chain[idx + 1];
        TopoWaypoint seed = withIncomingYaw(prev, chain[idx]);
        const double base_score =
            evaluator_.evaluate(SE2State(seed.pos.x(), seed.pos.y(), seed.yaw));
        TopoWaypoint best = seed;
        double best_score = base_score;
        bool found = false;

        const int radial_rings = 3;
        const int angular_samples = 12;
        const int yaw_samples = 8;
        const double radial_step = std::max(map_->resolution(), 0.5 * safe_dist);

        for (int ring = 0; ring <= radial_rings; ++ring) {
          const double radius = ring * radial_step;
          const int dir_samples = (ring == 0) ? 1 : angular_samples;
          for (int aidx = 0; aidx < dir_samples; ++aidx) {
            const double theta =
                (dir_samples == 1) ? 0.0 : kTwoPi * static_cast<double>(aidx) /
                                              static_cast<double>(dir_samples);
            for (int yidx = 0; yidx < yaw_samples; ++yidx) {
              const double yaw_offset =
                  (yidx == 0) ? 0.0 :
                  (kTwoPi * static_cast<double>(yidx - yaw_samples / 2) /
                   static_cast<double>(yaw_samples));
              TopoWaypoint trial(seed.pos.x() + radius * std::cos(theta),
                                 seed.pos.y() + radius * std::sin(theta));
              trial.yaw = normalizeAngle(seed.yaw + yaw_offset);
              trial.has_yaw = true;

              const double state_score =
                  evaluator_.evaluate(SE2State(trial.pos.x(), trial.pos.y(), trial.yaw));
              if (state_score < best_score - 1e-6) {
                continue;
              }
              if (!found || state_score > best_score) {
                found = true;
                best = trial;
                best_score = state_score;
              }
              if (state_score >= 0.0 &&
                  configurationLineFree(prev, trial) &&
                  configurationLineFree(trial, withExistingOrIncomingYaw(trial, next))) {
                chain[idx] = trial;
                return true;
              }
            }
          }
        }

        if (found && best_score > base_score + 1e-6 &&
            configurationLineFree(prev, best) &&
            configurationLineFree(best, withExistingOrIncomingYaw(best, next))) {
          chain[idx] = best;
          return true;
        }
        return false;
      };

      auto tryAnchoredWindowSearch =
          [&](const TopoWaypoint& prev,
              const TopoWaypoint& next,
              TopoWaypoint& seed) -> bool {
        TopoWaypoint best = seed;
        double best_score =
            evaluator_.evaluate(SE2State(seed.pos.x(), seed.pos.y(), seed.yaw));
        const int radial_rings = 3;
        const int angular_samples = 12;
        const int yaw_samples = 8;
        const double radial_step = std::max(map_->resolution(), 0.5 * safe_dist);

        for (int ring = 0; ring <= radial_rings; ++ring) {
          const double radius = ring * radial_step;
          const int dir_samples = (ring == 0) ? 1 : angular_samples;
          for (int aidx = 0; aidx < dir_samples; ++aidx) {
            const double theta =
                (dir_samples == 1) ? 0.0 : kTwoPi * static_cast<double>(aidx) /
                                              static_cast<double>(dir_samples);
            for (int yidx = 0; yidx < yaw_samples; ++yidx) {
              const double yaw_offset =
                  (yidx == 0) ? 0.0 :
                  (kTwoPi * static_cast<double>(yidx - yaw_samples / 2) /
                   static_cast<double>(yaw_samples));
              TopoWaypoint trial(seed.pos.x() + radius * std::cos(theta),
                                 seed.pos.y() + radius * std::sin(theta));
              trial.yaw = normalizeAngle(seed.yaw + yaw_offset);
              trial.has_yaw = true;
              const double state_score =
                  evaluator_.evaluate(SE2State(trial.pos.x(), trial.pos.y(), trial.yaw));
              if (state_score > best_score) {
                best_score = state_score;
                best = trial;
              }
              if (state_score >= 0.0 &&
                  configurationLineFree(prev, trial) &&
                  configurationLineFree(trial, withExistingOrIncomingYaw(trial, next))) {
                seed = trial;
                return true;
              }
            }
          }
        }

        if (best_score > evaluator_.evaluate(SE2State(seed.pos.x(), seed.pos.y(), seed.yaw)) &&
            configurationLineFree(prev, best) &&
            configurationLineFree(best, withExistingOrIncomingYaw(best, next))) {
          seed = best;
          return true;
        }
        return false;
      };

      for (int repair_pass = 0; repair_pass < 4; ++repair_pass) {
        bool changed = false;
        assignMissingYaw(result);
        for (size_t k = 1; k + 1 < result.size(); ++k) {
          TopoWaypoint candidate = withIncomingYaw(result[k - 1], result[k]);
          const double state_clearance =
              evaluator_.evaluate(SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw));
          const double incoming_clearance = configurationClearance(result[k - 1], candidate);
          if (state_clearance >= 0.0 && incoming_clearance >= 0.0) {
            result[k] = candidate;
            continue;
          }

          if (pushPointFromObstacle(candidate, safe_dist) &&
              configurationLineFree(result[k - 1], candidate)) {
            result[k] = candidate;
            changed = true;
            continue;
          }

          if (tryBypassWaypoint(result, k)) {
            changed = true;
            break;
          }

          if ((state_clearance < -0.02 || incoming_clearance < -0.02) &&
              tryResidualWindowSearch(result, k)) {
            changed = true;
            break;
          }

          if (tryRepairEdgeWindow(result, k)) {
            changed = true;
            break;
          }
        }

        if (!changed) {
          for (size_t k = 1; k < result.size(); ++k) {
            if (configurationLineFree(result[k - 1],
                                      withExistingOrIncomingYaw(result[k - 1], result[k]))) {
              continue;
            }
            if (tryRepairEdgeWindow(result, k)) {
              changed = true;
              break;
            }
          }
        }

        if (!changed) {
          break;
        }
      }

      if (!reference_tail.empty() &&
          (result.back().pos - reference_tail.back().pos).norm() > 1e-6) {
        TopoWaypoint tail = withExistingOrIncomingYaw(result.back(), reference_tail.back());
        if (configurationLineFree(result.back(), tail)) {
          result.push_back(tail);
        } else {
          TopoWaypoint repaired_tail = tail;
          if (pushPointFromObstacle(repaired_tail, safe_dist) &&
              configurationLineFree(result.back(), repaired_tail)) {
            result.push_back(repaired_tail);
          }
        }
      }

      for (size_t k = 1; k + 1 < result.size(); ++k) {
        TopoWaypoint adjusted = result[k];
        if (!adjusted.has_yaw) {
          adjusted = withIncomingYaw(result[k - 1], adjusted);
        }
        if (pushPointFromObstacle(adjusted, safe_dist)) {
          result[k] = adjusted;
        }
      }
      assignMissingYaw(result);

      for (size_t k = 1; k + 1 < result.size();) {
        TopoWaypoint merged = withIncomingYaw(result[k - 1], result[k + 1]);
        if (lineCollisionFree(result[k - 1].pos, merged.pos) &&
            configurationLineFree(result[k - 1], merged)) {
          result.erase(result.begin() + static_cast<long>(k));
          assignMissingYaw(result);
          continue;
        }
        ++k;
      }

      for (size_t k = 1; k < result.size(); ++k) {
        TopoWaypoint candidate = result[k];
        if (!candidate.has_yaw) {
          candidate = withIncomingYaw(result[k - 1], candidate);
        }
        const bool incoming_free =
            checker_->isFree(SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw));
        const bool incoming_line_free = configurationLineFree(result[k - 1], candidate);
        if (incoming_free && incoming_line_free) {
          result[k] = candidate;
          continue;
        }

        TopoWaypoint incoming_candidate = withIncomingYaw(result[k - 1], result[k]);
        if (k + 1 < result.size()) {
          if (pushPointFromObstacle(candidate, safe_dist) &&
              configurationLineFree(result[k - 1], candidate)) {
            result[k] = candidate;
            continue;
          }
          if (pushPointFromObstacle(incoming_candidate, safe_dist) &&
              configurationLineFree(result[k - 1], incoming_candidate)) {
            result[k] = incoming_candidate;
            continue;
          }
        }
      }
      assignMissingYaw(result);

      for (int cleanup_pass = 0; cleanup_pass < 4; ++cleanup_pass) {
        bool changed = false;
        for (size_t k = 1; k + 1 < result.size(); ++k) {
          TopoWaypoint candidate = withIncomingYaw(result[k - 1], result[k]);
          const double state_clearance =
              evaluator_.evaluate(SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw));
          if (state_clearance >= 0.0) {
            result[k] = candidate;
            continue;
          }
          if (tryBypassWaypoint(result, k)) {
            changed = true;
            break;
          }
          if (tryResidualWindowSearch(result, k)) {
            changed = true;
            break;
          }
          if (pushPointFromObstacle(candidate, safe_dist) &&
              configurationLineFree(result[k - 1], candidate) &&
              configurationLineFree(candidate, withExistingOrIncomingYaw(candidate, result[k + 1]))) {
            result[k] = candidate;
            changed = true;
            break;
          }
        }
        assignMissingYaw(result);
        if (!changed) {
          break;
        }
      }
      if (have_graph_endpoints &&
          result.size() > 2 &&
          (result.front().pos - nodes_.front()).norm() < 0.5 * map_->resolution()) {
        result.erase(result.begin());
      }
      if (have_graph_endpoints &&
          result.size() > 2 &&
          (result.back().pos - nodes_[1]).norm() < 0.5 * map_->resolution()) {
        result.pop_back();
      }
      assignMissingYaw(result);

      if (have_graph_endpoints && !result.empty()) {
        TopoWaypoint start_anchor(nodes_.front());
        start_anchor.yaw = std::atan2((result.front().pos - start_anchor.pos).y(),
                                      (result.front().pos - start_anchor.pos).x());
        start_anchor.has_yaw = true;
        TopoWaypoint first = withIncomingYaw(start_anchor, result.front());
        const double first_clearance =
            evaluator_.evaluate(SE2State(first.pos.x(), first.pos.y(), first.yaw));
        if (first_clearance < 0.0) {
          if (pushPointFromObstacle(first, safe_dist) &&
              configurationLineFree(start_anchor, first)) {
            result.front() = first;
          } else if (result.size() > 1) {
            bool bridged = false;
            const size_t max_span = std::min(result.size() - 1, static_cast<size_t>(3));
            for (size_t span = 1; span <= max_span; ++span) {
              std::vector<TopoWaypoint> bridge;
              if (!buildBridgeChain(start_anchor, result[span], 0, bridge) || bridge.empty()) {
                continue;
              }
              result.erase(result.begin(), result.begin() + static_cast<long>(span));
              if (bridge.size() > 1) {
                result.insert(result.begin(), bridge.begin(), bridge.end() - 1);
              }
              bridged = true;
              break;
            }
            if (!bridged) {
              TopoWaypoint searched = first;
              if (tryAnchoredWindowSearch(start_anchor, result[1], searched)) {
                result.front() = searched;
              }
            }
          }
        }
      }

      if (have_graph_endpoints && !result.empty()) {
        TopoWaypoint goal_anchor(nodes_[1]);
        goal_anchor.yaw = std::atan2((goal_anchor.pos - result.back().pos).y(),
                                     (goal_anchor.pos - result.back().pos).x());
        goal_anchor.has_yaw = true;
        const size_t last_idx = result.size() - 1;
        TopoWaypoint last = result[last_idx];
        if (!last.has_yaw && last_idx > 0) {
          last = withIncomingYaw(result[last_idx - 1], last);
        }
        const double last_clearance =
            evaluator_.evaluate(SE2State(last.pos.x(), last.pos.y(), last.yaw));
        if (last_clearance < 0.0) {
          const bool incoming_ok =
              (last_idx == 0) ? true : configurationLineFree(result[last_idx - 1], last);
          if (pushPointFromObstacle(last, safe_dist) && incoming_ok) {
            result[last_idx] = last;
          } else if (last_idx > 0) {
            bool bridged = false;
            const size_t max_span = std::min(last_idx + 1, static_cast<size_t>(3));
            for (size_t span = 1; span <= max_span; ++span) {
              const size_t from_idx = last_idx - (span - 1);
              if (from_idx == 0) {
                continue;
              }
              std::vector<TopoWaypoint> bridge;
              if (!buildBridgeChain(result[from_idx - 1], goal_anchor, 0, bridge) ||
                  bridge.empty()) {
                continue;
              }
              result.erase(result.begin() + static_cast<long>(from_idx), result.end());
              if (bridge.size() > 1) {
                result.insert(result.end(), bridge.begin(), bridge.end() - 1);
              }
              bridged = true;
              break;
            }
            if (!bridged) {
              TopoWaypoint searched = last;
              if (tryAnchoredWindowSearch(result[last_idx - 1], goal_anchor, searched)) {
                result[last_idx] = searched;
              }
            }
          }
        }
      }
      assignMissingYaw(result);
      return result;
    };

    auto lightweightFinalizeChain = [&](std::vector<TopoWaypoint> result,
                                        const std::vector<TopoWaypoint>& reference_tail) {
      if (result.empty()) {
        return result;
      }

      if (!reference_tail.empty() &&
          (result.back().pos - reference_tail.back().pos).norm() > 1e-6) {
        result.push_back(withExistingOrIncomingYaw(result.back(), reference_tail.back()));
      }

      if (have_graph_endpoints &&
          result.size() > 2 &&
          (result.front().pos - nodes_.front()).norm() < 0.5 * map_->resolution()) {
        result.erase(result.begin());
      }
      if (have_graph_endpoints &&
          result.size() > 2 &&
          (result.back().pos - nodes_[1]).norm() < 0.5 * map_->resolution()) {
        result.pop_back();
      }

      assignMissingYaw(result);

      for (size_t k = 1; k + 1 < result.size(); ++k) {
        TopoWaypoint adjusted = withIncomingYaw(result[k - 1], result[k]);
        if (pushPointFromObstacle(adjusted, safe_dist)) {
          result[k] = adjusted;
        } else {
          result[k] = adjusted;
        }
      }
      assignMissingYaw(result);

      for (size_t k = 1; k + 1 < result.size();) {
        TopoWaypoint merged = withIncomingYaw(result[k - 1], result[k + 1]);
        if (lineCollisionFree(result[k - 1].pos, merged.pos) &&
            configurationLineFree(result[k - 1], merged)) {
          result.erase(result.begin() + static_cast<long>(k));
          assignMissingYaw(result);
          continue;
        }
        ++k;
      }

      return result;
    };

    for (int pass = 0; pass < 2; ++pass) {
      bool changed = false;
      for (size_t k = 1; k + 1 < discrete.size(); ++k) {
        TopoWaypoint candidate = withIncomingYaw(discrete[k - 1], discrete[k]);
        const double clearance = evaluator_.evaluate(
            SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw));
        if (clearance >= 0.0) {
          discrete[k] = candidate;
          continue;
        }
        if (pushPointFromObstacle(candidate, safe_dist)) {
          discrete[k] = candidate;
          changed = true;
        }
      }
      assignMissingYaw(discrete);
      if (!changed) {
        break;
      }
    }

    if (discrete.size() >= 2) {
      path.waypoints = lightweightFinalizeChain(discrete, discrete);
      path.computeLength();
      continue;
    }

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
        TopoWaypoint candidate = withExistingOrIncomingYaw(anchor, discrete[j]);
        if (!lineCollisionFree(anchor.pos, candidate.pos)) {
          continue;
        }
        if (configurationLineFree(anchor, candidate)) {
          best_j = j;
          found_long_shortcut = true;
          break;
        }
      }

      if (found_long_shortcut) {
        TopoWaypoint appended = withExistingOrIncomingYaw(anchor, discrete[best_j]);
        result.push_back(appended);
        i = best_j;
        continue;
      }

      const TopoWaypoint next = withExistingOrIncomingYaw(anchor, discrete[i + 1]);
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

      bool inserted_repair = false;
      for (size_t target_j = std::min(discrete.size() - 1, i + 3); target_j > i; --target_j) {
        TopoWaypoint target = withExistingOrIncomingYaw(anchor, discrete[target_j]);
        double t_col = findCollisionT(anchor.pos, target.pos, *map_, safe_dist);
        if (t_col < 0.0) {
          t_col = 0.5;
        }

        TopoWaypoint pushed_wp(anchor.pos + t_col * (target.pos - anchor.pos));
        pushed_wp = withIncomingYaw(anchor, pushed_wp);
        if (pushPointFromObstacle(pushed_wp, safe_dist) &&
            lineCollisionFree(anchor.pos, pushed_wp.pos) &&
            configurationLineFree(anchor, pushed_wp) &&
            (pushed_wp.pos - anchor.pos).norm() > res * 0.50) {
          result.push_back(pushed_wp);
          ++inserted_pushes;
          inserted_repair = true;
          break;
        }

        if (target_j == i + 1) {
          break;
        }
      }

      if (inserted_repair) {
        continue;
      }

      TopoWaypoint repaired_next = next;
      if (pushPointFromObstacle(repaired_next, safe_dist) &&
          configurationLineFree(anchor, repaired_next)) {
        result.push_back(repaired_next);
        i += 1;
        continue;
      }

      // Do not keep a colliding dense sample in the final path. Advance the
      // source cursor and let the current anchor try the next waypoint instead.
      i += 1;
    }

    if (result.size() < 2 && discrete.size() >= 2) {
      result = discrete;
    }
    result = lightweightFinalizeChain(result, discrete);
    if (result.size() < 2 && discrete.size() >= 2) {
      result = discrete;
      assignMissingYaw(result);
    }

    path.waypoints = std::move(result);
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

bool TopologyPlanner::repairWaypointBodyFrame(TopoWaypoint& wp, double safe_dist) const {
  return pushPointFromObstacle(wp, safe_dist);
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
