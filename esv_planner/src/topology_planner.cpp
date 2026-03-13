#include "esv_planner/topology_planner.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <functional>
#include <unordered_set>
#include <string>

namespace esv_planner {

// ---------------------------------------------------------------------------
// Anonymous helpers
// ---------------------------------------------------------------------------
namespace {

void assignTangentYaw(std::vector<TopoWaypoint>& wps) {
  if (wps.size() <= 1) {
    if (!wps.empty()) { wps.front().yaw = 0.0; wps.front().has_yaw = true; }
    return;
  }
  for (size_t i = 0; i < wps.size(); ++i) {
    Eigen::Vector2d dir = (i == 0)
        ? (wps[1].pos - wps[0].pos)
        : (wps[i].pos - wps[i - 1].pos);
    wps[i].yaw = (dir.norm() < 1e-9)
        ? ((i > 0) ? wps[i - 1].yaw : 0.0)
        : std::atan2(dir.y(), dir.x());
    wps[i].has_yaw = true;
  }
}

void assignMissingYaw(std::vector<TopoWaypoint>& wps) {
  if (wps.size() <= 1) {
    if (!wps.empty() && !wps.front().has_yaw) {
      wps.front().yaw = 0.0; wps.front().has_yaw = true;
    }
    return;
  }
  for (size_t i = 0; i < wps.size(); ++i) {
    if (wps[i].has_yaw) continue;
    Eigen::Vector2d dir = (i + 1 < wps.size())
        ? (wps[i + 1].pos - wps[i].pos)
        : (wps[i].pos - wps[i - 1].pos);
    wps[i].yaw = (dir.norm() < 1e-9)
        ? ((i > 0 && wps[i - 1].has_yaw) ? wps[i - 1].yaw : 0.0)
        : std::atan2(dir.y(), dir.x());
    wps[i].has_yaw = true;
  }
}

double interpolateWaypointYaw(const TopoWaypoint& a, const TopoWaypoint& b, double t) {
  double ya = a.has_yaw ? a.yaw : std::atan2((b.pos - a.pos).y(), (b.pos - a.pos).x());
  double yb = b.has_yaw ? b.yaw : ya;
  return normalizeAngle(ya + normalizeAngle(yb - ya) * t);
}

TopoWaypoint withIncomingYaw(const TopoWaypoint& from, const TopoWaypoint& to) {
  TopoWaypoint out = to;
  Eigen::Vector2d dir = out.pos - from.pos;
  out.yaw = (dir.norm() < 1e-9)
      ? (from.has_yaw ? from.yaw : 0.0)
      : std::atan2(dir.y(), dir.x());
  out.has_yaw = true;
  return out;
}

TopoWaypoint withExistingOrIncomingYaw(const TopoWaypoint& from, const TopoWaypoint& to) {
  return to.has_yaw ? to : withIncomingYaw(from, to);
}

// Halton low-discrepancy sequence
double haltonSequence(int index, int base) {
  double result = 0.0, f = 1.0 / base;
  int i = index;
  while (i > 0) { result += f * (i % base); i /= base; f /= base; }
  return result;
}

// Binary-search for first collision parametric t along a->b
double findCollisionT(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                      const GridMap& map, double threshold) {
  double dist = (b - a).norm();
  double step = map.resolution() * 0.5;
  int n_steps = std::max(1, static_cast<int>(std::ceil(dist / step)));

  double t_col = -1.0;
  for (int i = 0; i <= n_steps; ++i) {
    double t = static_cast<double>(i) / n_steps;
    Eigen::Vector2d p = a + t * (b - a);
    if (map.getEsdf(p.x(), p.y()) < threshold) { t_col = t; break; }
  }
  if (t_col < 0.0) return -1.0;

  double t_lo = std::max(0.0, t_col - 1.0 / n_steps), t_hi = t_col;
  for (int iter = 0; iter < 16; ++iter) {
    double t_mid = 0.5 * (t_lo + t_hi);
    Eigen::Vector2d p = a + t_mid * (b - a);
    if (map.getEsdf(p.x(), p.y()) < threshold) t_hi = t_mid; else t_lo = t_mid;
  }
  return 0.5 * (t_lo + t_hi);
}

// Discretize a waypoint chain at uniform spacing
std::vector<TopoWaypoint> discretizePath(const std::vector<TopoWaypoint>& pts,
                                          double resolution) {
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
    if (accum > 0.0) {
      double needed = resolution - accum;
      if (needed <= remaining) {
        cursor += needed * unit; remaining -= needed;
        out.emplace_back(cursor); accum = 0.0;
      } else { accum += remaining; continue; }
    }
    while (remaining >= resolution) {
      cursor += resolution * unit; remaining -= resolution;
      out.emplace_back(cursor);
    }
    accum = remaining;
  }
  if ((out.back().pos - pts.back().pos).norm() > 1e-9) out.push_back(pts.back());
  assignTangentYaw(out);
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / initialisation
// ---------------------------------------------------------------------------
TopologyPlanner::TopologyPlanner() {}

void TopologyPlanner::init(const GridMap& map, const CollisionChecker& checker,
                           const SvsdfEvaluator& evaluator,
                           int num_samples, int knn, int max_paths,
                           double inscribed_radius) {
  map_ = &map;
  checker_ = &checker;
  evaluator_ = &evaluator;
  num_samples_ = num_samples;
  knn_ = knn;
  max_paths_ = max_paths;
  inscribed_radius_ = inscribed_radius;
}

// ---------------------------------------------------------------------------
// buildRoadmap -- Halton quasi-random PRM
// ---------------------------------------------------------------------------
void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal) {
  nodes_.clear();
  adjacency_.clear();
  nodes_.push_back(start);
  nodes_.push_back(goal);

  double x_min = map_->originX(), y_min = map_->originY();
  double x_max = x_min + map_->width() * map_->resolution();
  double y_max = y_min + map_->height() * map_->resolution();

  int accepted = 0;
  for (int i = 1; accepted < num_samples_; ++i) {
    double x = x_min + haltonSequence(i, 2) * (x_max - x_min);
    double y = y_min + haltonSequence(i, 3) * (y_max - y_min);
    if (map_->getEsdf(x, y) > inscribed_radius_) {
      nodes_.emplace_back(x, y);
      ++accepted;
    }
  }

  int n = static_cast<int>(nodes_.size());
  adjacency_.resize(n);
  for (int i = 0; i < n; ++i) {
    std::vector<std::pair<double, int>> dists;
    dists.reserve(n);
    for (int j = 0; j < n; ++j) {
      if (i != j) dists.emplace_back((nodes_[i] - nodes_[j]).norm(), j);
    }
    std::sort(dists.begin(), dists.end());
    int connections = 0;
    for (const auto& p : dists) {
      if (connections >= knn_) break;
      if (lineCollisionFree(nodes_[i], nodes_[p.second])) {
        adjacency_[i].emplace_back(p.second, p.first);
        ++connections;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// dijkstra -- single-source shortest path on PRM graph
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
    const PII current = pq.top();
    pq.pop();
    const double d = current.first;
    const int u = current.second;
    if (d > dist[u]) continue;
    if (u == dst) break;
    for (const auto& edge : adjacency_[u]) {
      const int v = edge.first;
      const double w_base = edge.second;
      double w = w_base + (edge_penalty.empty() ? 0.0 : edge_penalty[v]);
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  TopoPath result;
  if (dist[dst] >= kInf) return result;
  std::vector<int> idx;
  for (int v = dst; v != -1; v = prev[v]) idx.push_back(v);
  std::reverse(idx.begin(), idx.end());
  for (int i : idx) result.waypoints.emplace_back(nodes_[i]);
  assignTangentYaw(result.waypoints);
  result.computeLength();
  return result;
}

// ---------------------------------------------------------------------------
// searchPaths -- Dijkstra + progressive node penalty for homotopy diversity
// ---------------------------------------------------------------------------
std::vector<TopoPath> TopologyPlanner::searchPaths() {
  std::vector<TopoPath> result;
  if (nodes_.size() < 2) return result;

  const int n = static_cast<int>(nodes_.size());
  std::vector<double> node_penalty(n, 0.0);
  std::unordered_set<std::string> seen;

  auto keyOf = [](const std::vector<int>& p) {
    std::string k; k.reserve(p.size() * 6);
    for (size_t i = 0; i < p.size(); ++i) {
      if (i > 0) k.push_back('-');
      k += std::to_string(p[i]);
    }
    return k;
  };

  auto indicesFromPath = [&](const TopoPath& path) {
    std::vector<int> indices;
    for (const auto& wp : path.waypoints) {
      int best = -1; double best_d = kInf;
      for (int ni = 0; ni < n; ++ni) {
        double d = (nodes_[ni] - wp.pos).norm();
        if (d < best_d) { best_d = d; best = ni; }
      }
      if (best >= 0 && best_d < 1e-6) indices.push_back(best);
      else { indices.clear(); break; }
    }
    return indices;
  };

  for (int iter = 0; iter < max_paths_ * 6; ++iter) {
    TopoPath path = dijkstra(0, 1, node_penalty);
    if (path.waypoints.size() < 2) break;
    auto indices = indicesFromPath(path);
    if (indices.size() < 2) break;
    if (seen.insert(keyOf(indices)).second) {
      result.push_back(path);
      if (static_cast<int>(result.size()) >= max_paths_) break;
    }
    for (size_t i = 1; i + 1 < indices.size(); ++i)
      node_penalty[indices[i]] += std::max(0.5, path.length * 0.1);
  }
  return result;
}

// ---------------------------------------------------------------------------
// lineCollisionFree -- half-resolution ESDF check
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
// configurationClearance / configurationLineFree -- SE2 interpolation check
// ---------------------------------------------------------------------------
double TopologyPlanner::configurationClearance(const TopoWaypoint& a,
                                               const TopoWaypoint& b) const {
  double ya = a.has_yaw ? a.yaw : std::atan2((b.pos - a.pos).y(), (b.pos - a.pos).x());
  double yb = b.has_yaw ? b.yaw : ya;
  SE2State sa(a.pos.x(), a.pos.y(), ya);
  SE2State sb(b.pos.x(), b.pos.y(), yb);
  double step = std::max(map_->resolution(), 0.10);
  return interpolatedClearance(sa, sb, step,
      [this](const SE2State& s) { return evaluator_->evaluate(s); });
}

bool TopologyPlanner::configurationLineFree(const TopoWaypoint& a,
                                             const TopoWaypoint& b) const {
  return configurationClearance(a, b) >= 0.0;
}

// ---------------------------------------------------------------------------
// pushPointFromObstacle -- body-frame ESDF gradient ascent until safe
// ---------------------------------------------------------------------------
bool TopologyPlanner::pushPointFromObstacle(TopoWaypoint& wp, double safe_dist) const {
  const double eps = map_->resolution();

  auto isSafe = [&](const TopoWaypoint& c) {
    return evaluator_->evaluate(SE2State(c.pos.x(), c.pos.y(), c.yaw)) >= 0.0;
  };

  if (isSafe(wp)) { wp.has_yaw = true; return true; }

  // Try yaw repair first
  auto tryYawRepair = [&](TopoWaypoint& c) {
    constexpr int kYawSamples = 12;
    double best_cl = -kInf, best_yaw = c.yaw;
    for (int k = 0; k < kYawSamples; ++k) {
      double yaw = normalizeAngle(c.yaw + kTwoPi * k / kYawSamples);
      double cl = evaluator_->evaluate(SE2State(c.pos.x(), c.pos.y(), yaw));
      if (cl > best_cl) { best_cl = cl; best_yaw = yaw; }
      if (cl >= 0.0) { c.yaw = yaw; c.has_yaw = true; return true; }
    }
    c.yaw = best_yaw; c.has_yaw = true;
    return best_cl >= 0.0;
  };

  for (int attempt = 0; attempt < 24; ++attempt) {
    if (isSafe(wp)) { wp.has_yaw = true; return true; }

    // Compute SVSDF gradient via central differences
    Eigen::Vector2d grad_pos; double grad_yaw;
    SE2State st(wp.pos.x(), wp.pos.y(), wp.yaw);
    evaluator_->gradient(st, grad_pos, grad_yaw);

    if (grad_pos.norm() < 1e-9) {
      return tryYawRepair(wp);
    }

    double current_cl = evaluator_->evaluate(st);
    double step_pos = std::min(std::max(eps, 0.5 * (safe_dist - std::min(current_cl, safe_dist))),
                               4.0 * eps);
    Eigen::Vector2d dir = grad_pos.normalized();  // ascend toward free space

    bool accepted = false;
    for (double scale : {1.0, 0.5, 0.25}) {
      TopoWaypoint candidate = wp;
      candidate.pos += scale * step_pos * dir;
      const double yaw_step = std::max(-0.12, std::min(-0.05 * grad_yaw, 0.12));
      candidate.yaw = normalizeAngle(candidate.yaw + scale * yaw_step);
      candidate.has_yaw = true;
      double new_cl = evaluator_->evaluate(
          SE2State(candidate.pos.x(), candidate.pos.y(), candidate.yaw));
      if (new_cl > current_cl - 1e-6) {
        wp = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) return tryYawRepair(wp);
  }
  return isSafe(wp);
}

// ---------------------------------------------------------------------------
// repairWaypointBodyFrame -- public wrapper
// ---------------------------------------------------------------------------
bool TopologyPlanner::repairWaypointBodyFrame(TopoWaypoint& wp, double safe_dist) const {
  return pushPointFromObstacle(wp, safe_dist);
}

// ---------------------------------------------------------------------------
// shortenPaths -- Algorithm 1: visibility shortcut + collision push-away
//
//  For each path:
//   1. Discretize into dense waypoints
//   2. Walk from anchor; for each point, try longest visible shortcut
//   3. If blocked, find collision point via binary search, push away, insert
//   4. Finalize: push interior points, merge redundant waypoints
//   5. Filter topologically distinct paths
// ---------------------------------------------------------------------------
void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  const double safe_dist = inscribed_radius_;
  const double res = map_->resolution();
  const double shortcut_res = std::max(res, 0.10);

  // Budget: only shorten the shortest paths
  const size_t shorten_budget = 6;
  if (paths.size() > shorten_budget) {
    std::stable_sort(paths.begin(), paths.end(),
        [](const TopoPath& a, const TopoPath& b) { return a.length < b.length; });
    paths.resize(shorten_budget);
  }

  for (auto& path : paths) {
    if (path.waypoints.size() <= 2) continue;

    // Pre-repair interior waypoints
    std::vector<TopoWaypoint> discrete = path.waypoints;
    assignTangentYaw(discrete);
    for (size_t k = 1; k + 1 < discrete.size(); ++k) {
      TopoWaypoint adj = withIncomingYaw(discrete[k - 1], discrete[k]);
      double cl = evaluator_->evaluate(SE2State(adj.pos.x(), adj.pos.y(), adj.yaw));
      if (cl < 0.0) pushPointFromObstacle(adj, safe_dist);
      discrete[k] = adj;
    }

    // Visibility shortcut pass
    std::vector<TopoWaypoint> result;
    result.push_back(discrete.front());
    size_t i = 0;
    int inserted_pushes = 0;
    const int max_pushes = static_cast<int>(discrete.size()) * 2;

    while (i + 1 < discrete.size()) {
      const TopoWaypoint& anchor = result.back();

      // Try longest visible shortcut
      size_t best_j = i + 1;
      bool found_shortcut = false;
      for (size_t j = discrete.size() - 1; j > i + 1; --j) {
        TopoWaypoint cand = withExistingOrIncomingYaw(anchor, discrete[j]);
        if (lineCollisionFree(anchor.pos, cand.pos) && configurationLineFree(anchor, cand)) {
          best_j = j; found_shortcut = true; break;
        }
      }

      if (found_shortcut) {
        result.push_back(withExistingOrIncomingYaw(anchor, discrete[best_j]));
        i = best_j;
        continue;
      }

      // Next point directly visible?
      TopoWaypoint next = withExistingOrIncomingYaw(anchor, discrete[i + 1]);
      if (configurationLineFree(anchor, next)) {
        result.push_back(next); i += 1; continue;
      }

      // Blocked: find collision point, push away, insert waypoint
      if (inserted_pushes < max_pushes) {
        bool inserted = false;
        for (size_t tj = std::min(discrete.size() - 1, i + 3); tj > i; --tj) {
          TopoWaypoint target = withExistingOrIncomingYaw(anchor, discrete[tj]);
          double t_col = findCollisionT(anchor.pos, target.pos, *map_, safe_dist);
          if (t_col < 0.0) t_col = 0.5;

          TopoWaypoint pushed(anchor.pos + t_col * (target.pos - anchor.pos));
          pushed = withIncomingYaw(anchor, pushed);
          if (pushPointFromObstacle(pushed, safe_dist) &&
              lineCollisionFree(anchor.pos, pushed.pos) &&
              configurationLineFree(anchor, pushed) &&
              (pushed.pos - anchor.pos).norm() > res * 0.5) {
            result.push_back(pushed);
            ++inserted_pushes; inserted = true; break;
          }
          if (tj == i + 1) break;
        }
        if (inserted) continue;
      }

      // Fallback: push the next point and advance
      TopoWaypoint repaired = next;
      if (pushPointFromObstacle(repaired, safe_dist) && configurationLineFree(anchor, repaired)) {
        result.push_back(repaired);
      }
      i += 1;
    }

    if (result.size() < 2 && discrete.size() >= 2) result = discrete;

    // Finalize: push interior, merge redundant
    assignMissingYaw(result);
    for (size_t k = 1; k + 1 < result.size(); ++k) {
      TopoWaypoint adj = withIncomingYaw(result[k - 1], result[k]);
      pushPointFromObstacle(adj, safe_dist);
      result[k] = adj;
    }
    assignMissingYaw(result);

    // Merge pass: remove waypoints where direct connection is free
    for (size_t k = 1; k + 1 < result.size();) {
      TopoWaypoint merged = withIncomingYaw(result[k - 1], result[k + 1]);
      if (lineCollisionFree(result[k - 1].pos, merged.pos) &&
          configurationLineFree(result[k - 1], merged)) {
        result.erase(result.begin() + static_cast<long>(k));
        assignMissingYaw(result);
      } else {
        ++k;
      }
    }
    assignMissingYaw(result);

    if (result.size() < 2 && discrete.size() >= 2) {
      result = discrete; assignMissingYaw(result);
    }
    path.waypoints = std::move(result);
    path.computeLength();
  }

  // Filter topologically distinct paths
  std::vector<TopoPath> filtered;
  filtered.reserve(paths.size());
  for (const auto& p : paths) {
    if (p.waypoints.size() < 2) continue;
    if (isTopologicallyDistinct(p, filtered)) filtered.push_back(p);
    if (static_cast<int>(filtered.size()) >= max_paths_) break;
  }
  paths = filtered;
}

// ---------------------------------------------------------------------------
// isTopologicallyDistinct -- UVD with 40 sample points
// ---------------------------------------------------------------------------
bool TopologyPlanner::isTopologicallyDistinct(
    const TopoPath& path, const std::vector<TopoPath>& existing) const {
  constexpr int kSamples = 40;

  auto interpolate = [](const TopoPath& tp, double frac) -> Eigen::Vector2d {
    if (tp.waypoints.empty()) return Eigen::Vector2d::Zero();
    if (tp.length < 1e-9) return tp.waypoints.front().pos;
    double target = frac * tp.length, acc = 0.0;
    for (size_t j = 1; j < tp.waypoints.size(); ++j) {
      double seg = (tp.waypoints[j].pos - tp.waypoints[j - 1].pos).norm();
      if (acc + seg >= target) {
        double f = (seg > 1e-9) ? (target - acc) / seg : 0.0;
        return tp.waypoints[j - 1].pos + f * (tp.waypoints[j].pos - tp.waypoints[j - 1].pos);
      }
      acc += seg;
    }
    return tp.waypoints.back().pos;
  };

  for (const auto& ep : existing) {
    bool equivalent = true;
    for (int i = 0; i <= kSamples && equivalent; ++i) {
      double t = static_cast<double>(i) / kSamples;
      Eigen::Vector2d p1 = interpolate(path, t), p2 = interpolate(ep, t);
      double seg_dist = (p2 - p1).norm();
      if (seg_dist < 1e-6) continue;
      int steps = std::max(2, static_cast<int>(std::ceil(seg_dist / (map_->resolution() * 0.5))));
      for (int s = 0; s <= steps; ++s) {
        Eigen::Vector2d p = p1 + (static_cast<double>(s) / steps) * (p2 - p1);
        GridIndex gi = map_->worldToGrid(p.x(), p.y());
        if (map_->isOccupied(gi.x, gi.y)) {
          equivalent = false; break;
        }
      }
    }
    if (equivalent) return false;
  }
  return true;
}

}  // namespace esv_planner
