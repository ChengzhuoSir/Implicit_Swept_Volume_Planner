#include "esv_planner/topology_planner.h"

#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>

namespace esv_planner {

namespace {

double HaltonSequence(int index, int base) {
  double result = 0.0;
  double f = 1.0 / base;
  int value = index;
  while (value > 0) {
    result += f * static_cast<double>(value % base);
    value /= base;
    f /= static_cast<double>(base);
  }
  return result;
}

void AssignTangentYaw(std::vector<TopoWaypoint>* waypoints) {
  if (!waypoints || waypoints->empty()) {
    return;
  }
  if (waypoints->size() == 1) {
    (*waypoints)[0].yaw = 0.0;
    (*waypoints)[0].has_yaw = true;
    return;
  }

  for (size_t i = 0; i < waypoints->size(); ++i) {
    Eigen::Vector2d direction = (i == 0) ? ((*waypoints)[1].pos - (*waypoints)[0].pos)
                                         : ((*waypoints)[i].pos - (*waypoints)[i - 1].pos);
    if (direction.norm() < 1e-9) {
      (*waypoints)[i].yaw = (i > 0) ? (*waypoints)[i - 1].yaw : 0.0;
    } else {
      (*waypoints)[i].yaw = std::atan2(direction.y(), direction.x());
    }
    (*waypoints)[i].has_yaw = true;
  }
}

TopoWaypoint WithIncomingYaw(const TopoWaypoint& from, const TopoWaypoint& to) {
  TopoWaypoint output = to;
  const Eigen::Vector2d delta = output.pos - from.pos;
  if (delta.norm() < 1e-9) {
    output.yaw = from.has_yaw ? from.yaw : 0.0;
  } else {
    output.yaw = std::atan2(delta.y(), delta.x());
  }
  output.has_yaw = true;
  return output;
}

Eigen::Vector2d EsdfAscentDirection(const GridMap& map, const Eigen::Vector2d& world,
                                    double eps) {
  const double ex_p = map.getEsdf(world.x() + eps, world.y());
  const double ex_n = map.getEsdf(world.x() - eps, world.y());
  const double ey_p = map.getEsdf(world.x(), world.y() + eps);
  const double ey_n = map.getEsdf(world.x(), world.y() - eps);
  Eigen::Vector2d gradient(ex_p - ex_n, ey_p - ey_n);
  if (gradient.norm() > 1e-9) {
    return gradient.normalized();
  }

  const double base = map.getEsdf(world.x(), world.y());
  Eigen::Vector2d best = Eigen::Vector2d::Zero();
  double best_gain = 0.0;
  for (int k = 0; k < 16; ++k) {
    const double theta = kTwoPi * static_cast<double>(k) / 16.0;
    const Eigen::Vector2d direction(std::cos(theta), std::sin(theta));
    for (double scale : {1.0, 2.0, 3.0}) {
      const double value =
          map.getEsdf(world.x() + scale * eps * direction.x(),
                      world.y() + scale * eps * direction.y());
      const double gain = value - base;
      if (gain > best_gain) {
        best_gain = gain;
        best = direction;
      }
    }
  }
  return best;
}

double FindCollisionT(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const GridMap& map,
                      double threshold) {
  const double distance = (b - a).norm();
  const double step = map.resolution() * 0.5;
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));

  double collision_t = -1.0;
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d point = a + t * (b - a);
    if (map.getEsdf(point.x(), point.y()) < threshold) {
      collision_t = t;
      break;
    }
  }
  if (collision_t < 0.0) {
    return -1.0;
  }

  double low = std::max(0.0, collision_t - 1.0 / static_cast<double>(samples));
  double high = collision_t;
  for (int iter = 0; iter < 16; ++iter) {
    const double mid = 0.5 * (low + high);
    const Eigen::Vector2d point = a + mid * (b - a);
    if (map.getEsdf(point.x(), point.y()) < threshold) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return 0.5 * (low + high);
}

double DistanceToSegment(const Eigen::Vector2d& point, const Eigen::Vector2d& a,
                         const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double denom = ab.squaredNorm();
  double t = 0.0;
  if (denom > 1e-9) {
    t = (point - a).dot(ab) / denom;
    t = std::max(0.0, std::min(1.0, t));
  }
  return (point - (a + t * ab)).norm();
}

std::vector<TopoWaypoint> DiscretizePath(const std::vector<TopoWaypoint>& points,
                                         double resolution) {
  if (points.size() <= 1) {
    return points;
  }

  std::vector<TopoWaypoint> output;
  output.push_back(points.front());
  double accumulated = 0.0;

  for (size_t i = 1; i < points.size(); ++i) {
    const Eigen::Vector2d delta = points[i].pos - points[i - 1].pos;
    const double segment_length = delta.norm();
    if (segment_length < 1e-12) {
      continue;
    }

    const Eigen::Vector2d unit = delta / segment_length;
    double remaining = segment_length;
    Eigen::Vector2d cursor = points[i - 1].pos;

    if (accumulated > 0.0) {
      const double needed = resolution - accumulated;
      if (needed <= remaining) {
        cursor = cursor + needed * unit;
        remaining -= needed;
        output.emplace_back(cursor);
        accumulated = 0.0;
      } else {
        accumulated += remaining;
        continue;
      }
    }

    while (remaining >= resolution) {
      cursor = cursor + resolution * unit;
      remaining -= resolution;
      output.emplace_back(cursor);
    }
    accumulated = remaining;
  }

  if ((output.back().pos - points.back().pos).norm() > 1e-9) {
    output.push_back(points.back());
  }
  AssignTangentYaw(&output);
  return output;
}

std::pair<double, std::vector<int>> DijkstraFull(
    int src, int dst, int n, const std::vector<std::vector<std::pair<int, double>>>& adjacency,
    const std::set<std::pair<int, int>>& blocked_edges,
    const std::unordered_set<int>& blocked_nodes) {
  std::vector<double> distance(static_cast<size_t>(n), kInf);
  std::vector<int> previous(static_cast<size_t>(n), -1);
  distance[static_cast<size_t>(src)] = 0.0;

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  open.push(std::make_pair(0.0, src));

  while (!open.empty()) {
    const double best_distance = open.top().first;
    const int current = open.top().second;
    open.pop();

    if (best_distance > distance[static_cast<size_t>(current)]) {
      continue;
    }
    if (current == dst) {
      break;
    }

    for (const auto& edge : adjacency[static_cast<size_t>(current)]) {
      const int next = edge.first;
      const double weight = edge.second;
      if (blocked_nodes.count(next) && next != dst) {
        continue;
      }
      if (blocked_edges.count(std::make_pair(current, next))) {
        continue;
      }

      const double candidate = distance[static_cast<size_t>(current)] + weight;
      if (candidate < distance[static_cast<size_t>(next)]) {
        distance[static_cast<size_t>(next)] = candidate;
        previous[static_cast<size_t>(next)] = current;
        open.push(std::make_pair(candidate, next));
      }
    }
  }

  return std::make_pair(distance[static_cast<size_t>(dst)], previous);
}

}  // namespace

TopologyPlanner::TopologyPlanner() {}

void TopologyPlanner::init(const GridMap& map, const CollisionChecker& checker, int num_samples,
                           int knn, int max_paths, double inscribed_radius) {
  map_ = &map;
  checker_ = &checker;
  num_samples_ = num_samples;
  knn_ = knn;
  max_paths_ = max_paths;
  inscribed_radius_ = inscribed_radius;
  ROS_INFO("TopologyPlanner initialized: num_samples=%d, knn=%d, max_paths=%d, inscribed_radius=%.3f",
           num_samples_, knn_, max_paths_, inscribed_radius_);
}

void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
  nodes_.clear();
  adjacency_.clear();
  nodes_.push_back(start);
  nodes_.push_back(goal);

  const double x_min = map_->originX();
  const double y_min = map_->originY();
  const double x_max = x_min + static_cast<double>(map_->width()) * map_->resolution();
  const double y_max = y_min + static_cast<double>(map_->height()) * map_->resolution();

  int accepted = 0;
  for (int i = 1; accepted < num_samples_; ++i) {
    const double x = x_min + HaltonSequence(i, 2) * (x_max - x_min);
    const double y = y_min + HaltonSequence(i, 3) * (y_max - y_min);
    const double node_clearance = inscribed_radius_ + map_->resolution();
    if (map_->getEsdf(x, y) > node_clearance &&
        !checker_->safeYawIndices(x, y).empty()) {
      nodes_.push_back(Eigen::Vector2d(x, y));
      ++accepted;
    }
  }

  adjacency_.assign(nodes_.size(), std::vector<std::pair<int, double>>());
  int total_edges = 0;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    std::vector<std::pair<double, int>> distances;
    distances.reserve(nodes_.size());
    for (size_t j = 0; j < nodes_.size(); ++j) {
      if (i == j) {
        continue;
      }
      distances.push_back(
          std::make_pair((nodes_[i] - nodes_[j]).norm(), static_cast<int>(j)));
    }
    std::sort(distances.begin(), distances.end());

    int connections = 0;
    for (const auto& item : distances) {
      if (connections >= knn_) {
        break;
      }
      if (lineCollisionFree(nodes_[i], nodes_[static_cast<size_t>(item.second)])) {
        adjacency_[i].push_back(std::make_pair(item.second, item.first));
        ++connections;
        ++total_edges;
      }
    }
  }
  ROS_INFO("Topology Roadmap: nodes=%zu, edges=%d, start_neighbors=%zu, goal_neighbors=%zu",
           nodes_.size(), total_edges, adjacency_[0].size(), adjacency_[1].size());
}

TopoPath TopologyPlanner::dijkstra(
    int src, int dst, const std::vector<double>& node_penalty,
    const std::set<std::pair<int, int>>& blocked_edges,
    const std::unordered_set<int>& blocked_nodes) const {
  const int n = static_cast<int>(nodes_.size());
  std::vector<double> distance(static_cast<size_t>(n), kInf);
  std::vector<int> previous(static_cast<size_t>(n), -1);
  distance[static_cast<size_t>(src)] = 0.0;

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  open.push(std::make_pair(0.0, src));

  while (!open.empty()) {
    const double best_distance = open.top().first;
    const int current = open.top().second;
    open.pop();

    if (best_distance > distance[static_cast<size_t>(current)]) {
      continue;
    }
    if (current == dst) {
      break;
    }

    for (const auto& edge : adjacency_[static_cast<size_t>(current)]) {
      const int next = edge.first;
      if (blocked_nodes.count(next) && next != dst) {
        continue;
      }
      if (blocked_edges.count(std::make_pair(current, next))) {
        continue;
      }

      double weight = edge.second;
      if (!node_penalty.empty()) {
        weight += node_penalty[static_cast<size_t>(next)];
      }
      const double candidate = distance[static_cast<size_t>(current)] + weight;
      if (candidate < distance[static_cast<size_t>(next)]) {
        distance[static_cast<size_t>(next)] = candidate;
        previous[static_cast<size_t>(next)] = current;
        open.push(std::make_pair(candidate, next));
      }
    }
  }

  TopoPath path;
  if (!std::isfinite(distance[static_cast<size_t>(dst)])) {
    return path;
  }

  std::vector<int> indices;
  for (int v = dst; v != -1; v = previous[static_cast<size_t>(v)]) {
    indices.push_back(v);
  }
  std::reverse(indices.begin(), indices.end());
  for (int index : indices) {
    path.waypoints.emplace_back(nodes_[static_cast<size_t>(index)]);
  }
  AssignTangentYaw(&path.waypoints);
  path.computeLength();
  return path;
}

std::vector<TopoPath> TopologyPlanner::searchPaths() {
  const std::set<std::pair<int, int>> blocked_edges;
  const std::unordered_set<int> blocked_nodes;
  return searchPathsImpl(blocked_edges, blocked_nodes);
}

std::vector<TopoPath> TopologyPlanner::searchPathsAvoidingRegion(
    const Eigen::Vector2d& center, double radius) {
  return searchPathsAvoidingCenters(std::vector<Eigen::Vector2d>(1, center), radius);
}

std::vector<TopoPath> TopologyPlanner::searchPathsAvoidingCenters(
    const std::vector<Eigen::Vector2d>& centers, double radius) {
  if (centers.empty()) {
    return searchPaths();
  }

  std::set<std::pair<int, int>> blocked_edges;
  std::unordered_set<int> blocked_nodes;
  const double inflated_radius = radius + map_->resolution();
  const auto point_blocked = [&](const Eigen::Vector2d& point) {
    for (const Eigen::Vector2d& center : centers) {
      if ((point - center).norm() <= inflated_radius) {
        return true;
      }
    }
    return false;
  };

  for (size_t i = 2; i < nodes_.size(); ++i) {
    if (point_blocked(nodes_[i])) {
      blocked_nodes.insert(static_cast<int>(i));
    }
  }

  for (size_t i = 0; i < adjacency_.size(); ++i) {
    for (const auto& edge : adjacency_[i]) {
      const int next = edge.first;
      for (const Eigen::Vector2d& center : centers) {
        if (DistanceToSegment(center, nodes_[i], nodes_[static_cast<size_t>(next)]) <=
            inflated_radius) {
          blocked_edges.insert(std::make_pair(static_cast<int>(i), next));
          break;
        }
      }
    }
  }

  return searchPathsImpl(blocked_edges, blocked_nodes);
}

std::vector<TopoPath> TopologyPlanner::searchPathsImpl(
    const std::set<std::pair<int, int>>& blocked_edges,
    const std::unordered_set<int>& blocked_nodes) const {
  std::vector<TopoPath> result;
  if (nodes_.size() < 2) {
    return result;
  }

  const int node_count = static_cast<int>(nodes_.size());
  const int src = 0;
  const int dst = 1;
  std::vector<double> node_penalty(static_cast<size_t>(node_count), 0.0);
  std::unordered_set<std::string> seen;

  const auto key_of = [](const std::vector<int>& indices) {
    std::string key;
    key.reserve(indices.size() * 6);
    for (size_t i = 0; i < indices.size(); ++i) {
      if (i > 0) {
        key.push_back('-');
      }
      key += std::to_string(indices[i]);
    }
    return key;
  };

  const auto indices_from_path = [&](const TopoPath& path) {
    std::vector<int> indices;
    indices.reserve(path.waypoints.size());
    for (const TopoWaypoint& waypoint : path.waypoints) {
      int best_index = -1;
      double best_distance = kInf;
      for (int i = 0; i < node_count; ++i) {
        const double distance = (nodes_[static_cast<size_t>(i)] - waypoint.pos).norm();
        if (distance < best_distance) {
          best_distance = distance;
          best_index = i;
        }
      }
      if (best_index >= 0 && best_distance < 1e-6) {
        indices.push_back(best_index);
      } else {
        indices.clear();
        break;
      }
    }
    return indices;
  };

  for (int iter = 0; iter < max_paths_ * 6; ++iter) {
    TopoPath path = dijkstra(src, dst, node_penalty, blocked_edges, blocked_nodes);
    if (path.waypoints.size() < 2) {
      break;
    }

    const std::vector<int> indices = indices_from_path(path);
    if (indices.size() < 2) {
      break;
    }

    if (seen.insert(key_of(indices)).second) {
      result.push_back(path);
      if (static_cast<int>(result.size()) >= max_paths_) {
        break;
      }
    }

    for (size_t i = 1; i + 1 < indices.size(); ++i) {
      node_penalty[static_cast<size_t>(indices[i])] += std::max(0.5, path.length * 0.1);
    }
  }

  return result;
}

bool TopologyPlanner::lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  const double distance = (b - a).norm();
  const double step = map_->resolution() * 0.5;
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
  const double clearance_margin = inscribed_radius_ + map_->resolution();
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d point = a + t * (b - a);
    if (map_->getEsdf(point.x(), point.y()) < clearance_margin) {
      return false;
    }
  }
  const Eigen::Vector2d midpoint = 0.5 * (a + b);
  if (checker_->safeYawIndices(midpoint.x(), midpoint.y()).empty()) {
    return false;
  }
  return true;
}

bool TopologyPlanner::configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const {
  const Eigen::Vector2d delta = b.pos - a.pos;
  const double distance = delta.norm();
  const int samples =
      std::max(2, static_cast<int>(std::ceil(distance / (map_->resolution() * 0.5))) + 1);
  const double yaw = b.has_yaw ? b.yaw : std::atan2(delta.y(), delta.x());
  const double clearance_margin = inscribed_radius_ + map_->resolution();
  for (int i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    const Eigen::Vector2d position = a.pos + t * delta;
    if (map_->getEsdf(position.x(), position.y()) < clearance_margin) {
      return false;
    }
    if (!checker_->isFree(SE2State(position.x(), position.y(), yaw))) {
      return false;
    }
  }
  return true;
}

bool TopologyPlanner::pushPointFromObstacle(TopoWaypoint& waypoint, double safe_dist) const {
  const std::vector<Eigen::Vector2d> boundary =
      checker_->footprint().sampleBoundary(waypoint.yaw, map_->resolution() * 0.5);
  if (boundary.empty()) {
    return false;
  }

  const double eps = map_->resolution();
  for (int attempt = 0; attempt < 24; ++attempt) {
    double min_distance = kInf;
    Eigen::Vector2d worst_world = waypoint.pos;
    for (const Eigen::Vector2d& sample : boundary) {
      const Eigen::Vector2d world = waypoint.pos + sample;
      const double distance = map_->getEsdf(world.x(), world.y());
      if (distance < min_distance) {
        min_distance = distance;
        worst_world = world;
      }
    }

    if (min_distance >= safe_dist &&
        checker_->isFree(SE2State(waypoint.pos.x(), waypoint.pos.y(), waypoint.yaw))) {
      waypoint.has_yaw = true;
      return true;
    }

    Eigen::Vector2d gradient = EsdfAscentDirection(*map_, worst_world, eps);
    if (gradient.norm() < 1e-9) {
      gradient = EsdfAscentDirection(*map_, waypoint.pos, eps);
    }
    if (gradient.norm() < 1e-9) {
      return false;
    }

    const double deficit = safe_dist - min_distance;
    const double push = std::min(std::max(eps, deficit + eps), 4.0 * eps);
    waypoint.pos += push * gradient.normalized();
    waypoint.has_yaw = true;
  }

  return checker_->isFree(SE2State(waypoint.pos.x(), waypoint.pos.y(), waypoint.yaw));
}

void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  const double safe_dist = inscribed_radius_ + map_->resolution();
  const double resolution = map_->resolution();

  for (TopoPath& path : paths) {
    if (path.waypoints.size() <= 2) {
      continue;
    }

    std::vector<TopoWaypoint> discrete = DiscretizePath(path.waypoints, resolution);
    for (size_t i = 1; i + 1 < discrete.size(); ++i) {
      TopoWaypoint adjusted = WithIncomingYaw(discrete[i - 1], discrete[i]);
      const double esdf = map_->getEsdf(adjusted.pos.x(), adjusted.pos.y());
      if (esdf < safe_dist ||
          !checker_->isFree(SE2State(adjusted.pos.x(), adjusted.pos.y(), adjusted.yaw))) {
        if (pushPointFromObstacle(adjusted, safe_dist)) {
          discrete[i] = WithIncomingYaw(discrete[i - 1], adjusted);
        }
      }
    }
    AssignTangentYaw(&discrete);

    std::vector<TopoWaypoint> result;
    result.push_back(discrete.front());
    size_t i = 0;
    int inserted_pushes = 0;
    const int max_inserted_pushes = static_cast<int>(discrete.size()) * 2;

    while (i + 1 < discrete.size()) {
      const TopoWaypoint anchor = result.back();

      size_t best_j = i + 1;
      bool found_shortcut = false;
      for (size_t j = discrete.size() - 1; j > i + 1; --j) {
        TopoWaypoint candidate = WithIncomingYaw(anchor, discrete[j]);
        if (configurationLineFree(anchor, candidate)) {
          best_j = j;
          found_shortcut = true;
          break;
        }
      }

      if (found_shortcut) {
        result.push_back(WithIncomingYaw(anchor, discrete[best_j]));
        i = best_j;
        continue;
      }

      const TopoWaypoint next = WithIncomingYaw(anchor, discrete[i + 1]);
      if (configurationLineFree(anchor, next)) {
        result.push_back(next);
        ++i;
        continue;
      }

      if (inserted_pushes >= max_inserted_pushes) {
        result.push_back(next);
        ++i;
        continue;
      }

      const size_t target_j = std::min(discrete.size() - 1, i + 3);
      const TopoWaypoint target = WithIncomingYaw(anchor, discrete[target_j]);
      double collision_t = FindCollisionT(anchor.pos, target.pos, *map_, safe_dist);
      if (collision_t < 0.0) {
        collision_t = 0.5;
      }

      TopoWaypoint pushed(anchor.pos + collision_t * (target.pos - anchor.pos));
      pushed = WithIncomingYaw(anchor, pushed);
      if (pushPointFromObstacle(pushed, safe_dist) &&
          configurationLineFree(anchor, WithIncomingYaw(anchor, pushed)) &&
          (pushed.pos - anchor.pos).norm() > resolution * 0.75) {
        result.push_back(WithIncomingYaw(anchor, pushed));
        ++inserted_pushes;
        continue;
      }

      result.push_back(next);
      ++i;
    }

    if ((result.back().pos - discrete.back().pos).norm() > 1e-6) {
      result.push_back(WithIncomingYaw(result.back(), discrete.back()));
    }

    for (size_t k = 1; k + 1 < result.size(); ++k) {
      TopoWaypoint adjusted = WithIncomingYaw(result[k - 1], result[k]);
      const double esdf = map_->getEsdf(adjusted.pos.x(), adjusted.pos.y());
      if (esdf < safe_dist ||
          !checker_->isFree(SE2State(adjusted.pos.x(), adjusted.pos.y(), adjusted.yaw))) {
        if (pushPointFromObstacle(adjusted, safe_dist)) {
          result[k] = WithIncomingYaw(result[k - 1], adjusted);
        }
      }
    }
    AssignTangentYaw(&result);

    for (size_t k = 1; k + 1 < result.size();) {
      const TopoWaypoint merged = WithIncomingYaw(result[k - 1], result[k + 1]);
      if (configurationLineFree(result[k - 1], merged)) {
        result.erase(result.begin() + static_cast<long>(k));
        AssignTangentYaw(&result);
        continue;
      }
      ++k;
    }

    path.waypoints = result;
    path.computeLength();
  }

  std::vector<TopoPath> filtered;
  filtered.reserve(paths.size());
  for (const TopoPath& path : paths) {
    if (path.waypoints.size() < 2) {
      continue;
    }
    if (isTopologicallyDistinct(path, filtered)) {
      filtered.push_back(path);
    }
    if (static_cast<int>(filtered.size()) >= max_paths_) {
      break;
    }
  }
  paths.swap(filtered);
}

bool TopologyPlanner::isTopologicallyDistinct(
    const TopoPath& path, const std::vector<TopoPath>& existing) const {
  const int sample_count = 40;

  const auto interpolate = [](const TopoPath& topo_path, double fraction) -> Eigen::Vector2d {
    if (topo_path.waypoints.empty()) {
      return Eigen::Vector2d::Zero();
    }
    if (topo_path.length < 1e-9) {
      return topo_path.waypoints.front().pos.eval();
    }

    const double target = fraction * topo_path.length;
    double accumulated = 0.0;
    for (size_t i = 1; i < topo_path.waypoints.size(); ++i) {
      const double segment =
          (topo_path.waypoints[i].pos - topo_path.waypoints[i - 1].pos).norm();
      if (accumulated + segment >= target) {
        const double local_fraction =
            (segment > 1e-9) ? (target - accumulated) / segment : 0.0;
        return (topo_path.waypoints[i - 1].pos +
                local_fraction *
                    (topo_path.waypoints[i].pos - topo_path.waypoints[i - 1].pos))
            .eval();
      }
      accumulated += segment;
    }
    return topo_path.waypoints.back().pos.eval();
  };

  for (const TopoPath& candidate : existing) {
    bool equivalent = true;
    for (int i = 0; i <= sample_count; ++i) {
      const double fraction = static_cast<double>(i) / static_cast<double>(sample_count);
      const Eigen::Vector2d p1 = interpolate(path, fraction);
      const Eigen::Vector2d p2 = interpolate(candidate, fraction);
      const double segment_distance = (p2 - p1).norm();
      if (segment_distance < 1e-6) {
        continue;
      }

      const int checks = std::max(
          2, static_cast<int>(std::ceil(segment_distance / (map_->resolution() * 0.5))));
      for (int s = 0; s <= checks; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(checks);
        const Eigen::Vector2d point = p1 + t * (p2 - p1);
        const GridIndex index = map_->worldToGrid(point.x(), point.y());
        if (map_->isOccupied(index.x, index.y)) {
          equivalent = false;
          break;
        }
      }
      if (!equivalent) {
        break;
      }
    }
    if (equivalent) {
      return false;
    }
  }

  return true;
}

}  // namespace esv_planner
