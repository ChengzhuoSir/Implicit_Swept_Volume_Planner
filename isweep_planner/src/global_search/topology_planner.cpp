#include "isweep_planner/global_search/topology_planner.h"
#include "isweep_planner/core/math_utils.h"
#include "isweep_planner/third_party/nanoflann.hpp"

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

namespace isweep_planner {

// KD-tree wrapper (pimpl to isolate nanoflann from headers with PI macro)
struct PointCloud2D {
  const std::vector<Eigen::Vector2d>* pts = nullptr;
  size_t kdtree_get_point_count() const { return pts ? pts->size() : 0; }
  double kdtree_get_pt(size_t idx, size_t dim) const { return (*pts)[idx](dim); }
  template <class B> bool kdtree_get_bbox(B&) const { return false; }
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>;

struct KDTreeWrapper {
  PointCloud2D cloud;
  std::unique_ptr<KDTree2D> tree;

  void build(const std::vector<Eigen::Vector2d>& nodes) {
    cloud.pts = &nodes;
    tree = std::make_unique<KDTree2D>(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree->buildIndex();
  }

  // Returns number of results found; fills out_indices and out_dists (squared distances)
  size_t knnSearch(const double* query, size_t k,
                   std::vector<unsigned int>& out_indices,
                   std::vector<double>& out_dists) const {
    out_indices.resize(k);
    out_dists.resize(k);
    return tree->knnSearch(query, k, out_indices.data(), out_dists.data());
  }
};

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

bool ShouldPreserveWaypoint(const TopoWaypoint& waypoint, const GridMap& map,
                            const CollisionChecker& checker, double preserve_clearance,
                            int min_safe_yaw_count) {
  const double clearance = map.getEsdf(waypoint.pos.x(), waypoint.pos.y());
  if (!std::isfinite(clearance) || clearance < preserve_clearance) {
    return true;
  }
  return static_cast<int>(
             checker.safeYawCount(waypoint.pos.x(), waypoint.pos.y())) <
         min_safe_yaw_count;
}

bool RangeContainsPreservedWaypoint(const std::vector<int>& preserve_prefix, size_t start,
                                    size_t end) {
  if (start >= end || end >= preserve_prefix.size()) {
    return false;
  }
  const size_t query_begin = start + 1;
  if (query_begin >= end) {
    return false;
  }
  return preserve_prefix[end] > preserve_prefix[query_begin];
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

TopologyPlanner::~TopologyPlanner() = default;

void TopologyPlanner::init(const GridMap& map, const CollisionChecker& checker, int num_samples,
                           int knn, int max_paths, double inscribed_radius) {
  const bool params_changed =
      map_ == nullptr || num_samples_ != num_samples || knn_ != knn ||
      max_paths_ != max_paths || std::abs(inscribed_radius_ - inscribed_radius) > 1e-9;
  const bool map_changed =
      map_ == nullptr || cached_width_ != map.width() ||
      cached_height_ != map.height() ||
      std::abs(cached_resolution_ - map.resolution()) > 1e-9 ||
      std::abs(cached_origin_x_ - map.originX()) > 1e-9 ||
      std::abs(cached_origin_y_ - map.originY()) > 1e-9;
  map_ = &map;
  checker_ = &checker;
  num_samples_ = num_samples;
  knn_ = knn;
  max_paths_ = max_paths;
  inscribed_radius_ = inscribed_radius;
  cached_width_ = map.width();
  cached_height_ = map.height();
  cached_resolution_ = map.resolution();
  cached_origin_x_ = map.originX();
  cached_origin_y_ = map.originY();
  if (!base_roadmap_ready_ || params_changed || map_changed) {
    rebuildBaseRoadmap();
  }
  ROS_INFO("TopologyPlanner initialized: num_samples=%d, knn=%d, max_paths=%d, inscribed_radius=%.3f",
           num_samples_, knn_, max_paths_, inscribed_radius_);
}

void TopologyPlanner::rebuildBaseRoadmap() {
  base_nodes_.clear();
  base_adjacency_.clear();
  base_total_edges_ = 0;
  if (map_ == nullptr || checker_ == nullptr) {
    base_roadmap_ready_ = false;
    return;
  }
  const double x_min = map_->originX();
  const double y_min = map_->originY();
  const double x_max = x_min + static_cast<double>(map_->width()) * map_->resolution();
  const double y_max = y_min + static_cast<double>(map_->height()) * map_->resolution();

  base_nodes_.reserve(static_cast<size_t>(num_samples_));
  int accepted = 0;
  for (int i = 1; accepted < num_samples_; ++i) {
    const double x = x_min + HaltonSequence(i, 2) * (x_max - x_min);
    const double y = y_min + HaltonSequence(i, 3) * (y_max - y_min);
    const double node_clearance = inscribed_radius_ + map_->resolution();
    if (map_->getEsdf(x, y) > node_clearance) {
      base_nodes_.push_back(Eigen::Vector2d(x, y));
      ++accepted;
    }
  }

  base_adjacency_.assign(base_nodes_.size(), std::vector<std::pair<int, double>>());

  // Build KD-tree for fast KNN
  kdtree_ = std::make_unique<KDTreeWrapper>();
  kdtree_->build(base_nodes_);

  const size_t search_k = static_cast<size_t>(knn_ * 3);
  std::vector<unsigned int> indices;
  std::vector<double> dists;
  int total_edges = 0;
  for (size_t i = 0; i < base_nodes_.size(); ++i) {
    const double query[2] = {base_nodes_[i].x(), base_nodes_[i].y()};
    const size_t found = kdtree_->knnSearch(query, search_k, indices, dists);

    int connections = 0;
    for (size_t m = 0; m < found && connections < knn_; ++m) {
      const size_t j = indices[m];
      if (j == i) continue;
      const double dist = std::sqrt(dists[m]);
      if (lineCollisionFree(base_nodes_[i], base_nodes_[j])) {
        base_adjacency_[i].push_back(std::make_pair(static_cast<int>(j), dist));
        ++connections;
        ++total_edges;
      }
    }
  }
  base_total_edges_ = total_edges;
  base_roadmap_ready_ = true;
  ROS_INFO("Topology base roadmap cached: nodes=%zu, edges=%d",
           base_nodes_.size(), total_edges);
}

void TopologyPlanner::buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
  if (!base_roadmap_ready_) {
    rebuildBaseRoadmap();
  }

  nodes_.clear();
  adjacency_.clear();
  nodes_.reserve(base_nodes_.size() + 2);
  nodes_.push_back(start);
  nodes_.push_back(goal);
  nodes_.insert(nodes_.end(), base_nodes_.begin(), base_nodes_.end());

  adjacency_.assign(nodes_.size(), std::vector<std::pair<int, double>>());
  int special_edges = 0;

  const auto connect_special_node = [&](int special_index) {
    const double query[2] = {nodes_[static_cast<size_t>(special_index)].x(),
                             nodes_[static_cast<size_t>(special_index)].y()};
    const size_t search_k = static_cast<size_t>(knn_ * 3);
    std::vector<unsigned int> knn_indices;
    std::vector<double> knn_dists;
    const size_t found = kdtree_->knnSearch(query, search_k, knn_indices, knn_dists);

    int connections = 0;
    for (size_t m = 0; m < found && connections < knn_; ++m) {
      const size_t j = knn_indices[m];
      const int node_index = static_cast<int>(j + 2);
      const double dist = std::sqrt(knn_dists[m]);
      if (!lineCollisionFree(nodes_[static_cast<size_t>(special_index)],
                             nodes_[static_cast<size_t>(node_index)])) {
        continue;
      }
      adjacency_[static_cast<size_t>(special_index)].push_back(
          std::make_pair(node_index, dist));
      adjacency_[static_cast<size_t>(node_index)].push_back(
          std::make_pair(special_index, dist));
      special_edges += 2;
      ++connections;
    }
  };

  connect_special_node(0);
  connect_special_node(1);
  if (lineCollisionFree(start, goal)) {
    const double direct_distance = (goal - start).norm();
    adjacency_[0].push_back(std::make_pair(1, direct_distance));
    adjacency_[1].push_back(std::make_pair(0, direct_distance));
    special_edges += 2;
  }

  const int total_edges = base_total_edges_ + special_edges;
  ROS_INFO("Topology Roadmap: nodes=%zu, edges=%d, start_neighbors=%zu, goal_neighbors=%zu",
           nodes_.size(), total_edges, adjacency_[0].size(), adjacency_[1].size());
}

TopoPath TopologyPlanner::dijkstra(
    int src, int dst, const std::vector<double>& node_penalty,
    const std::set<std::pair<int, int>>& blocked_edges,
    const std::unordered_set<int>& blocked_nodes,
    std::vector<int>* out_indices) const {
  const int n = static_cast<int>(nodes_.size());
  std::vector<double> distance(static_cast<size_t>(n), kInf);
  std::vector<int> previous(static_cast<size_t>(n), -1);
  distance[static_cast<size_t>(src)] = 0.0;

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  open.push(std::make_pair(0.0, src));

  auto relax_edge = [&](int current, int next, double weight) {
    if (blocked_nodes.count(next) && next != dst) {
      return;
    }
    if (blocked_edges.count(std::make_pair(current, next))) {
      return;
    }

    if (!node_penalty.empty()) {
      weight += node_penalty[static_cast<size_t>(next)];
    }
    
    // Voronoi Ridge Bias: add penalty for nodes close to obstacles
    const double esdf = map_->getEsdf(nodes_[static_cast<size_t>(next)].x(), nodes_[static_cast<size_t>(next)].y());
    const double safe_margin = inscribed_radius_ * 2.0;
    if (esdf < safe_margin && esdf > 0.0) {
      weight += (safe_margin - esdf) * 1.5;
    }

    const double candidate = distance[static_cast<size_t>(current)] + weight;
    if (candidate < distance[static_cast<size_t>(next)]) {
      distance[static_cast<size_t>(next)] = candidate;
      previous[static_cast<size_t>(next)] = current;
      open.push(std::make_pair(candidate, next));
    }
  };

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

    if (current >= 2) {
      const size_t base_index = static_cast<size_t>(current - 2);
      for (const auto& edge : base_adjacency_[base_index]) {
        relax_edge(current, edge.first + 2, edge.second);
      }
    }
    for (const auto& edge : adjacency_[static_cast<size_t>(current)]) {
      relax_edge(current, edge.first, edge.second);
    }
  }

  TopoPath path;
  if (!std::isfinite(distance[static_cast<size_t>(dst)])) {
    if (out_indices != nullptr) {
      out_indices->clear();
    }
    return path;
  }

  std::vector<int> indices;
  for (int v = dst; v != -1; v = previous[static_cast<size_t>(v)]) {
    indices.push_back(v);
  }
  std::reverse(indices.begin(), indices.end());
  if (out_indices != nullptr) {
    *out_indices = indices;
  }
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

  for (size_t i = 0; i < nodes_.size(); ++i) {
    const auto inspect_edge = [&](int next) {
      for (const Eigen::Vector2d& center : centers) {
        if (DistanceToSegment(center, nodes_[i], nodes_[static_cast<size_t>(next)]) <=
            inflated_radius) {
          blocked_edges.insert(std::make_pair(static_cast<int>(i), next));
          break;
        }
      }
    };

    if (i >= 2) {
      const size_t base_index = i - 2;
      for (const auto& edge : base_adjacency_[base_index]) {
        inspect_edge(edge.first + 2);
      }
    }
    for (const auto& edge : adjacency_[i]) {
      inspect_edge(edge.first);
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

  for (int iter = 0; iter < max_paths_ * 6; ++iter) {
    std::vector<int> indices;
    TopoPath path = dijkstra(src, dst, node_penalty, blocked_edges, blocked_nodes,
                             &indices);
    if (path.waypoints.size() < 2 || indices.size() < 2) {
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

bool TopologyPlanner::pushPointFromObstacle(TopoWaypoint& waypoint, double safe_dist, const Eigen::Vector2d& tension) const {
  const std::vector<Eigen::Vector2d> boundary =
      checker_->footprint().sampleBoundary(waypoint.yaw, map_->resolution() * 0.5);
  if (boundary.empty()) {
    return false;
  }

  const double eps = map_->resolution();
  Eigen::Vector2d momentum = Eigen::Vector2d::Zero();
  
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

    Eigen::Vector2d gradient = math::EsdfAscentDirection(*map_, worst_world, eps);
    if (gradient.norm() < 1e-9) {
      gradient = math::EsdfAscentDirection(*map_, waypoint.pos, eps);
    }
    
    Eigen::Vector2d combined_dir = Eigen::Vector2d::Zero();
    Eigen::Vector2d tension_term = (tension.norm() > 1e-9) ? Eigen::Vector2d(tension.normalized() * 0.3) : Eigen::Vector2d::Zero();
    if (gradient.norm() >= 1e-9) {
      combined_dir = (gradient.normalized() * 1.0 + tension_term + momentum * 0.5).normalized();
    } else if (tension.norm() > 1e-9 || momentum.norm() > 1e-9) {
      combined_dir = (tension_term + momentum * 0.5).normalized();
    }
    
    if (combined_dir.norm() < 1e-9) {
      return false;
    }

    const double deficit = safe_dist - min_distance;
    const double push = std::min(std::max(eps, deficit + eps), 4.0 * eps);
    waypoint.pos += push * combined_dir;
    momentum = combined_dir;
    waypoint.has_yaw = true;
  }

  return checker_->isFree(SE2State(waypoint.pos.x(), waypoint.pos.y(), waypoint.yaw));
}

void TopologyPlanner::shortenPaths(std::vector<TopoPath>& paths) {
  const double safe_dist = inscribed_radius_ + map_->resolution();
  const double resolution = map_->resolution();
  const double preserve_clearance = safe_dist + map_->resolution();
  const int min_safe_yaw_count = 4;

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
        Eigen::Vector2d tension = (discrete[i - 1].pos + discrete[i + 1].pos) * 0.5 - adjusted.pos;
        if (pushPointFromObstacle(adjusted, safe_dist, tension)) {
          discrete[i] = WithIncomingYaw(discrete[i - 1], adjusted);
        }
      }
    }
    AssignTangentYaw(&discrete);
    std::vector<int> discrete_preserve_prefix(discrete.size() + 1, 0);
    for (size_t idx = 0; idx < discrete.size(); ++idx) {
      discrete_preserve_prefix[idx + 1] =
          discrete_preserve_prefix[idx] +
          (ShouldPreserveWaypoint(discrete[idx], *map_, *checker_, preserve_clearance,
                                  min_safe_yaw_count)
               ? 1
               : 0);
    }

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
        if (RangeContainsPreservedWaypoint(discrete_preserve_prefix, i, j)) {
          continue;
        }
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
      Eigen::Vector2d tension = (anchor.pos + target.pos) * 0.5 - pushed.pos;
      if (pushPointFromObstacle(pushed, safe_dist, tension) &&
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
        Eigen::Vector2d tension = (result[k - 1].pos + result[k + 1].pos) * 0.5 - adjusted.pos;
        if (pushPointFromObstacle(adjusted, safe_dist, tension)) {
          result[k] = WithIncomingYaw(result[k - 1], adjusted);
        }
      }
    }
    AssignTangentYaw(&result);

    for (size_t k = 1; k + 1 < result.size();) {
      if (ShouldPreserveWaypoint(result[k], *map_, *checker_, preserve_clearance,
                                 min_safe_yaw_count)) {
        ++k;
        continue;
      }
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
  const double clearance_margin = inscribed_radius_ + 2.0 * map_->resolution();

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
        if (map_->getEsdf(point.x(), point.y()) < clearance_margin) {
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

}  // namespace isweep_planner
