#pragma once

#include <memory>
#include <set>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/core/common.h"
#include "isweep_planner/env/grid_map.h"

namespace isweep_planner {

// Forward-declare KD-tree wrapper (defined in topology_planner.cpp)
struct KDTreeWrapper;

class TopologyPlanner {
 public:
  TopologyPlanner();
  ~TopologyPlanner();

  void init(const GridMap& map, const CollisionChecker& checker, int num_samples, int knn,
            int max_paths, double inscribed_radius);

  void buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
  std::vector<TopoPath> searchPaths();
  void shortenPaths(std::vector<TopoPath>& paths);

 private:
  bool lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
  bool configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const;
  bool isTopologicallyDistinct(const TopoPath& path,
                               const std::vector<TopoPath>& existing) const;
  bool pushPointFromObstacle(TopoWaypoint& waypoint, double safe_dist, const Eigen::Vector2d& tension = Eigen::Vector2d::Zero()) const;
  void rebuildBaseRoadmap();
  TopoPath dijkstra(int src, int dst, const std::vector<double>& node_penalty,
                    const std::set<std::pair<int, int>>& blocked_edges,
                    const std::unordered_set<int>& blocked_nodes,
                    std::vector<int>* out_indices = nullptr) const;
  std::vector<TopoPath> searchPathsImpl(
      const std::set<std::pair<int, int>>& blocked_edges,
      const std::unordered_set<int>& blocked_nodes) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  int num_samples_ = 520;
  int knn_ = 18;
  int max_paths_ = 14;
  double inscribed_radius_ = 0.1;
  bool base_roadmap_ready_ = false;
  int cached_width_ = 0;
  int cached_height_ = 0;
  double cached_resolution_ = 0.0;
  double cached_origin_x_ = 0.0;
  double cached_origin_y_ = 0.0;

  std::vector<Eigen::Vector2d> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adjacency_;
  std::vector<Eigen::Vector2d> base_nodes_;
  std::vector<std::vector<std::pair<int, double>>> base_adjacency_;
  int base_total_edges_ = 0;

  // KD-tree for fast KNN (pimpl to avoid nanoflann/PI macro conflict)
  std::unique_ptr<KDTreeWrapper> kdtree_;
};

}  // namespace isweep_planner
