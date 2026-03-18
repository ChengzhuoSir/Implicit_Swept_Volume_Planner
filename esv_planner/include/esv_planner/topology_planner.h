#pragma once

#include <set>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "esv_planner/collision_checker.h"
#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"

namespace esv_planner {

class TopologyPlanner {
 public:
  TopologyPlanner();

  void init(const GridMap& map, const CollisionChecker& checker, int num_samples, int knn,
            int max_paths, double inscribed_radius);

  void buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
  std::vector<TopoPath> searchPaths();
  std::vector<TopoPath> searchPathsAvoidingRegion(const Eigen::Vector2d& center,
                                                  double radius);
  std::vector<TopoPath> searchPathsAvoidingCenters(
      const std::vector<Eigen::Vector2d>& centers, double radius);
  void shortenPaths(std::vector<TopoPath>& paths);

 private:
  bool lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
  bool configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const;
  bool isTopologicallyDistinct(const TopoPath& path,
                               const std::vector<TopoPath>& existing) const;
  bool pushPointFromObstacle(TopoWaypoint& waypoint, double safe_dist) const;
  TopoPath dijkstra(int src, int dst, const std::vector<double>& node_penalty,
                    const std::set<std::pair<int, int>>& blocked_edges,
                    const std::unordered_set<int>& blocked_nodes) const;
  std::vector<TopoPath> searchPathsImpl(
      const std::set<std::pair<int, int>>& blocked_edges,
      const std::unordered_set<int>& blocked_nodes) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  int num_samples_ = 520;
  int knn_ = 18;
  int max_paths_ = 14;
  double inscribed_radius_ = 0.1;

  std::vector<Eigen::Vector2d> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adjacency_;
};

}  // namespace esv_planner
