#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <vector>

namespace esv_planner {

class TopologyPlanner {
public:
  TopologyPlanner();

  void init(const GridMap& map, const CollisionChecker& checker,
            int num_samples, int knn, int max_paths);

  // Build PRM roadmap
  void buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);

  // Search for topologically distinct paths
  std::vector<TopoPath> searchPaths();

  // Geometry-aware path shortcut (Algorithm 1)
  void shortenPaths(std::vector<TopoPath>& paths);

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  int num_samples_ = 520;
  int knn_ = 18;
  int max_paths_ = 14;

  // PRM graph
  std::vector<Eigen::Vector2d> nodes_;
  std::vector<std::vector<int>> adjacency_;

  // Helpers
  bool lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
  bool isTopologicallyDistinct(const TopoPath& path, const std::vector<TopoPath>& existing) const;
};

}  // namespace esv_planner
