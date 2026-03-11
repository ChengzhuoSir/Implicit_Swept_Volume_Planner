#pragma once

#include <esv_planner/common.h>
#include <esv_planner/continuous_svsdf_evaluator.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <vector>

namespace esv_planner {

class TopologyPlanner {
public:
  TopologyPlanner();

  void init(const GridMap& map, const CollisionChecker& checker,
            int num_samples, int knn, int max_paths,
            double inscribed_radius);

  // Build PRM roadmap
  void buildRoadmap(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);

  // Search for topologically distinct paths (Dijkstra + penalty)
  std::vector<TopoPath> searchPaths();

  // Geometry-aware path shortcut (Algorithm 1) with ESDF push-off
  void shortenPaths(std::vector<TopoPath>& paths);

  // Exposed for focused regression of the body-frame push stage.
  bool repairWaypointBodyFrame(TopoWaypoint& wp, double safe_dist) const;

  size_t numNodes() const { return nodes_.size(); }
  int startDegree() const {
    return adjacency_.empty() ? 0 : static_cast<int>(adjacency_[0].size());
  }
  int goalDegree() const {
    return adjacency_.size() < 2 ? 0 : static_cast<int>(adjacency_[1].size());
  }

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  int num_samples_ = 520;
  int knn_ = 18;
  int max_paths_ = 14;
  double inscribed_radius_ = 0.1;
  ContinuousSvsdfEvaluator evaluator_;

  // PRM graph
  std::vector<Eigen::Vector2d> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adjacency_;  // (neighbor, weight)

  // Helpers
  bool lineCollisionFree(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
  bool isTopologicallyDistinct(const TopoPath& path, const std::vector<TopoPath>& existing) const;

  // Dijkstra shortest path
  TopoPath dijkstra(int src, int dst, const std::vector<double>& edge_penalty) const;

  // Algorithm 1: push a waypoint away using footprint-aware obstacle queries.
  bool pushPointFromObstacle(TopoWaypoint& wp, double safe_dist) const;
  double configurationClearance(const TopoWaypoint& a, const TopoWaypoint& b) const;
  bool configurationLineFree(const TopoWaypoint& a, const TopoWaypoint& b) const;
};

}  // namespace esv_planner
