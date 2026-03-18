#pragma once

#include <esv_planner/collision_checker.h>
#include <esv_planner/grid_map.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct AstarSearchResult {
  bool success = false;
  std::vector<Eigen::Vector2d> path;
  int expanded_nodes = 0;
};

struct SweptAstarSearchOptions {
  std::vector<Eigen::Vector2d> blocked_centers;
  double blocked_radius = 0.0;
  int min_safe_yaw_count = 0;
  int min_x = -1;
  int min_y = -1;
  int max_x = -1;
  int max_y = -1;
  double preferred_clearance = 0.0;
  double clearance_penalty = 0.0;
  double blocked_center_penalty = 0.0;
};

class SweptAstar {
public:
  SweptAstar() = default;

  void init(const GridMap& map, double required_clearance);
  void init(const GridMap& map, const CollisionChecker& checker, double required_clearance);

  AstarSearchResult search(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& goal) const;
  AstarSearchResult search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                           const SweptAstarSearchOptions& options) const;

private:
  bool isTraversable(int gx, int gy) const;
  bool isTraversable(int gx, int gy,
                     const SweptAstarSearchOptions& options) const;
  bool withinBounds(int gx, int gy,
                    const SweptAstarSearchOptions& options) const;
  double traversalPenalty(int gx, int gy,
                          const SweptAstarSearchOptions& options) const;
  double heuristic(int gx, int gy, int goal_x, int goal_y) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  double required_clearance_ = 0.0;
};

}  // namespace esv_planner
