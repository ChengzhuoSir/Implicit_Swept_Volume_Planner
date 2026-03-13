#pragma once

#include <esv_planner/grid_map.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct AstarSearchResult {
  bool success = false;
  std::vector<Eigen::Vector2d> path;
  int expanded_nodes = 0;
};

class SweptAstar {
public:
  SweptAstar() = default;

  void init(const GridMap& map, double required_clearance);

  AstarSearchResult search(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& goal) const;

private:
  bool isTraversable(int gx, int gy) const;
  double heuristic(int gx, int gy, int goal_x, int goal_y) const;

  const GridMap* map_ = nullptr;
  double required_clearance_ = 0.0;
};

}  // namespace esv_planner
