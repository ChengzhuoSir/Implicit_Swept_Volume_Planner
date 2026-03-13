#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_obstacle_source.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

class LocalObstacleCache {
public:
  void build(const GridObstacleSource& obstacle_source,
             const std::vector<SE2State>& states,
             double half_extent);

  const std::vector<Eigen::Vector2d>& points() const { return points_; }
  size_t size() const { return points_.size(); }

private:
  std::vector<Eigen::Vector2d> points_;
};

}  // namespace esv_planner
