#pragma once

#include <esv_planner/grid_map.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

class GridObstacleSource {
public:
  GridObstacleSource() = default;

  void init(const GridMap& map);

  const GridMap* map() const { return map_; }
  const std::vector<Eigen::Vector2d>& obstaclePoints() const { return obstacle_points_; }
  size_t size() const { return obstacle_points_.size(); }

  void queryAabb(const Eigen::Vector2d& min_corner,
                 const Eigen::Vector2d& max_corner,
                 std::vector<Eigen::Vector2d>* out_points) const;

private:
  const GridMap* map_ = nullptr;
  std::vector<Eigen::Vector2d> obstacle_points_;
};

}  // namespace esv_planner
