#include "esv_planner/grid_obstacle_source.h"

#include <algorithm>

namespace esv_planner {

void GridObstacleSource::init(const GridMap& map) {
  map_ = &map;
  obstacle_points_.clear();
  if (!map_) {
    return;
  }

  obstacle_points_.reserve(static_cast<size_t>(map.width() * map.height() / 4));
  for (int gy = 0; gy < map.height(); ++gy) {
    for (int gx = 0; gx < map.width(); ++gx) {
      if (!map.isRawOccupied(gx, gy)) {
        continue;
      }
      obstacle_points_.push_back(map.gridToWorld(gx, gy));
    }
  }
}

void GridObstacleSource::queryAabb(const Eigen::Vector2d& min_corner,
                                   const Eigen::Vector2d& max_corner,
                                   std::vector<Eigen::Vector2d>* out_points) const {
  if (!out_points) {
    return;
  }
  out_points->clear();
  if (!map_) {
    return;
  }

  GridIndex min_idx = map_->worldToGrid(min_corner.x(), min_corner.y());
  GridIndex max_idx = map_->worldToGrid(max_corner.x(), max_corner.y());
  const int min_x = std::max(0, std::min(min_idx.x, max_idx.x));
  const int max_x = std::min(map_->width() - 1, std::max(min_idx.x, max_idx.x));
  const int min_y = std::max(0, std::min(min_idx.y, max_idx.y));
  const int max_y = std::min(map_->height() - 1, std::max(min_idx.y, max_idx.y));

  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      if (!map_->isRawOccupied(gx, gy)) {
        continue;
      }
      out_points->push_back(map_->gridToWorld(gx, gy));
    }
  }
}

}  // namespace esv_planner
