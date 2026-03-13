#include "esv_planner/local_obstacle_cache.h"

#include <set>
#include <utility>

namespace esv_planner {

void LocalObstacleCache::build(const GridObstacleSource& obstacle_source,
                               const std::vector<SE2State>& states,
                               double half_extent) {
  points_.clear();
  if (states.empty() || obstacle_source.map() == nullptr) {
    return;
  }

  std::set<std::pair<int, int> > unique_cells;
  std::vector<Eigen::Vector2d> local_points;
  for (size_t i = 0; i < states.size(); ++i) {
    const Eigen::Vector2d center = states[i].position();
    obstacle_source.queryAabb(center - Eigen::Vector2d::Constant(half_extent),
                              center + Eigen::Vector2d::Constant(half_extent),
                              &local_points);
    for (size_t j = 0; j < local_points.size(); ++j) {
      const GridIndex cell =
          obstacle_source.map()->worldToGrid(local_points[j].x(), local_points[j].y());
      const std::pair<int, int> key(cell.x, cell.y);
      if (unique_cells.insert(key).second) {
        points_.push_back(local_points[j]);
      }
    }
  }
}

}  // namespace esv_planner
