#pragma once

#include <Eigen/Core>
#include "isweep_planner/env/grid_map.h"

namespace isweep_planner {
namespace math {

// Computes the normalized gradient (ascent direction) of the ESDF at the given world position.
// If the local gradient is zero (e.g., deep inside an obstacle), it performs a radial
// search to find the nearest direction pointing towards free space.
Eigen::Vector2d EsdfAscentDirection(const GridMap& map, const Eigen::Vector2d& world, double eps);

// Pushes a 2D point away from obstacles towards higher ESDF values.
// Returns true if the point was pushed to a safe distance, false otherwise.
bool PushPointFromObstacle(const GridMap& map, Eigen::Vector2d& pos, 
                           double target_clearance, double step_size, int max_iters);

} // namespace math
} // namespace isweep_planner