#pragma once

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct PolygonSdfQuery {
  double signed_distance = 0.0;
  Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  bool inside = false;
};

PolygonSdfQuery queryPolygonSignedDistance(
    const std::vector<Eigen::Vector2d>& vertices,
    const Eigen::Vector2d& point);

}  // namespace esv_planner
