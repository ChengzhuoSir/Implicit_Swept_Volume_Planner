#include "isweep_planner/env/polygon_sdf.h"

#include <isweep_planner/core/common.h>

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {

Eigen::Vector2d projectToSegment(const Eigen::Vector2d& point,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Vector2d& b) {
  const Eigen::Vector2d edge = b - a;
  const double denom = edge.squaredNorm();
  double t = 0.0;
  if (denom > 1e-12) {
    t = (point - a).dot(edge) / denom;
    t = std::max(0.0, std::min(1.0, t));
  }
  return a + t * edge;
}

}  // namespace

PolygonSdfQuery queryPolygonSignedDistance(
    const std::vector<Eigen::Vector2d>& vertices,
    const Eigen::Vector2d& point) {
  PolygonSdfQuery query;
  if (vertices.size() < 3) {
    return query;
  }

  query.inside = pointInPolygon(point.x(), point.y(), vertices);

  double best_distance = kInf;
  size_t best_edge = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const Eigen::Vector2d& a = vertices[i];
    const Eigen::Vector2d& b = vertices[(i + 1) % vertices.size()];
    const Eigen::Vector2d closest = projectToSegment(point, a, b);
    const double distance = (point - closest).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_edge = i;
      query.closest_point = closest;
    }
  }

  query.signed_distance = query.inside ? -best_distance : best_distance;
  if (best_distance > 1e-9) {
    query.gradient = (point - query.closest_point) / best_distance;
    if (query.inside) {
      query.gradient = -query.gradient;
    }
  } else {
    const Eigen::Vector2d edge =
        vertices[(best_edge + 1) % vertices.size()] - vertices[best_edge];
    const double len = edge.norm();
    if (len > 1e-12) {
      query.gradient = Eigen::Vector2d(edge.y(), -edge.x()) / len;
    }
  }

  return query;
}

}  // namespace isweep_planner
