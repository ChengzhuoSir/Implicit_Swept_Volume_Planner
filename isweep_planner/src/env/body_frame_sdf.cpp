#include "isweep_planner/env/body_frame_sdf.h"

#include <algorithm>

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

void BodyFrameSdf::setPolygon(const std::vector<Eigen::Vector2d>& vertices) {
  vertices_ = vertices;
}

double BodyFrameSdf::signedDistance(const Eigen::Vector2d& point) const {
  return query(point).signed_distance;
}

BodyFrameSdf::NearestEdgeResult BodyFrameSdf::nearestEdge(
    const Eigen::Vector2d& point) const {
  NearestEdgeResult result;
  if (vertices_.size() < 2) return result;

  for (size_t i = 0; i < vertices_.size(); ++i) {
    const Eigen::Vector2d& a = vertices_[i];
    const Eigen::Vector2d& b = vertices_[(i + 1) % vertices_.size()];
    const Eigen::Vector2d closest = projectToSegment(point, a, b);
    const double distance = (point - closest).norm();
    if (distance < result.distance) {
      result.distance = distance;
      result.edge_index = i;
      result.closest_point = closest;
    }
  }
  return result;
}

Eigen::Vector2d BodyFrameSdf::outwardNormal(size_t edge_index) const {
  if (vertices_.size() < 2) return Eigen::Vector2d::Zero();
  const Eigen::Vector2d& a = vertices_[edge_index];
  const Eigen::Vector2d& b = vertices_[(edge_index + 1) % vertices_.size()];
  const Eigen::Vector2d edge = b - a;
  const double len = edge.norm();
  if (len < 1e-12) return Eigen::Vector2d::Zero();
  return Eigen::Vector2d(edge.y(), -edge.x()) / len;
}

std::vector<BodyFrameQuery> BodyFrameSdf::queryBatch(
    const Eigen::Ref<const Eigen::MatrixXd>& points) const {
  std::vector<BodyFrameQuery> result;
  result.resize(static_cast<size_t>(points.rows()));
  if (vertices_.size() < 3 || points.cols() != 2) return result;

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const Eigen::Vector2d point = points.row(i).transpose();
    const PolygonSdfQuery polygon_query = queryPolygonSignedDistance(vertices_, point);

    BodyFrameQuery q;
    q.signed_distance = polygon_query.signed_distance;
    q.closest_point = polygon_query.closest_point;
    q.gradient = polygon_query.gradient;
    q.inside = polygon_query.inside;
    q.winding_number = polygon_query.inside ? 1.0 : 0.0;
    result[static_cast<size_t>(i)] = q;
  }

  return result;
}

BodyFrameQuery BodyFrameSdf::query(const Eigen::Vector2d& point) const {
  Eigen::Matrix<double, 1, 2> sample;
  sample.row(0) = point.transpose();
  const auto queries = queryBatch(sample);
  if (queries.empty()) return BodyFrameQuery{};
  return queries.front();
}

}  // namespace isweep_planner
