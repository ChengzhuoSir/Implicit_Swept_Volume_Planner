#include "esv_planner/body_frame_sdf.h"

#include <igl/winding_number.h>

#include <algorithm>
#include <cmath>

namespace esv_planner {

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
  winding_vertices_.resize(static_cast<Eigen::Index>(vertices_.size()), 2);
  winding_edges_.resize(static_cast<Eigen::Index>(vertices_.size()), 2);
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(vertices_.size()); ++i) {
    winding_vertices_.row(i) = vertices_[static_cast<size_t>(i)].transpose();
    winding_edges_(i, 0) = static_cast<int>(i);
    winding_edges_(i, 1) = static_cast<int>((i + 1) % static_cast<Eigen::Index>(vertices_.size()));
  }
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

  Eigen::VectorXd winding(points.rows());
  igl::winding_number(winding_vertices_, winding_edges_, points, winding);

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    BodyFrameQuery q;
    q.winding_number = winding(i);
    q.inside = std::abs(q.winding_number) > 0.5;

    const Eigen::Vector2d point = points.row(i).transpose();
    const NearestEdgeResult nearest = nearestEdge(point);
    q.closest_point = nearest.closest_point;
    q.signed_distance = q.inside ? -nearest.distance : nearest.distance;

    if (nearest.distance > 1e-9) {
      q.gradient = (point - nearest.closest_point) / nearest.distance;
      if (q.inside) q.gradient = -q.gradient;
    } else {
      q.gradient = outwardNormal(nearest.edge_index);
    }

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

}  // namespace esv_planner
