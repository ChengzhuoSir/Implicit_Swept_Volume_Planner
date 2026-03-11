#pragma once

#include <esv_planner/common.h>
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace esv_planner {

struct BodyFrameQuery {
  double signed_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  double winding_number = 0.0;
  bool inside = false;
};

namespace body_frame_sdf_detail {

inline double generalizedWindingNumber(const Eigen::Vector2d& point,
                                       const std::vector<Eigen::Vector2d>& verts) {
  if (verts.size() < 3) return 0.0;
  double angle_sum = 0.0;
  for (size_t i = 0; i < verts.size(); ++i) {
    const Eigen::Vector2d a = verts[i] - point;
    const Eigen::Vector2d b = verts[(i + 1) % verts.size()] - point;
    const double cross = a.x() * b.y() - a.y() * b.x();
    const double dot = a.dot(b);
    angle_sum += std::atan2(cross, dot);
  }
  return angle_sum / (2.0 * kPi);
}

inline bool pointInsideByWinding(const Eigen::Vector2d& point,
                                 const std::vector<Eigen::Vector2d>& verts,
                                 double* winding_number = nullptr) {
  const double w = generalizedWindingNumber(point, verts);
  if (winding_number) *winding_number = w;
  return std::abs(w) > 0.5;
}

inline Eigen::Vector2d projectToSegment(const Eigen::Vector2d& point,
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

inline Eigen::Vector2d outwardUnitNormal(const Eigen::Vector2d& a,
                                         const Eigen::Vector2d& b) {
  const Eigen::Vector2d edge = b - a;
  const double len = edge.norm();
  if (len < 1e-12) return Eigen::Vector2d::Zero();
  return Eigen::Vector2d(edge.y(), -edge.x()) / len;
}

}  // namespace body_frame_sdf_detail

class BodyFrameSdf {
public:
  BodyFrameSdf() = default;

  void setPolygon(const std::vector<Eigen::Vector2d>& vertices) { vertices_ = vertices; }

  const std::vector<Eigen::Vector2d>& vertices() const { return vertices_; }

  double signedDistance(const Eigen::Vector2d& point) const {
    return query(point).signed_distance;
  }

  BodyFrameQuery query(const Eigen::Vector2d& point) const {
    BodyFrameQuery q;
    if (vertices_.size() < 3) return q;

    q.inside = body_frame_sdf_detail::pointInsideByWinding(
        point, vertices_, &q.winding_number);

    double best_dist = std::numeric_limits<double>::infinity();
    size_t best_edge = 0;
    Eigen::Vector2d best_closest = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < vertices_.size(); ++i) {
      const Eigen::Vector2d& a = vertices_[i];
      const Eigen::Vector2d& b = vertices_[(i + 1) % vertices_.size()];
      const Eigen::Vector2d closest =
          body_frame_sdf_detail::projectToSegment(point, a, b);
      const double dist = (point - closest).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_closest = closest;
        best_edge = i;
      }
    }

    q.closest_point = best_closest;
    q.signed_distance = q.inside ? -best_dist : best_dist;

    if (best_dist > 1e-9) {
      q.gradient = (point - best_closest) / best_dist;
      if (q.inside) q.gradient = -q.gradient;
    } else {
      q.gradient = body_frame_sdf_detail::outwardUnitNormal(
          vertices_[best_edge], vertices_[(best_edge + 1) % vertices_.size()]);
    }
    return q;
  }

private:
  std::vector<Eigen::Vector2d> vertices_;
};

}  // namespace esv_planner
