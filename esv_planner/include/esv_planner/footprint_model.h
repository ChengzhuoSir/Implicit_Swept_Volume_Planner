#pragma once

#include <esv_planner/common.h>
#include <Eigen/Dense>
#include <vector>

namespace esv_planner {

class FootprintModel {
public:
  FootprintModel();

  // Set polygon vertices (in body frame, CCW order)
  void setPolygon(const std::vector<Eigen::Vector2d>& vertices);

  // Inscribed circle radius
  double inscribedRadius() const { return inscribed_radius_; }
  void setInscribedRadius(double r) { inscribed_radius_ = r; }

  // Circumscribed circle radius
  double circumscribedRadius() const { return circumscribed_radius_; }

  // Compute signed distance from a point (in body frame) to the polygon boundary
  // Positive = outside, Negative = inside
  double bodyFrameSdf(const Eigen::Vector2d& point) const;

  // Get polygon vertices
  const std::vector<Eigen::Vector2d>& vertices() const { return vertices_; }

  // Get polygon rotated by yaw
  std::vector<Eigen::Vector2d> rotatedVertices(double yaw) const;

private:
  std::vector<Eigen::Vector2d> vertices_;
  double inscribed_radius_ = 0.1;
  double circumscribed_radius_ = 0.0;

  void computeCircumscribedRadius();
};

}  // namespace esv_planner
