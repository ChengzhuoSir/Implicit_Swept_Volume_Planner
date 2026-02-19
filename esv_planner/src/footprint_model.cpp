#include "esv_planner/footprint_model.h"
#include <cmath>
#include <limits>

namespace esv_planner {

FootprintModel::FootprintModel() {}

void FootprintModel::setPolygon(const std::vector<Eigen::Vector2d>& vertices) {
  vertices_ = vertices;
  computeCircumscribedRadius();
}

void FootprintModel::computeCircumscribedRadius() {
  circumscribed_radius_ = 0.0;
  for (const auto& v : vertices_) {
    double r = v.norm();
    if (r > circumscribed_radius_) circumscribed_radius_ = r;
  }
}

double FootprintModel::bodyFrameSdf(const Eigen::Vector2d& point) const {
  // Signed distance from point to polygon boundary
  // Negative = inside, Positive = outside
  if (vertices_.size() < 3) return std::numeric_limits<double>::infinity();

  int n = static_cast<int>(vertices_.size());
  double min_dist = std::numeric_limits<double>::infinity();
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d& vi = vertices_[i];
    const Eigen::Vector2d& vj = vertices_[j];

    // Ray casting for inside/outside
    if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
        (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x())) {
      inside = !inside;
    }

    // Distance to edge segment
    Eigen::Vector2d edge = vj - vi;
    double t = (point - vi).dot(edge) / edge.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));
    Eigen::Vector2d closest = vi + t * edge;
    double dist = (point - closest).norm();
    if (dist < min_dist) min_dist = dist;
  }

  return inside ? -min_dist : min_dist;
}

std::vector<Eigen::Vector2d> FootprintModel::rotatedVertices(double yaw) const {
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  std::vector<Eigen::Vector2d> result;
  result.reserve(vertices_.size());
  for (const auto& v : vertices_) {
    result.emplace_back(c * v.x() - s * v.y(), s * v.x() + c * v.y());
  }
  return result;
}

}  // namespace esv_planner
