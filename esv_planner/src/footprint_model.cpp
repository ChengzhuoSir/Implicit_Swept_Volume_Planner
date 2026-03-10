#include "esv_planner/footprint_model.h"
#include <cmath>
#include <limits>

namespace esv_planner {

namespace {

bool pointInPolygon(double px, double py,
                    const std::vector<Eigen::Vector2d>& verts) {
  bool inside = false;
  const int n = static_cast<int>(verts.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double yi = verts[i].y();
    const double yj = verts[j].y();
    const double xi = verts[i].x();
    const double xj = verts[j].x();
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

FootprintModel::FootprintModel() {}

void FootprintModel::setPolygon(const std::vector<Eigen::Vector2d>& vertices) {
  vertices_ = vertices;
  dense_sample_caches_.clear();
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

std::vector<Eigen::Vector2d> FootprintModel::sampleBoundary(double yaw, double step) const {
  std::vector<Eigen::Vector2d> rotated = rotatedVertices(yaw);
  std::vector<Eigen::Vector2d> samples;
  if (rotated.empty()) return samples;
  if (rotated.size() == 1) {
    samples.push_back(rotated.front());
    return samples;
  }

  const double spacing = std::max(1e-3, step);
  for (size_t i = 0; i < rotated.size(); ++i) {
    const Eigen::Vector2d& a = rotated[i];
    const Eigen::Vector2d& b = rotated[(i + 1) % rotated.size()];
    Eigen::Vector2d delta = b - a;
    double len = delta.norm();
    if (len < 1e-9) {
      samples.push_back(a);
      continue;
    }

    int n = std::max(2, static_cast<int>(std::ceil(len / spacing)) + 1);
    for (int s = 0; s < n - 1; ++s) {
      double r = static_cast<double>(s) / static_cast<double>(n - 1);
      samples.push_back(a + r * delta);
    }
  }
  samples.push_back(rotated.front());
  return samples;
}

const std::vector<Eigen::Vector2d>& FootprintModel::denseBodySamples(
    double boundary_step, double interior_step) const {
  const double boundary_spacing = std::max(1e-3, boundary_step);
  const double interior_spacing = std::max(1e-3, interior_step);

  for (const auto& cache : dense_sample_caches_) {
    if (std::abs(cache.boundary_step - boundary_spacing) < 1e-9 &&
        std::abs(cache.interior_step - interior_spacing) < 1e-9) {
      return cache.samples;
    }
  }

  DenseSampleCache cache;
  cache.boundary_step = boundary_spacing;
  cache.interior_step = interior_spacing;

  if (vertices_.empty()) {
    dense_sample_caches_.push_back(std::move(cache));
    return dense_sample_caches_.back().samples;
  }

  if (vertices_.size() == 1) {
    cache.samples.push_back(vertices_.front());
    dense_sample_caches_.push_back(std::move(cache));
    return dense_sample_caches_.back().samples;
  }

  cache.samples.reserve(vertices_.size() * 8);
  for (size_t i = 0; i < vertices_.size(); ++i) {
    const Eigen::Vector2d& a = vertices_[i];
    const Eigen::Vector2d& b = vertices_[(i + 1) % vertices_.size()];
    const Eigen::Vector2d delta = b - a;
    const double len = delta.norm();
    if (len < 1e-9) {
      cache.samples.push_back(a);
      continue;
    }

    const int n = std::max(2, static_cast<int>(std::ceil(len / boundary_spacing)) + 1);
    for (int s = 0; s < n - 1; ++s) {
      const double r = static_cast<double>(s) / static_cast<double>(n - 1);
      cache.samples.push_back(a + r * delta);
    }
  }
  cache.samples.push_back(vertices_.front());

  double bb_min_x = std::numeric_limits<double>::max();
  double bb_max_x = std::numeric_limits<double>::lowest();
  double bb_min_y = std::numeric_limits<double>::max();
  double bb_max_y = std::numeric_limits<double>::lowest();
  for (const auto& v : vertices_) {
    bb_min_x = std::min(bb_min_x, v.x());
    bb_max_x = std::max(bb_max_x, v.x());
    bb_min_y = std::min(bb_min_y, v.y());
    bb_max_y = std::max(bb_max_y, v.y());
  }

  for (double gx = bb_min_x + 0.5 * interior_spacing; gx <= bb_max_x; gx += interior_spacing) {
    for (double gy = bb_min_y + 0.5 * interior_spacing; gy <= bb_max_y; gy += interior_spacing) {
      if (pointInPolygon(gx, gy, vertices_)) {
        cache.samples.emplace_back(gx, gy);
      }
    }
  }

  cache.samples.emplace_back(0.0, 0.0);
  dense_sample_caches_.push_back(std::move(cache));
  return dense_sample_caches_.back().samples;
}

}  // namespace esv_planner
