#pragma once

#include <esv_planner/common.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace esv_planner {

struct GeometrySegment {
  Eigen::Vector2d a = Eigen::Vector2d::Zero();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
};

namespace geometry_map_detail {

inline bool isOccupiedValue(int8_t val) {
  return val > 50 || val < 0;
}

inline bool isInside(int x, int y, int width, int height) {
  return x >= 0 && x < width && y >= 0 && y < height;
}

inline Eigen::Vector2d projectToSegment(const Eigen::Vector2d& point,
                                        const GeometrySegment& segment) {
  const Eigen::Vector2d edge = segment.b - segment.a;
  const double denom = edge.squaredNorm();
  double t = 0.0;
  if (denom > 1e-12) {
    t = (point - segment.a).dot(edge) / denom;
    t = std::max(0.0, std::min(1.0, t));
  }
  return segment.a + t * edge;
}

inline double segmentDistanceSquared(const Eigen::Vector2d& point,
                                     const GeometrySegment& segment) {
  return (point - projectToSegment(point, segment)).squaredNorm();
}

inline Eigen::Vector2d segmentMidpoint(const GeometrySegment& segment) {
  return 0.5 * (segment.a + segment.b);
}

}  // namespace geometry_map_detail

class GeometryMap {
public:
  GeometryMap() = default;

  void rebuild(int width, int height, double resolution,
               double origin_x, double origin_y,
               const std::vector<int8_t>& occupancy) {
    segments_.clear();
    boundary_points_.clear();
    boundary_cell_centers_.clear();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    if (width_ <= 0 || height_ <= 0 || occupancy.empty()) return;

    auto cellEdgeSegment = [&](int gx, int gy, int edge_index) {
      const double x0 = origin_x_ + static_cast<double>(gx) * resolution_;
      const double y0 = origin_y_ + static_cast<double>(gy) * resolution_;
      const double x1 = x0 + resolution_;
      const double y1 = y0 + resolution_;
      GeometrySegment segment;
      switch (edge_index) {
        case 0:
          segment.a = Eigen::Vector2d(x0, y0);
          segment.b = Eigen::Vector2d(x0, y1);
          break;
        case 1:
          segment.a = Eigen::Vector2d(x1, y0);
          segment.b = Eigen::Vector2d(x1, y1);
          break;
        case 2:
          segment.a = Eigen::Vector2d(x0, y0);
          segment.b = Eigen::Vector2d(x1, y0);
          break;
        default:
          segment.a = Eigen::Vector2d(x0, y1);
          segment.b = Eigen::Vector2d(x1, y1);
          break;
      }
      return segment;
    };

    auto emitBoundaryEdge = [&](int gx, int gy, int edge_index) {
      const GeometrySegment segment = cellEdgeSegment(gx, gy, edge_index);
      segments_.push_back(segment);
      boundary_points_.push_back(
          geometry_map_detail::segmentMidpoint(segment));
    };

    for (int gy = 0; gy < height_; ++gy) {
      for (int gx = 0; gx < width_; ++gx) {
        const int idx = gy * width_ + gx;
        if (!geometry_map_detail::isOccupiedValue(occupancy[idx])) continue;
        const int left_x = gx - 1;
        const int right_x = gx + 1;
        const int down_y = gy - 1;
        const int up_y = gy + 1;

        if (!geometry_map_detail::isInside(left_x, gy, width_, height_) ||
            !geometry_map_detail::isOccupiedValue(occupancy[gy * width_ + left_x])) {
          emitBoundaryEdge(gx, gy, 0);
        }
        if (!geometry_map_detail::isInside(right_x, gy, width_, height_) ||
            !geometry_map_detail::isOccupiedValue(occupancy[gy * width_ + right_x])) {
          emitBoundaryEdge(gx, gy, 1);
        }
        if (!geometry_map_detail::isInside(gx, down_y, width_, height_) ||
            !geometry_map_detail::isOccupiedValue(occupancy[down_y * width_ + gx])) {
          emitBoundaryEdge(gx, gy, 2);
        }
        if (!geometry_map_detail::isInside(gx, up_y, width_, height_) ||
            !geometry_map_detail::isOccupiedValue(occupancy[up_y * width_ + gx])) {
          emitBoundaryEdge(gx, gy, 3);
        }

        bool boundary_cell = false;
        for (int dy = -1; dy <= 1 && !boundary_cell; ++dy) {
          for (int dx = -1; dx <= 1 && !boundary_cell; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int nx = gx + dx;
            const int ny = gy + dy;
            if (!geometry_map_detail::isInside(nx, ny, width_, height_) ||
                !geometry_map_detail::isOccupiedValue(occupancy[ny * width_ + nx])) {
              boundary_cell = true;
            }
          }
        }

        if (boundary_cell) {
          const double cx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
          const double cy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
          boundary_cell_centers_.emplace_back(cx, cy);
        }
      }
    }
  }

  bool empty() const { return segments_.empty(); }

  const std::vector<GeometrySegment>& segments() const { return segments_; }

  const std::vector<Eigen::Vector2d>& boundaryPoints() const {
    return boundary_points_;
  }

  const std::vector<Eigen::Vector2d>& boundaryCellCenters() const {
    return boundary_cell_centers_;
  }

  Eigen::Vector2d nearestObstacle(const Eigen::Vector2d& query,
                                  double* out_distance = nullptr) const {
    if (segments_.empty()) {
      if (out_distance) *out_distance = std::numeric_limits<double>::infinity();
      return Eigen::Vector2d::Zero();
    }

    double best_dist2 = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_point = Eigen::Vector2d::Zero();
    for (const auto& segment : segments_) {
      const Eigen::Vector2d candidate =
          geometry_map_detail::projectToSegment(query, segment);
      const double dist2 = (candidate - query).squaredNorm();
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best_point = candidate;
      }
    }

    if (out_distance) *out_distance = std::sqrt(best_dist2);
    return best_point;
  }

  std::vector<GeometrySegment> queryLocalSegments(const Eigen::Vector2d& query,
                                                  double radius,
                                                  size_t max_segments) const {
    std::vector<std::pair<double, size_t>> ranked;
    const double radius2 = radius * radius;
    ranked.reserve(segments_.size());
    for (size_t i = 0; i < segments_.size(); ++i) {
      const double dist2 =
          geometry_map_detail::segmentDistanceSquared(query, segments_[i]);
      if (dist2 <= radius2) {
        ranked.emplace_back(dist2, i);
      }
    }

    std::sort(ranked.begin(), ranked.end(),
              [&](const std::pair<double, size_t>& lhs,
                  const std::pair<double, size_t>& rhs) {
                if (std::abs(lhs.first - rhs.first) > 1e-12) {
                  return lhs.first < rhs.first;
                }
                const GeometrySegment& a = segments_[lhs.second];
                const GeometrySegment& b = segments_[rhs.second];
                if (std::abs(a.a.x() - b.a.x()) > 1e-12) return a.a.x() < b.a.x();
                if (std::abs(a.a.y() - b.a.y()) > 1e-12) return a.a.y() < b.a.y();
                if (std::abs(a.b.x() - b.b.x()) > 1e-12) return a.b.x() < b.b.x();
                return a.b.y() < b.b.y();
              });

    const size_t keep = std::min(max_segments, ranked.size());
    std::vector<GeometrySegment> result;
    result.reserve(keep);
    for (size_t i = 0; i < keep; ++i) {
      result.push_back(segments_[ranked[i].second]);
    }
    return result;
  }

private:
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<GeometrySegment> segments_;
  std::vector<Eigen::Vector2d> boundary_points_;
  std::vector<Eigen::Vector2d> boundary_cell_centers_;
};

}  // namespace esv_planner
