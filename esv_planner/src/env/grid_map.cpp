#include "esv_planner/env/grid_map.h"
#include <cmath>
#include <algorithm>

namespace esv_planner {

namespace {

bool isOccupiedValue(int8_t val) {
  return val > 50 || val < 0;
}

// Felzenszwalb-Huttenlocher 1D squared-distance transform
void distanceTransform1D(const double* f, double* d, int* v, double* z, int n) {
  int k = 0;
  v[0] = 0;
  z[0] = -1e18;
  z[1] = 1e18;
  for (int q = 1; q < n; ++q) {
    double s;
    while (true) {
      s = ((f[q] + (double)q * q) - (f[v[k]] + (double)v[k] * v[k])) /
          (2.0 * q - 2.0 * v[k]);
      if (s > z[k]) break;
      --k;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e18;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = (double)(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

std::vector<double> computeDistanceField(int width,
                                         int height,
                                         double resolution,
                                         const std::vector<int8_t>& grid,
                                         bool source_is_occupied) {
  const int n = width * height;
  const double INF = 1e18;

  // Initialize: source cells = 0, others = INF
  std::vector<double> field(n, INF);
  for (int i = 0; i < n; ++i) {
    if (isOccupiedValue(grid[i]) == source_is_occupied) {
      field[i] = 0.0;
    }
  }

  const int max_dim = std::max(width, height);
  std::vector<double> f(max_dim);
  std::vector<double> d(max_dim);
  std::vector<int> v(max_dim);
  std::vector<double> z(max_dim + 1);

  // Row-wise pass
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      f[x] = field[y * width + x];
    }
    distanceTransform1D(f.data(), d.data(), v.data(), z.data(), width);
    for (int x = 0; x < width; ++x) {
      field[y * width + x] = d[x];
    }
  }

  // Column-wise pass
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      f[y] = field[y * width + x];
    }
    distanceTransform1D(f.data(), d.data(), v.data(), z.data(), height);
    for (int y = 0; y < height; ++y) {
      field[y * width + x] = d[y];
    }
  }

  // Convert squared grid distance to meters
  for (int i = 0; i < n; ++i) {
    field[i] = std::sqrt(field[i]) * resolution;
  }

  return field;
}

}  // namespace

GridMap::GridMap() {}

void GridMap::fromOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  width_ = msg->info.width;
  height_ = msg->info.height;
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;

  occupancy_.assign(msg->data.begin(), msg->data.end());
  inflated_ = occupancy_;
  ready_ = true;

  computeEsdf();
  geometry_map_.rebuild(width_, height_, resolution_, origin_x_, origin_y_, inflated_);
}

GridIndex GridMap::worldToGrid(double wx, double wy) const {
  int gx = static_cast<int>((wx - origin_x_) / resolution_);
  int gy = static_cast<int>((wy - origin_y_) / resolution_);
  return GridIndex(gx, gy);
}

Eigen::Vector2d GridMap::gridToWorld(int gx, int gy) const {
  double wx = origin_x_ + (gx + 0.5) * resolution_;
  double wy = origin_y_ + (gy + 0.5) * resolution_;
  return Eigen::Vector2d(wx, wy);
}

bool GridMap::isInside(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

bool GridMap::isOccupied(int gx, int gy) const {
  if (!isInside(gx, gy)) return true;
  int8_t val = inflated_[gy * width_ + gx];
  return isOccupiedValue(val);
}

bool GridMap::isRawOccupied(int gx, int gy) const {
  if (!isInside(gx, gy)) return true;
  return isOccupiedValue(occupancy_[gy * width_ + gx]);
}

double GridMap::getEsdf(double wx, double wy) const {
  GridIndex gi = worldToGrid(wx, wy);
  if (!isInside(gi.x, gi.y)) return kInf;
  return esdf_[gi.y * width_ + gi.x];
}

double GridMap::getEsdfCell(int gx, int gy) const {
  if (!isInside(gx, gy)) return kInf;
  return esdf_[linearIndex(gx, gy)];
}

void GridMap::inflateByRadius(double radius) {
  int r_cells = static_cast<int>(std::ceil(radius / resolution_));
  inflated_ = occupancy_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int8_t val = occupancy_[y * width_ + x];
      if (isOccupiedValue(val)) {
        // Mark all cells within radius as occupied
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
          for (int dx = -r_cells; dx <= r_cells; ++dx) {
            if (dx * dx + dy * dy <= r_cells * r_cells) {
              int nx = x + dx, ny = y + dy;
              if (isInside(nx, ny)) {
                inflated_[ny * width_ + nx] = 100;
              }
            }
          }
        }
      }
    }
  }

  computeEsdf();
  geometry_map_.rebuild(width_, height_, resolution_, origin_x_, origin_y_, inflated_);
}

void GridMap::computeEsdf() {
  const int n = width_ * height_;
  esdf_.assign(n, kInf);
  if (n == 0) return;

  const std::vector<double> dist_to_occ =
      computeDistanceField(width_, height_, resolution_, inflated_, true);
  const std::vector<double> dist_to_free =
      computeDistanceField(width_, height_, resolution_, inflated_, false);

  for (int i = 0; i < n; ++i) {
    if (isOccupiedValue(inflated_[i])) {
      esdf_[i] = -dist_to_free[i];
    } else {
      esdf_[i] = dist_to_occ[i];
    }
  }
}

}  // namespace esv_planner
