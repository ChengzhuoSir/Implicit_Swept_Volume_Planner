#include "esv_planner/grid_map.h"
#include <queue>
#include <cmath>
#include <algorithm>

namespace esv_planner {

namespace {

bool isOccupiedValue(int8_t val) {
  return val > 50 || val < 0;
}

std::vector<double> computeDistanceField(int width,
                                         int height,
                                         double resolution,
                                         const std::vector<int8_t>& grid,
                                         bool source_is_occupied) {
  const int n = width * height;
  std::vector<double> dist(n, kInf);

  struct Cell {
    int x, y;
    double dist;
    bool operator>(const Cell& o) const { return dist > o.dist; }
  };

  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int idx = y * width + x;
      const bool occupied = isOccupiedValue(grid[idx]);
      if (occupied == source_is_occupied) {
        dist[idx] = 0.0;
        pq.push({x, y, 0.0});
      }
    }
  }

  const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  const double dd8[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

  auto isInside = [width, height](int gx, int gy) {
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
  };

  while (!pq.empty()) {
    Cell c = pq.top();
    pq.pop();

    const int idx = c.y * width + c.x;
    if (c.dist > dist[idx]) continue;

    for (int i = 0; i < 8; ++i) {
      const int nx = c.x + dx8[i];
      const int ny = c.y + dy8[i];
      if (!isInside(nx, ny)) continue;

      const double nd = c.dist + dd8[i] * resolution;
      const int nidx = ny * width + nx;
      if (nd < dist[nidx]) {
        dist[nidx] = nd;
        pq.push({nx, ny, nd});
      }
    }
  }

  return dist;
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
