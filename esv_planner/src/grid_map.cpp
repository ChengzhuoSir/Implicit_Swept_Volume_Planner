#include "esv_planner/grid_map.h"
#include <queue>
#include <cmath>
#include <algorithm>

namespace esv_planner {

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
  return val > 50 || val < 0;
}

double GridMap::getEsdf(double wx, double wy) const {
  GridIndex gi = worldToGrid(wx, wy);
  if (!isInside(gi.x, gi.y)) return -1.0;
  return esdf_[gi.y * width_ + gi.x];
}

void GridMap::inflateByRadius(double radius) {
  int r_cells = static_cast<int>(std::ceil(radius / resolution_));
  inflated_ = occupancy_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int8_t val = occupancy_[y * width_ + x];
      if (val > 50 || val < 0) {
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
}

void GridMap::computeEsdf() {
  // Brushfire-based ESDF computation
  int n = width_ * height_;
  esdf_.assign(n, kInf);

  struct Cell {
    int x, y;
    double dist;
    bool operator>(const Cell& o) const { return dist > o.dist; }
  };

  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;

  // Seed with obstacle cells
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int idx = y * width_ + x;
      int8_t val = occupancy_[idx];
      if (val > 50 || val < 0) {
        esdf_[idx] = 0.0;
        pq.push({x, y, 0.0});
      }
    }
  }

  const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  const double dd8[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

  while (!pq.empty()) {
    Cell c = pq.top();
    pq.pop();

    int idx = c.y * width_ + c.x;
    if (c.dist > esdf_[idx]) continue;

    for (int i = 0; i < 8; ++i) {
      int nx = c.x + dx8[i];
      int ny = c.y + dy8[i];
      if (!isInside(nx, ny)) continue;

      double nd = c.dist + dd8[i] * resolution_;
      int nidx = ny * width_ + nx;
      if (nd < esdf_[nidx]) {
        esdf_[nidx] = nd;
        pq.push({nx, ny, nd});
      }
    }
  }

  // Make signed: negative inside obstacles
  for (int i = 0; i < n; ++i) {
    int8_t val = occupancy_[i];
    if (val > 50 || val < 0) {
      esdf_[i] = -esdf_[i];
    }
  }
}

}  // namespace esv_planner
