#include "esv_planner/env/collision_checker.h"
#include <cmath>

namespace esv_planner {

CollisionChecker::CollisionChecker() {}

void CollisionChecker::init(const GridMap& map, const FootprintModel& footprint, int num_yaw_bins) {
  map_ = &map;
  footprint_ = &footprint;
  num_yaw_bins_ = num_yaw_bins;
  generateRobotKernels();
  safe_yaw_cache_.clear();
}

void CollisionChecker::generateRobotKernels() {
  kernels_.clear();
  kernels_.resize(num_yaw_bins_);

  double res = map_->resolution();
  double cr = footprint_->circumscribedRadius();
  int range = static_cast<int>(std::ceil(cr / res)) + 1;

  for (int bin = 0; bin < num_yaw_bins_; ++bin) {
    double yaw = yawFromBin(bin);
    auto rotated = footprint_->rotatedVertices(yaw);

    for (int dy = -range; dy <= range; ++dy) {
      for (int dx = -range; dx <= range; ++dx) {
        Eigen::Vector2d cell_center(dx * res, dy * res);
        // Check if cell center is inside the rotated polygon
        // Use ray casting
        int n = static_cast<int>(rotated.size());
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
          const Eigen::Vector2d& vi = rotated[i];
          const Eigen::Vector2d& vj = rotated[j];
          if (((vi.y() > cell_center.y()) != (vj.y() > cell_center.y())) &&
              (cell_center.x() < (vj.x() - vi.x()) * (cell_center.y() - vi.y()) /
               (vj.y() - vi.y()) + vi.x())) {
            inside = !inside;
          }
        }
        if (inside) {
          kernels_[bin].emplace_back(dx, dy);
        }
      }
    }
  }
}

bool CollisionChecker::isFree(const SE2State& state) const {
  int bin = binFromYaw(state.yaw);
  return isYawSafe(state.x, state.y, bin);
}

std::vector<int> CollisionChecker::safeYawIndices(double wx, double wy) const {
  GridIndex gi = map_->worldToGrid(wx, wy);
  if (!map_->isInside(gi.x, gi.y)) return {};
  int key = map_->linearIndex(gi.x, gi.y);
  auto it = safe_yaw_cache_.find(key);
  if (it != safe_yaw_cache_.end()) return it->second;
  std::vector<int> safe;
  for (int bin = 0; bin < num_yaw_bins_; ++bin) {
    if (isYawSafe(wx, wy, bin)) {
      safe.push_back(bin);
    }
  }
  safe_yaw_cache_[key] = safe;
  return safe;
}

int CollisionChecker::safeYawCount(double wx, double wy) const {
  return static_cast<int>(safeYawIndices(wx, wy).size());
}

bool CollisionChecker::hasAnySafeYaw(double wx, double wy) const {
  return !safeYawIndices(wx, wy).empty();
}

bool CollisionChecker::isYawSafe(double wx, double wy, int yaw_bin) const {
  if (yaw_bin < 0 || yaw_bin >= num_yaw_bins_) return false;

  GridIndex center = map_->worldToGrid(wx, wy);
  const auto& kernel = kernels_[yaw_bin];

  for (const auto& offset : kernel) {
    int gx = center.x + offset.x;
    int gy = center.y + offset.y;
    if (map_->isOccupied(gx, gy)) {
      return false;
    }
  }
  return true;
}

double CollisionChecker::yawFromBin(int bin) const {
  return -kPi + bin * kTwoPi / num_yaw_bins_;
}

int CollisionChecker::binFromYaw(double yaw) const {
  double normalized = normalizeAngle(yaw);
  int bin = static_cast<int>((normalized + kPi) / (kTwoPi / num_yaw_bins_));
  if (bin < 0) bin = 0;
  if (bin >= num_yaw_bins_) bin = num_yaw_bins_ - 1;
  return bin;
}

}  // namespace esv_planner
