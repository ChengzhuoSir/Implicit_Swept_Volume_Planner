#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <unordered_map>
#include <vector>

namespace esv_planner {

class CollisionChecker {
public:
  CollisionChecker();

  void init(const GridMap& map, const FootprintModel& footprint, int num_yaw_bins);

  // Generate robot kernel templates for all discretized yaw angles
  void generateRobotKernels();

  // Check if a given SE2 state is collision-free
  bool isFree(const SE2State& state) const;

  // SafeYaw: find collision-free yaw angles at a given (x, y)
  std::vector<int> safeYawIndices(double wx, double wy) const;

  // Convenience: count of safe yaw bins (uses cache)
  int safeYawCount(double wx, double wy) const;

  // Convenience: whether any safe yaw exists (uses cache)
  bool hasAnySafeYaw(double wx, double wy) const;

  // Check if a specific yaw bin is safe at (wx, wy)
  bool isYawSafe(double wx, double wy, int yaw_bin) const;

  // Yaw discretization helpers
  double yawFromBin(int bin) const;
  int binFromYaw(double yaw) const;
  int numYawBins() const { return num_yaw_bins_; }
  const FootprintModel& footprint() const { return *footprint_; }

private:
  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
  int num_yaw_bins_ = 18;

  // Robot kernel: for each yaw bin, a list of relative grid offsets that the robot occupies
  std::vector<std::vector<GridIndex>> kernels_;

  // Cache: grid linear index -> safe yaw bin indices
  mutable std::unordered_map<int, std::vector<int>> safe_yaw_cache_;
};

}  // namespace esv_planner
