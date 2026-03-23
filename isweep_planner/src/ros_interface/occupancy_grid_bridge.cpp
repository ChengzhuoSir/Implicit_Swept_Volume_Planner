#include "isweep_planner/ros_interface/occupancy_grid_bridge.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {

constexpr int kOccupiedThreshold = 50;

}  // namespace

bool OccupancyGridBridge::Populate(const nav_msgs::OccupancyGrid& map, PCSmapManager* manager) {
  if (manager == nullptr || map.info.width == 0 || map.info.height == 0) {
    return false;
  }

  const double source_resolution = map.info.resolution;
  const double target_resolution =
      manager->occupancy_resolution > 0.0 ? manager->occupancy_resolution : source_resolution;
  const int target_width = std::max(
      1, static_cast<int>(std::ceil(map.info.width * source_resolution / target_resolution)));
  const int target_height = std::max(
      1, static_cast<int>(std::ceil(map.info.height * source_resolution / target_resolution)));

  manager->obs_forblender.clear();
  manager->aabb_points.clear();
  manager->boundary_xyzmin =
      Eigen::Vector3d(map.info.origin.position.x, map.info.origin.position.y, -0.5 * target_resolution);
  manager->boundary_xyzmax =
      Eigen::Vector3d(map.info.origin.position.x + target_width * target_resolution,
                      map.info.origin.position.y + target_height * target_resolution,
                      0.5 * target_resolution);
  manager->occupancy_resolution = target_resolution;

  if (!manager->occupancy_map) {
    manager->occupancy_map.reset(new GridMap3D);
  } else {
    manager->occupancy_map->releaseMemory();
  }

  manager->occupancy_map->grid_resolution = target_resolution;
  manager->occupancy_map->debug_output = false;
  manager->occupancy_map->createGridMap(manager->boundary_xyzmin, manager->boundary_xyzmax);
  manager->occupancy_map->clearGridMap();

  for (int target_y = 0; target_y < target_height; ++target_y) {
    const int src_y_min = std::max(0, static_cast<int>(std::floor(target_y * target_resolution / source_resolution)));
    const int src_y_max = std::min(
        static_cast<int>(map.info.height) - 1,
        static_cast<int>(std::ceil((target_y + 1) * target_resolution / source_resolution)) - 1);
    for (int target_x = 0; target_x < target_width; ++target_x) {
      const int src_x_min = std::max(0, static_cast<int>(std::floor(target_x * target_resolution / source_resolution)));
      const int src_x_max = std::min(
          static_cast<int>(map.info.width) - 1,
          static_cast<int>(std::ceil((target_x + 1) * target_resolution / source_resolution)) - 1);

      bool occupied = false;
      for (int src_y = src_y_min; src_y <= src_y_max && !occupied; ++src_y) {
        for (int src_x = src_x_min; src_x <= src_x_max; ++src_x) {
          const size_t linear_index = static_cast<size_t>(src_y) * map.info.width + src_x;
          const int8_t value = map.data[linear_index];
          if (value < 0 || value >= kOccupiedThreshold) {
            occupied = true;
            break;
          }
        }
      }

      if (occupied) {
        const int addr = manager->occupancy_map->toAddr(target_x, target_y, 0);
        manager->occupancy_map->grid_map[addr] = 1.0;
      }
    }
  }

  manager->recieved_globalmap = true;
  return true;
}

}  // namespace isweep_planner
