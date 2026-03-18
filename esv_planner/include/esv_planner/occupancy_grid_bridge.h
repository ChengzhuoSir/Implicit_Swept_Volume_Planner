#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <map_manager/PCSmap_manager.h>

namespace esv_planner {

class OccupancyGridBridge {
 public:
  static bool Populate(const nav_msgs::OccupancyGrid& map, PCSmapManager* manager);
};

}  // namespace esv_planner
