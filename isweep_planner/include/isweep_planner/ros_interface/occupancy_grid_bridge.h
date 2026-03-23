#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <isweep_planner/env/map_manager/PCSmap_manager.h>

namespace isweep_planner {

class OccupancyGridBridge {
 public:
  static bool Populate(const nav_msgs::OccupancyGrid& map, PCSmapManager* manager);
};

}  // namespace isweep_planner
