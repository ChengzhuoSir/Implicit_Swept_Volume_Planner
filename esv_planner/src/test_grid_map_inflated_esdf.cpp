#include <cmath>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/grid_map.h"

using namespace esv_planner;

int main() {
  nav_msgs::OccupancyGrid::Ptr grid(new nav_msgs::OccupancyGrid());
  grid->info.resolution = 0.05;
  grid->info.width = 50;
  grid->info.height = 50;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->data.assign(grid->info.width * grid->info.height, 0);

  // Single occupied cell at (20,20).
  const int ox = 20, oy = 20;
  grid->data[oy * grid->info.width + ox] = 100;

  GridMap map;
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(grid));

  // Query point at cell (23,20), distance ~0.15m from obstacle center.
  const double wx = (23 + 0.5) * grid->info.resolution;
  const double wy = (20 + 0.5) * grid->info.resolution;

  double before = map.getInflatedEsdf(wx, wy);
  if (!(before > 0.10)) {
    std::cerr << "[FAIL] Unexpected pre-inflation inflated ESDF: " << before << "\n";
    return 1;
  }

  // Inflate by 0.20m, the query point should now be within inflated obstacle band.
  map.inflateByRadius(0.20);
  double after = map.getInflatedEsdf(wx, wy);

  if (!(after <= 0.0)) {
    std::cerr << "[FAIL] Inflated ESDF not updated after inflation. after=" << after << "\n";
    return 1;
  }

  std::cout << "[PASS] inflated ESDF updated after inflateByRadius. before="
            << before << " after=" << after << "\n";
  return 0;
}
