#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <esv_planner/grid_map.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.1;
  map->info.width = 20;
  map->info.height = 20;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  for (int y = 8; y <= 11; ++y) {
    for (int x = 8; x <= 11; ++x) {
      map->data[static_cast<size_t>(y) * map->info.width + x] = 100;
    }
  }
  return map;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_grid_map_esdf", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));

  const double free_esdf = map.getEsdf(0.25, 0.25);
  const double inside_esdf = map.getEsdf(0.95, 0.95);

  std::cout << "[test] free_esdf=" << free_esdf
            << " inside_esdf=" << inside_esdf << "\n";

  if (!(free_esdf > 0.0)) {
    std::cerr << "[test] FAIL: expected free-space ESDF to be positive\n";
    return 1;
  }
  if (!(inside_esdf < 0.0)) {
    std::cerr << "[test] FAIL: expected obstacle-interior ESDF to be negative\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
