#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/collision_checker.h>
#include <esv_planner/continuous_feasibility.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeOpenMap() {
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
  map->info.resolution = 0.05;
  map->info.width = 40;
  map->info.height = 40;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);
  map->data[static_cast<size_t>(20) * map->info.width + 20] = 100;
  return map;
}

FootprintModel makeBoxFootprint() {
  FootprintModel footprint;
  footprint.setPolygon({
      {0.2, 0.2}, {0.2, -0.2}, {-0.2, -0.2}, {-0.2, 0.2}});
  footprint.setInscribedRadius(0.1);
  return footprint;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_grid_continuous_feasibility_default_backend",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeOpenMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));

  FootprintModel footprint = makeBoxFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  GridContinuousFeasibilityChecker feasibility(map, checker, 0.1);
  const auto mode = feasibility.backendMode();
  std::cout << "[test] backend_mode=" << static_cast<int>(mode) << "\n";
  if (mode != UnifiedContinuousEvaluator::BackendMode::GeometryBodyFrame) {
    std::cerr << "[test] FAIL: default feasibility backend should use geometry body-frame mode\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
