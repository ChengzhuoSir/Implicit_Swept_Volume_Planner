#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeNarrowThroatMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 120;
  map->info.height = 80;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 100);

  auto carveRect = [&](int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        map->data[static_cast<size_t>(y) * map->info.width + x] = 0;
      }
    }
  };

  // Narrow throat on the left: too tight for the T footprint at the center.
  carveRect(4, 35, 41, 44);
  // Wider chamber begins 5 cm to the right, so structured translation search can escape today.
  carveRect(42, 26, 110, 53);
  return map;
}

FootprintModel makeTFootprint() {
  FootprintModel footprint;
  footprint.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);
  return footprint;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_topology_body_push_no_search",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeNarrowThroatMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner planner;
  planner.init(map, checker, 100, 8, 4, footprint.inscribedRadius());

  TopoWaypoint wp(2.05, 1.975);
  wp.yaw = 0.0;
  wp.has_yaw = true;
  const bool was_free = checker.isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  const bool ok = planner.repairWaypointBodyFrame(wp, footprint.inscribedRadius());
  const bool now_free = checker.isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));

  std::cout << "[test] was_free=" << was_free
            << " ok=" << ok
            << " now_free=" << now_free
            << " repaired=(" << wp.pos.x() << ", " << wp.pos.y()
            << ", " << wp.yaw << ")\n";

  if (was_free) {
    std::cerr << "[test] FAIL: seed waypoint must start in collision\n";
    return 1;
  }
  if (ok || now_free) {
    std::cerr << "[test] FAIL: strict body-frame push should prune this local minimum instead of searching out\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
