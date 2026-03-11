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

nav_msgs::OccupancyGrid::Ptr makeSyntheticMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 100;
  map->info.height = 100;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  auto fillRect = [&](int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        map->data[static_cast<size_t>(y) * map->info.width + x] = 100;
      }
    }
  };

  fillRect(38, 20, 60, 74);
  fillRect(20, 20, 38, 38);
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
  ros::init(argc, argv, "test_topology_body_push", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSyntheticMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner planner;
  planner.init(map, checker, 100, 8, 4, footprint.inscribedRadius());

  TopoWaypoint wp(0.75, 1.40);
  wp.yaw = 0.0;
  wp.has_yaw = true;
  const Eigen::Vector2d original = wp.pos;
  const bool was_free = checker.isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  const double before = map.getEsdf(wp.pos.x(), wp.pos.y());
  const bool ok = planner.repairWaypointBodyFrame(wp, footprint.inscribedRadius());
  const bool now_free = checker.isFree(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  const double after = map.getEsdf(wp.pos.x(), wp.pos.y());

  std::cout << "[test] was_free=" << was_free
            << " ok=" << ok
            << " now_free=" << now_free
            << " before=" << before
            << " after=" << after
            << " moved=" << (wp.pos - original).norm()
            << " pos=(" << wp.pos.x() << "," << wp.pos.y() << ")"
            << " yaw=" << wp.yaw << "\n";

  if (was_free) {
    std::cerr << "[test] FAIL: expected seed waypoint to start in collision\n";
    return 1;
  }
  if (!ok || !now_free) {
    std::cerr << "[test] FAIL: expected body-frame push to recover waypoint\n";
    return 1;
  }
  if (!(after > before + 1e-6)) {
    std::cerr << "[test] FAIL: expected body-frame push to increase local clearance\n";
    return 1;
  }
  if ((wp.pos - original).norm() < 1e-3) {
    std::cerr << "[test] FAIL: expected waypoint to move\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
