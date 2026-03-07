#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

using namespace esv_planner;

namespace {

void dumpWaypoints(const std::vector<TopoWaypoint>& waypoints) {
  for (size_t i = 0; i < waypoints.size(); ++i) {
    std::cerr << "[test] wp[" << i << "]=(" << waypoints[i].pos.x()
              << "," << waypoints[i].pos.y() << ") yaw="
              << (waypoints[i].has_yaw ? waypoints[i].yaw : 999.0) << "\n";
  }
}

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

bool waypointStatesFree(const std::vector<TopoWaypoint>& waypoints,
                       const CollisionChecker& checker,
                       double start_yaw,
                       std::string* failure) {
  if (waypoints.size() < 2) return false;
  if (!checker.isFree(SE2State(waypoints.front().pos.x(), waypoints.front().pos.y(), start_yaw))) {
    if (failure != nullptr) {
      std::ostringstream oss;
      oss << "start pose collision pos=(" << waypoints.front().pos.x()
          << "," << waypoints.front().pos.y() << ") yaw=" << start_yaw;
      *failure = oss.str();
    }
    return false;
  }
  for (size_t i = 1; i < waypoints.size(); ++i) {
    Eigen::Vector2d delta = waypoints[i].pos - waypoints[i - 1].pos;
    double yaw = waypoints[i].has_yaw ? waypoints[i].yaw : std::atan2(delta.y(), delta.x());
    if (!checker.isFree(SE2State(waypoints[i].pos.x(), waypoints[i].pos.y(), yaw))) {
      if (failure != nullptr) {
        std::ostringstream oss;
        oss << "waypoint=" << i
            << " pos=(" << waypoints[i].pos.x() << "," << waypoints[i].pos.y() << ")"
            << " yaw=" << yaw;
        *failure = oss.str();
      }
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_topology_shortcut", ros::init_options::NoSigintHandler);
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

  TopoPath path;
  path.waypoints = {
      TopoWaypoint(0.70, 1.40),
      TopoWaypoint(1.30, 1.40),
      TopoWaypoint(1.30, 4.30),
      TopoWaypoint(4.40, 4.30),
      TopoWaypoint(4.40, 1.40)};
  path.computeLength();

  const size_t original_points = path.waypoints.size();
  const double original_length = path.length;
  std::vector<TopoPath> paths{path};
  planner.shortenPaths(paths);

  if (paths.empty()) {
    std::cerr << "[test] FAIL: expected one shortened path\n";
    return 1;
  }

  const TopoPath& shortened = paths.front();
  std::cout << "[test] original_points=" << original_points
            << " shortened_points=" << shortened.waypoints.size()
            << " length=" << shortened.length << "\n";

  if (shortened.waypoints.size() > 1 && !shortened.waypoints[1].has_yaw) {
    std::cerr << "[test] FAIL: expected shortened waypoint to retain yaw metadata\n";
    return 1;
  }

  std::string failure;
  if (!waypointStatesFree(shortened.waypoints, checker, 0.0, &failure)) {
    std::cerr << "[test] FAIL: expected geometry-aware shortcut to push waypoint states collision free\n";
    std::cerr << "[test] detail: " << failure << "\n";
    dumpWaypoints(shortened.waypoints);
    return 1;
  }

  if (!(shortened.length + 1e-6 < original_length)) {
    std::cerr << "[test] FAIL: expected shortcut to reduce path length\n";
    dumpWaypoints(shortened.waypoints);
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
