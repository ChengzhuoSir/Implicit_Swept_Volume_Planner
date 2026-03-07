#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>

#include <iostream>
#include <vector>

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
  ros::init(argc, argv, "test_motion_sequence", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSyntheticMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.10, 5);

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

  std::vector<TopoPath> topo_paths{path};
  planner.shortenPaths(topo_paths);
  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected topology shortcut to produce a path\n";
    return 1;
  }
  path = topo_paths.front();

  std::vector<MotionSegment> segments = generator.generate(
      path, SE2State(0.70, 1.40, 0.0), SE2State(4.40, 1.40, 0.0));

  int high_count = 0;
  bool has_replacement_segment = false;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      ++high_count;
    }
    if (seg.waypoints.size() >= 4) {
      has_replacement_segment = true;
    }
  }

  std::cout << "[test] segments=" << segments.size()
            << " high_count=" << high_count
            << " replacement_like=" << has_replacement_segment << "\n";
  for (size_t i = 0; i < segments.size(); ++i) {
    std::cout << "[test] seg[" << i << "] risk="
              << (segments[i].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
              << " size=" << segments[i].waypoints.size() << "\n";
    if (segments[i].risk == RiskLevel::HIGH) {
      const size_t limit = std::min<size_t>(segments[i].waypoints.size(), 6);
      for (size_t j = 0; j < limit; ++j) {
        const auto& st = segments[i].waypoints[j];
        std::cout << "[test]   high[" << j << "]=(" << st.x << "," << st.y
                  << "," << st.yaw << ")\n";
      }
    }
  }

  if (!has_replacement_segment) {
    std::cerr << "[test] FAIL: expected SegAdjust-like expansion of a recoverable segment\n";
    return 1;
  }

  if (high_count != 0) {
    std::cerr << "[test] FAIL: expected recoverable synthetic case to remain low risk\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
