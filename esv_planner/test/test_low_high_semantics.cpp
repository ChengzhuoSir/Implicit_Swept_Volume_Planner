#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>
#include <esv_planner/continuous_feasibility.h>

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
  ros::init(argc, argv, "test_low_high_semantics", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSyntheticMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);
  GridContinuousFeasibilityChecker feasibility(map, checker, 0.10);

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

  const auto segments = generator.generate(
      topo_paths.front(), SE2State(0.70, 1.40, 0.0), SE2State(4.40, 1.40, 0.0));
  if (segments.empty()) {
    std::cerr << "[test] FAIL: expected sequence generator to produce segments\n";
    return 1;
  }

  const double required = feasibility.requiredClearance();
  bool saw_sub_margin_segment = false;
  bool saw_safe_low_segment = false;
  for (size_t i = 0; i < segments.size(); ++i) {
    const double clearance = feasibility.segmentClearance(segments[i].waypoints);
    const bool sub_margin = clearance + 1e-6 < required;
    std::cout << "[test] seg[" << i << "] risk="
              << (segments[i].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
              << " size=" << segments[i].waypoints.size()
              << " unified_clearance=" << clearance
              << " required=" << required << "\n";

    if (segments[i].risk == RiskLevel::LOW) {
      saw_safe_low_segment = true;
      if (sub_margin) {
        std::cerr << "[test] FAIL: LOW segment violated unified continuous margin rule\n";
        return 1;
      }
    }

    if (sub_margin) {
      saw_sub_margin_segment = true;
      if (segments[i].risk != RiskLevel::HIGH) {
        std::cerr << "[test] FAIL: sub-margin segment must be labeled HIGH under unified rule\n";
        return 1;
      }
    }
  }

  if (!saw_safe_low_segment) {
    std::cerr << "[test] FAIL: expected at least one LOW segment to remain feasible\n";
    return 1;
  }
  if (!saw_sub_margin_segment) {
    std::cerr << "[test] FAIL: expected synthetic case to contain a sub-margin segment\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
