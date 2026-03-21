#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/env/collision_checker.h"
#include "esv_planner/env/footprint_model.h"
#include "esv_planner/env/grid_map.h"
#include "esv_planner/optimization/midend/se2_sequence_generator.h"
#include "esv_planner/global_search/topology_planner.h"

namespace esv_planner {
namespace {

nav_msgs::OccupancyGrid::Ptr MakeSyntheticMap() {
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
  map->info.resolution = 0.5;
  map->info.width = 40;
  map->info.height = 24;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->data.assign(map->info.width * map->info.height, 0);

  const auto set_occ = [&map](int x, int y) {
    if (x < 0 || y < 0 ||
        x >= static_cast<int>(map->info.width) ||
        y >= static_cast<int>(map->info.height)) {
      return;
    }
    map->data[static_cast<size_t>(y) * map->info.width + static_cast<size_t>(x)] = 100;
  };

  for (int x = 0; x < static_cast<int>(map->info.width); ++x) {
    set_occ(x, 0);
    set_occ(x, static_cast<int>(map->info.height) - 1);
  }
  for (int y = 0; y < static_cast<int>(map->info.height); ++y) {
    set_occ(0, y);
    set_occ(static_cast<int>(map->info.width) - 1, y);
  }

  for (int y = 1; y < static_cast<int>(map->info.height) - 1; ++y) {
    if (y == 6 || y == 17) {
      continue;
    }
    set_occ(18, y);
    set_occ(21, y);
  }

  for (int x = 8; x <= 31; ++x) {
    if (x >= 17 && x <= 22) {
      continue;
    }
    set_occ(x, 11);
    set_occ(x, 12);
  }

  return map;
}

FootprintModel MakeTestFootprint() {
  FootprintModel footprint;
  std::vector<Eigen::Vector2d> polygon;
  polygon.push_back(Eigen::Vector2d(0.20, 0.25));
  polygon.push_back(Eigen::Vector2d(0.20, -0.25));
  polygon.push_back(Eigen::Vector2d(-0.10, -0.25));
  polygon.push_back(Eigen::Vector2d(-0.10, -0.09));
  polygon.push_back(Eigen::Vector2d(-0.30, -0.09));
  polygon.push_back(Eigen::Vector2d(-0.30, 0.09));
  polygon.push_back(Eigen::Vector2d(-0.10, 0.09));
  polygon.push_back(Eigen::Vector2d(-0.10, 0.25));
  footprint.setPolygon(polygon);
  footprint.setInscribedRadius(0.08);
  return footprint;
}

TEST(CoarseToFineStagesTest, TopologyPlannerFindsMultipleCandidates) {
  GridMap map;
  map.fromOccupancyGrid(MakeSyntheticMap());
  FootprintModel footprint = MakeTestFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner planner;
  planner.init(map, checker, 220, 12, 4, footprint.inscribedRadius());
  planner.buildRoadmap(Eigen::Vector2d(3.0, 4.0), Eigen::Vector2d(16.5, 8.5));

  std::vector<TopoPath> paths = planner.searchPaths();
  planner.shortenPaths(paths);

  ASSERT_GE(paths.size(), 2U);
  EXPECT_GE(paths[0].waypoints.size(), 2U);
  EXPECT_GE(paths[1].waypoints.size(), 2U);
}

TEST(CoarseToFineStagesTest, MotionSequenceMarksHighAndLowRiskSegments) {
  GridMap map;
  map.fromOccupancyGrid(MakeSyntheticMap());
  FootprintModel footprint = MakeTestFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner planner;
  planner.init(map, checker, 220, 12, 4, footprint.inscribedRadius());
  planner.buildRoadmap(Eigen::Vector2d(3.0, 4.0), Eigen::Vector2d(16.5, 8.5));
  std::vector<TopoPath> paths = planner.searchPaths();
  planner.shortenPaths(paths);
  ASSERT_FALSE(paths.empty());

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.25, 5);
  const SE2State start(3.0, 4.0, 0.0);
  const SE2State goal(16.5, 8.5, 0.0);
  const std::vector<MotionSegment> segments = generator.generate(paths.front(), start, goal);

  ASSERT_FALSE(segments.empty());
  bool has_high = false;
  bool has_low = false;
  for (const MotionSegment& segment : segments) {
    has_high = has_high || segment.risk == RiskLevel::HIGH;
    has_low = has_low || segment.risk == RiskLevel::LOW;
  }
  EXPECT_TRUE(has_high);
  EXPECT_TRUE(has_low);
}

}  // namespace
}  // namespace esv_planner
