#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include "esv_planner/se2_svsdf_solver.h"
#include "esv_planner/strict_frontend.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/grid_map.h"

namespace esv_planner {
namespace {

nav_msgs::OccupancyGrid::Ptr MakeOpenMap() {
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
  map->info.resolution = 0.5;
  map->info.width = 40;
  map->info.height = 20;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->data.assign(map->info.width * map->info.height, 0);
  return map;
}

FootprintModel MakeFootprint() {
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

TEST(StrictRuntimeComponentsTest, StrictSe2SolverReturnsNonEmptyTrajectory) {
  GridMap map;
  map.fromOccupancyGrid(MakeOpenMap());

  Se2SvsdfSolver solver;
  solver.InitializeForTesting(MakeFootprint());
  ASSERT_TRUE(solver.UpdateMap(*MakeOpenMap()));

  std::vector<SE2State> segment;
  segment.push_back(SE2State(1.0, 1.0, 0.0));
  segment.push_back(SE2State(3.0, 1.0, 0.0));
  segment.push_back(SE2State(5.0, 1.0, 0.0));

  Trajectory traj;
  EXPECT_TRUE(solver.Solve(segment, &traj));
  EXPECT_FALSE(traj.empty());
}

TEST(StrictRuntimeComponentsTest, StrictFrontendReturnsMultipleCandidates) {
  StrictFrontend frontend;
  frontend.InitializeForTesting(MakeFootprint());
  ASSERT_TRUE(frontend.UpdateMap(*MakeOpenMap()));

  const std::vector<TopoPath> candidates =
      frontend.Search(SE2State(1.0, 1.0, 0.0), SE2State(8.0, 6.0, 0.0));

  EXPECT_GE(candidates.size(), 2U);
}

nav_msgs::OccupancyGrid::Ptr MakeNarrowPassageMap() {
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
  map->info.resolution = 0.1;
  map->info.width = 100;
  map->info.height = 40;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->data.assign(map->info.width * map->info.height, 0);
  // Create walls leaving a 0.6m gap in the center (y=1.7 to y=2.3)
  for (unsigned int x = 30; x < 70; ++x) {
    for (unsigned int y = 0; y < 17; ++y) {
      map->data[y * map->info.width + x] = 100;
    }
    for (unsigned int y = 23; y < 40; ++y) {
      map->data[y * map->info.width + x] = 100;
    }
  }
  return map;
}

TEST(StrictRuntimeComponentsTest, StrictSe2SolverHandlesNarrowPassage) {
  Se2SvsdfSolver solver;
  solver.InitializeForTesting(MakeFootprint());
  ASSERT_TRUE(solver.UpdateMap(*MakeNarrowPassageMap()));

  std::vector<SE2State> segment;
  segment.push_back(SE2State(1.0, 2.0, 0.0));
  segment.push_back(SE2State(5.0, 2.0, 0.0));
  segment.push_back(SE2State(9.0, 2.0, 0.0));

  Trajectory traj;
  const bool ok = solver.Solve(segment, &traj);
  // The solver should either succeed or fail gracefully (no crash/NaN)
  if (ok) {
    EXPECT_FALSE(traj.empty());
    // Verify trajectory has finite values
    const double total = traj.totalDuration();
    EXPECT_TRUE(std::isfinite(total));
    EXPECT_GT(total, 0.0);
  }
}

}  // namespace
}  // namespace esv_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "strict_runtime_components_test");
  ros::start();
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
