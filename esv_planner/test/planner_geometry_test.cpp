#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/env/body_frame_sdf.h"
#include "esv_planner/env/footprint_model.h"
#include "esv_planner/env/grid_map.h"
#include "esv_planner/env/grid_obstacle_source.h"
#include "esv_planner/global_search/swept_astar.h"
#include "esv_planner/optimization/evaluator/svsdf_evaluator.h"

namespace esv_planner {
namespace {

TEST(BodyFrameSdfTest, ReportsSignedDistanceSigns) {
  BodyFrameSdf sdf;
  std::vector<Eigen::Vector2d> polygon;
  polygon.push_back(Eigen::Vector2d(-1.0, -1.0));
  polygon.push_back(Eigen::Vector2d(1.0, -1.0));
  polygon.push_back(Eigen::Vector2d(1.0, 1.0));
  polygon.push_back(Eigen::Vector2d(-1.0, 1.0));
  sdf.setPolygon(polygon);

  const BodyFrameQuery inside = sdf.query(Eigen::Vector2d(0.0, 0.0));
  const BodyFrameQuery outside = sdf.query(Eigen::Vector2d(2.0, 0.0));

  EXPECT_TRUE(inside.inside);
  EXPECT_LT(inside.signed_distance, 0.0);
  EXPECT_FALSE(outside.inside);
  EXPECT_GT(outside.signed_distance, 0.0);
}

TEST(GridObstacleSourceTest, QueriesOccupiedCellsInAabb) {
  nav_msgs::OccupancyGrid::Ptr msg(new nav_msgs::OccupancyGrid());
  msg->info.width = 5;
  msg->info.height = 5;
  msg->info.resolution = 1.0;
  msg->info.origin.position.x = 0.0;
  msg->info.origin.position.y = 0.0;
  msg->data.assign(25, 0);
  msg->data[1 * 5 + 1] = 100;
  msg->data[3 * 5 + 3] = 100;

  GridMap map;
  map.fromOccupancyGrid(msg);

  GridObstacleSource source;
  source.init(map);
  std::vector<Eigen::Vector2d> queried;
  source.queryAabb(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2.0, 2.0), &queried);

  ASSERT_EQ(queried.size(), 1U);
  EXPECT_NEAR(queried.front().x(), 1.5, 1e-6);
  EXPECT_NEAR(queried.front().y(), 1.5, 1e-6);
}

TEST(SvsdfEvaluatorTest, GridGradientUsesFieldSlopeInsideSingleCell) {
  nav_msgs::OccupancyGrid::Ptr msg(new nav_msgs::OccupancyGrid());
  msg->info.width = 5;
  msg->info.height = 5;
  msg->info.resolution = 1.0;
  msg->info.origin.position.x = 0.0;
  msg->info.origin.position.y = 0.0;
  msg->data.assign(25, 0);
  msg->data[2 * 5 + 3] = 100;

  GridMap map;
  map.fromOccupancyGrid(msg);

  FootprintModel footprint;
  std::vector<Eigen::Vector2d> point_footprint;
  point_footprint.push_back(Eigen::Vector2d(0.3, 0.3));
  footprint.setPolygon(point_footprint);

  SvsdfEvaluator evaluator;
  evaluator.initGridEsdf(map, footprint);

  const SE2State state(2.2, 2.2, 0.0);
  Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
  double grad_yaw = 0.0;
  evaluator.gradient(state, grad_pos, grad_yaw);

  EXPECT_LT(grad_pos.x(), -0.1);
  EXPECT_NEAR(grad_pos.y(), 0.0, 1e-6);
  EXPECT_GT(grad_yaw, 0.1);
}

TEST(SweptAstarTest, AvoidsBlockedCentersInsideLocalBounds) {
  nav_msgs::OccupancyGrid::Ptr msg(new nav_msgs::OccupancyGrid());
  msg->info.width = 100;
  msg->info.height = 60;
  msg->info.resolution = 0.1;
  msg->info.origin.position.x = 0.0;
  msg->info.origin.position.y = 0.0;
  msg->data.assign(msg->info.width * msg->info.height, 0);

  GridMap map;
  map.fromOccupancyGrid(msg);

  FootprintModel footprint;
  std::vector<Eigen::Vector2d> polygon;
  polygon.push_back(Eigen::Vector2d(0.05, 0.05));
  polygon.push_back(Eigen::Vector2d(0.05, -0.05));
  polygon.push_back(Eigen::Vector2d(-0.05, -0.05));
  polygon.push_back(Eigen::Vector2d(-0.05, 0.05));
  footprint.setPolygon(polygon);
  footprint.setInscribedRadius(0.04);

  CollisionChecker checker;
  checker.init(map, footprint, 16);

  SweptAstar astar;
  astar.init(map, checker, 0.05);

  SweptAstarSearchOptions options;
  options.blocked_centers.push_back(Eigen::Vector2d(4.0, 2.0));
  options.blocked_radius = 0.8;
  options.min_safe_yaw_count = 1;
  options.min_x = 0;
  options.min_y = 0;
  options.max_x = 90;
  options.max_y = 40;
  options.preferred_clearance = 0.15;
  options.clearance_penalty = 0.5;
  options.blocked_center_penalty = 1.0;

  const AstarSearchResult result =
      astar.search(Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(7.0, 2.0), options);

  ASSERT_TRUE(result.success);
  ASSERT_GT(result.path.size(), 2U);
  double max_lateral_offset = 0.0;
  for (const Eigen::Vector2d& point : result.path) {
    EXPECT_GE((point - options.blocked_centers.front()).norm(), 0.75);
    max_lateral_offset = std::max(max_lateral_offset, std::abs(point.y() - 2.0));
  }
  EXPECT_GT(max_lateral_offset, 0.3);
}

}  // namespace
}  // namespace esv_planner
