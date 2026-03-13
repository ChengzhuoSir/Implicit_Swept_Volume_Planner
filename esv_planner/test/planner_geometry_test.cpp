#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/body_frame_sdf.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/grid_obstacle_source.h"

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

}  // namespace
}  // namespace esv_planner
