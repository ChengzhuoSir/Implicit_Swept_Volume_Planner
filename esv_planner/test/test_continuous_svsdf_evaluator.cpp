#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/continuous_svsdf_evaluator.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeSparseHitMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 240;
  map->info.height = 80;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  const int gx = 75;   // x ~= 3.775
  const int gy = 20;   // y ~= 1.025
  map->data[static_cast<size_t>(gy) * map->info.width + gx] = 100;
  return map;
}

Trajectory makeFastStraightTrajectory() {
  Trajectory traj;
  PolyPiece p;
  p.duration = 0.1;
  p.coeffs.setZero();
  p.coeffs.col(0) = Eigen::Vector2d(0.5, 1.025);
  p.coeffs.col(1) = Eigen::Vector2d((7.0 - 0.5) / 0.1, 0.0);
  traj.pos_pieces.push_back(p);

  YawPolyPiece y;
  y.duration = 0.1;
  y.coeffs.setZero();
  y.coeffs(0, 0) = 0.0;
  traj.yaw_pieces.push_back(y);
  return traj;
}

FootprintModel makeSlimFootprint() {
  FootprintModel fp;
  fp.setPolygon({{0.10, 0.08}, {0.10, -0.08}, {-0.10, -0.08}, {-0.10, 0.08}});
  fp.setInscribedRadius(0.08);
  return fp;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_continuous_svsdf_evaluator",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSparseHitMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  const FootprintModel footprint = makeSlimFootprint();
  SvsdfEvaluator sampled;
  sampled.init(map, footprint);

  ContinuousSvsdfEvaluator continuous;
  continuous.init(map, footprint);

  const Trajectory traj = makeFastStraightTrajectory();
  const double sampled_clearance = sampled.evaluateTrajectory(traj, 0.05);
  const double continuous_clearance = continuous.evaluateTrajectory(traj);

  std::cout << "[test] sampled_clearance=" << sampled_clearance
            << " continuous_clearance=" << continuous_clearance << "\n";

  if (!(sampled_clearance > 0.0)) {
    std::cerr << "[test] FAIL: legacy sampled evaluator should miss this narrow hit case\n";
    return 1;
  }
  if (!(continuous_clearance < 0.0)) {
    std::cerr << "[test] FAIL: continuous evaluator should detect the missed collision\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
