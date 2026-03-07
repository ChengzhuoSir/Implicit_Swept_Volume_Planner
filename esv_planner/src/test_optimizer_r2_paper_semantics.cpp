#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/trajectory_optimizer.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 60;
  map->info.height = 30;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  // Horizontal obstacle strip above the reference path.
  const int wall_y = 8;
  for (int x = 0; x < static_cast<int>(map->info.width); ++x) {
    map->data[static_cast<size_t>(wall_y) * map->info.width + x] = 100;
  }

  return map;
}

std::vector<SE2State> makeWaypoints() {
  std::vector<SE2State> wps;
  const double y = 0.30;
  for (int i = 0; i < 6; ++i) {
    wps.emplace_back(0.30 + 0.24 * static_cast<double>(i), y, 0.0);
  }
  return wps;
}

double maxLateralDeviation(const Trajectory& traj, double ref_y) {
  if (traj.empty()) return kInf;
  const double T = traj.totalDuration();
  const int samples = 20;
  double max_dev = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double t = T * static_cast<double>(i) / static_cast<double>(samples);
    const SE2State st = traj.sample(t);
    max_dev = std::max(max_dev, std::abs(st.y - ref_y));
  }
  return max_dev;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_optimizer_r2_paper_semantics",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  FootprintModel footprint;
  footprint.setPolygon({{0.05, 0.05}, {0.05, -0.05}, {-0.05, -0.05}, {-0.05, 0.05}});
  footprint.setInscribedRadius(0.025);

  SvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  OptimizerParams params;
  params.max_iterations = 80;
  params.step_size = 0.01;
  params.lambda_smooth = 1.0;
  params.lambda_pos_residual = 3.0;
  params.lambda_yaw_residual = 2.0;
  params.lambda_safety = 80.0;
  params.safety_margin = 0.20;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  const auto wps = makeWaypoints();
  const double ref_y = wps.front().y;
  Trajectory traj = optimizer.optimizeR2(wps, 2.0);
  const double max_dev = maxLateralDeviation(traj, ref_y);

  std::cout << "[test] max_lateral_dev=" << max_dev << "\n";

  if (traj.empty()) {
    std::cerr << "[test] FAIL: optimizeR2 returned empty trajectory\n";
    return 1;
  }

  if (max_dev > 0.03) {
    std::cerr << "[test] FAIL: R2 optimization deviated too far from the reference line\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
