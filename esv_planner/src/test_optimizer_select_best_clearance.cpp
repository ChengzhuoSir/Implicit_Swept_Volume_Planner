#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/trajectory_optimizer.h>

#include <cmath>
#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 120;
  map->info.height = 40;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  const int wall_y = 10;
  for (int x = 0; x < static_cast<int>(map->info.width); ++x) {
    map->data[static_cast<size_t>(wall_y) * map->info.width + x] = 100;
  }
  return map;
}

Trajectory makeStraightTrajectory(double y) {
  Trajectory traj;

  PolyPiece pp;
  pp.duration = 4.0;
  pp.coeffs(0, 0) = 0.50;
  pp.coeffs(0, 1) = 1.00;
  pp.coeffs(1, 0) = y;
  traj.pos_pieces.push_back(pp);

  YawPolyPiece yp;
  yp.duration = pp.duration;
  yp.coeffs(0, 0) = 0.0;
  traj.yaw_pieces.push_back(yp);

  return traj;
}

double minClearance(const SvsdfEvaluator& svsdf, const Trajectory& traj) {
  return svsdf.evaluateTrajectory(traj);
}

double sampleY(const Trajectory& traj) {
  const double t = 0.5 * traj.totalDuration();
  return traj.sample(t).y;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_optimizer_select_best_clearance",
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
  params.max_vel = 2.0;
  params.max_acc = 2.0;
  params.max_yaw_rate = 2.0;
  params.safety_margin = 0.1;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  const Trajectory penetrating = makeStraightTrajectory(0.50);
  const Trajectory collision_free = makeStraightTrajectory(0.25);
  const double penetrating_clearance = minClearance(svsdf, penetrating);
  const double safe_clearance = minClearance(svsdf, collision_free);

  std::cout << "[test] penetrating_clearance=" << penetrating_clearance
            << " safe_clearance=" << safe_clearance << "\n";

  if (!(penetrating_clearance < 0.0 && safe_clearance > 0.0)) {
    std::cerr << "[test] FAIL: invalid penetrating-vs-safe fixture\n";
    return 1;
  }

  Trajectory selected = optimizer.selectBest({penetrating, collision_free});
  if (selected.empty()) {
    std::cerr << "[test] FAIL: selectBest returned empty for mixed candidates\n";
    return 1;
  }
  if (minClearance(svsdf, selected) < 0.0) {
    std::cerr << "[test] FAIL: selectBest kept a penetrating candidate\n";
    return 1;
  }

  const Trajectory low_clearance = makeStraightTrajectory(0.35);
  const Trajectory high_clearance = makeStraightTrajectory(0.15);
  const double low_clear = minClearance(svsdf, low_clearance);
  const double high_clear = minClearance(svsdf, high_clearance);

  std::cout << "[test] low_clearance=" << low_clear
            << " high_clearance=" << high_clear << "\n";

  if (!(low_clear > 0.0 && high_clear > low_clear)) {
    std::cerr << "[test] FAIL: invalid low-vs-high clearance fixture\n";
    return 1;
  }

  selected = optimizer.selectBest({low_clearance, high_clearance});
  if (selected.empty()) {
    std::cerr << "[test] FAIL: selectBest returned empty for collision-free candidates\n";
    return 1;
  }
  if (std::abs(sampleY(selected) - sampleY(high_clearance)) > 1e-6) {
    std::cerr << "[test] FAIL: selectBest did not prefer the higher-clearance trajectory\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
