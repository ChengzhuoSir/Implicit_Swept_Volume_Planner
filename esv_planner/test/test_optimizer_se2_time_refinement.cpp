#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/trajectory_optimizer.h>

#include <iostream>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeFreeMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 80;
  map->info.height = 80;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);
  return map;
}

std::vector<SE2State> makeSharpTurn() {
  return {
      SE2State(0.4, 0.4, 0.0),
      SE2State(0.8, 0.4, 0.0),
      SE2State(1.2, 0.4, 0.0),
      SE2State(1.2, 0.8, 1.57),
      SE2State(1.2, 1.2, 1.57),
  };
}

void maxDynamics(const Trajectory& traj,
                 double& max_vel,
                 double& max_acc,
                 double& max_yaw_rate) {
  max_vel = 0.0;
  max_acc = 0.0;
  max_yaw_rate = 0.0;
  const double total = traj.totalDuration();
  for (double t = 0.0; t <= total + 1e-9; t += 0.01) {
    const double tt = std::min(t, total);
    max_vel = std::max(max_vel, traj.sampleVelocity(tt).norm());
    max_acc = std::max(max_acc, traj.sampleAcceleration(tt).norm());

    double acc = 0.0;
    for (size_t i = 0; i < traj.yaw_pieces.size(); ++i) {
      const auto& yp = traj.yaw_pieces[i];
      if (tt <= acc + yp.duration || i + 1 == traj.yaw_pieces.size()) {
        max_yaw_rate = std::max(max_yaw_rate, std::abs(yp.velocity(tt - acc)));
        break;
      }
      acc += yp.duration;
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_optimizer_se2_time_refinement",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeFreeMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  FootprintModel footprint;
  footprint.setPolygon({{0.08, 0.08}, {0.08, -0.08}, {-0.08, -0.08}, {-0.08, 0.08}});
  footprint.setInscribedRadius(0.04);

  SvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  OptimizerParams params;
  params.max_iterations = 60;
  params.step_size = 0.005;
  params.max_vel = 0.35;
  params.max_acc = 0.6;
  params.max_yaw_rate = 0.8;
  params.lambda_smooth = 1.0;
  params.lambda_safety = 10.0;
  params.safety_margin = 0.1;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  const auto wps = makeSharpTurn();
  const double initial_time = 1.0;
  Trajectory traj = optimizer.optimizeSE2(wps, initial_time);

  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;
  maxDynamics(traj, max_vel, max_acc, max_yaw_rate);

  std::cout << "[test] duration=" << traj.totalDuration()
            << " max_vel=" << max_vel
            << " max_acc=" << max_acc
            << " max_yaw_rate=" << max_yaw_rate << "\n";

  if (traj.empty()) {
    std::cerr << "[test] FAIL: optimizeSE2 returned empty trajectory\n";
    return 1;
  }
  if (!(traj.totalDuration() > initial_time * 1.10)) {
    std::cerr << "[test] FAIL: optimizeSE2 did not expand total time under tight dynamics\n";
    return 1;
  }
  if (max_vel > params.max_vel * 1.10 ||
      max_acc > params.max_acc * 1.10 ||
      max_yaw_rate > params.max_yaw_rate * 1.10) {
    std::cerr << "[test] FAIL: optimizeSE2 still violates dynamics after time refinement\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
