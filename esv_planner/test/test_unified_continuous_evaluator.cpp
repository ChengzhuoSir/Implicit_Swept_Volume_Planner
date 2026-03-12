#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/unified_continuous_evaluator.h>

#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

namespace {

nav_msgs::OccupancyGrid::Ptr makeMap() {
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
  map->info.resolution = 0.1;
  map->info.width = 40;
  map->info.height = 40;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);
  map->data[static_cast<size_t>(20) * map->info.width + 20] = 100;
  return map;
}

}  // namespace

int main() {
  using namespace esv_planner;

  GridMap map;
  auto occ = makeMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));

  FootprintModel footprint;
  footprint.setPolygon({Eigen::Vector2d(-0.2, -0.1), Eigen::Vector2d(0.2, -0.1),
                        Eigen::Vector2d(0.2, 0.1), Eigen::Vector2d(-0.2, 0.1)});

  UnifiedContinuousEvaluator evaluator;
  evaluator.initGridEsdf(map, footprint);

  const SE2State safe_state(0.8, 0.8, 0.0);
  const double state_clearance = evaluator.evaluate(safe_state);
  if (!(state_clearance > 0.0)) {
    std::cerr << "expected_positive_state_clearance got=" << state_clearance
              << std::endl;
    return 1;
  }

  Trajectory traj;
  PolyPiece pp;
  pp.duration = 0.8;
  pp.coeffs(0, 0) = safe_state.x;
  pp.coeffs(1, 0) = safe_state.y;
  traj.pos_pieces.push_back(pp);

  YawPolyPiece yp;
  yp.duration = 0.8;
  yp.coeffs(0, 0) = safe_state.yaw;
  traj.yaw_pieces.push_back(yp);

  const double traj_clearance = evaluator.evaluateTrajectory(traj);
  if (std::abs(traj_clearance - state_clearance) > 1e-6) {
    std::cerr << "trajectory_state_mismatch state=" << state_clearance
              << " traj=" << traj_clearance << std::endl;
    return 1;
  }

  Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
  double grad_yaw = 0.0;
  evaluator.gradient(safe_state, grad_pos, grad_yaw);
  if (!std::isfinite(grad_pos.x()) || !std::isfinite(grad_pos.y()) ||
      !std::isfinite(grad_yaw)) {
    std::cerr << "non_finite_gradient" << std::endl;
    return 1;
  }

  std::cout << "state_clearance=" << state_clearance
            << " traj_clearance=" << traj_clearance << std::endl;
  return 0;
}
