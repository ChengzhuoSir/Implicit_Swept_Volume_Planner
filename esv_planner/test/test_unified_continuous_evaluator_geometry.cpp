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
  for (int y = 18; y <= 21; ++y) {
    for (int x = 18; x <= 21; ++x) {
      map->data[static_cast<size_t>(y) * map->info.width + x] = 100;
    }
  }
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

  UnifiedContinuousEvaluator grid_eval;
  grid_eval.initGridEsdf(map, footprint);

  UnifiedContinuousEvaluator geom_eval;
  geom_eval.initGeometryBodyFrame(map.geometryMap(), footprint);

  if (geom_eval.mode() != UnifiedContinuousEvaluator::BackendMode::GeometryBodyFrame) {
    std::cerr << "wrong_backend_mode" << std::endl;
    return 1;
  }

  const SE2State safe_state(0.8, 0.8, 0.0);
  const SE2State colliding_state(1.95, 1.95, 0.0);

  const double safe_grid = grid_eval.evaluate(safe_state);
  const double safe_geom = geom_eval.evaluate(safe_state);
  const double colliding_geom = geom_eval.evaluate(colliding_state);
  if (!(safe_geom > 0.0)) {
    std::cerr << "expected_positive_safe_geometry_clearance got=" << safe_geom << std::endl;
    return 1;
  }
  if (!(colliding_geom < 0.0)) {
    std::cerr << "expected_negative_collision_geometry_clearance got=" << colliding_geom << std::endl;
    return 1;
  }
  if (std::abs(safe_grid - safe_geom) > 0.25) {
    std::cerr << "grid_geometry_diverged safe_grid=" << safe_grid
              << " safe_geom=" << safe_geom << std::endl;
    return 1;
  }

  Trajectory traj;
  PolyPiece pp;
  pp.duration = 0.8;
  pp.coeffs(0, 0) = colliding_state.x;
  pp.coeffs(1, 0) = colliding_state.y;
  traj.pos_pieces.push_back(pp);

  YawPolyPiece yp;
  yp.duration = 0.8;
  yp.coeffs(0, 0) = colliding_state.yaw;
  traj.yaw_pieces.push_back(yp);

  const double traj_clearance = geom_eval.evaluateTrajectory(traj);
  if (!(traj_clearance < 0.0)) {
    std::cerr << "expected_negative_geometry_traj_clearance got=" << traj_clearance << std::endl;
    return 1;
  }

  Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
  double grad_yaw = 0.0;
  geom_eval.gradient(colliding_state, grad_pos, grad_yaw);
  if (!std::isfinite(grad_pos.x()) || !std::isfinite(grad_pos.y()) ||
      !std::isfinite(grad_yaw)) {
    std::cerr << "non_finite_geometry_gradient" << std::endl;
    return 1;
  }

  std::cout << "safe_grid=" << safe_grid
            << " safe_geom=" << safe_geom
            << " colliding_geom=" << colliding_geom
            << " traj_clearance=" << traj_clearance << std::endl;
  return 0;
}
