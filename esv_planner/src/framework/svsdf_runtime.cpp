#include "esv_planner/framework/svsdf_runtime.h"
#include "esv_planner/ros_interface/ros_visualizer.h"
#include "esv_planner/framework/recovery_manager.h"
#include "esv_planner/global_search/swept_astar.h"
#include "esv_planner/core/trajectory_utils.h"
#include "esv_planner/core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <geometry_msgs/PoseStamped.h>

namespace esv_planner {

void SvsdfRuntime::PushSegmentWaypointsFromObstacles(
    std::vector<MotionSegment>* segments) const {
  if (segments == nullptr) return;
  const double push_margin = 0.40;
  const double push_step = 0.12;
  const int max_iters = 20;

  for (auto& seg : *segments) {
    for (auto& wp : seg.waypoints) {
      Eigen::Vector2d pos(wp.x, wp.y);
      math::PushPointFromObstacle(grid_map_, pos, push_margin, push_step, max_iters);
      wp.x = pos.x();
      wp.y = pos.y();
    }
  }

  for (size_t i = 0; i + 1 < segments->size(); ++i) {
    if (!(*segments)[i].waypoints.empty() && !(*segments)[i + 1].waypoints.empty()) {
      (*segments)[i + 1].waypoints.front() = (*segments)[i].waypoints.back();
    }
  }
}

void SvsdfRuntime::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  LoadParameters(pnh);
  strict_solver_.Initialize(nh, pnh, footprint_);
  initialized_ = true;
}

void SvsdfRuntime::LoadParameters(ros::NodeHandle& pnh) {
  footprint_.setPolygon(LoadFootprint(pnh));

  double inscribed_radius = 0.08;
  pnh.param("inscribed_radius", inscribed_radius, 0.08);
  footprint_.setInscribedRadius(inscribed_radius);

  pnh.param("topology/num_samples", topology_num_samples_, 520);
  pnh.param("topology/knn", topology_knn_, 18);
  pnh.param("topology/max_paths", topology_max_paths_, 14);

  pnh.param("se2/discretization_step", se2_disc_step_, 0.15);
  pnh.param("kernel_yaw_num", yaw_bins_, 18);
  pnh.param("se2/max_push_attempts", max_push_attempts_, 5);

  pnh.param("optimizer/max_iterations", optimizer_params_.max_iterations, 100);
  pnh.param("optimizer/lambda_smooth", optimizer_params_.lambda_smooth, 1.0);
  pnh.param("optimizer/lambda_time", optimizer_params_.lambda_time, 1.0);
  pnh.param("optimizer/lambda_safety", optimizer_params_.lambda_safety, 10.0);
  pnh.param("optimizer/lambda_dynamics", optimizer_params_.lambda_dynamics, 1.0);
  pnh.param("optimizer/lambda_pos_residual",
            optimizer_params_.lambda_pos_residual, 5.0);
  pnh.param("optimizer/lambda_yaw_residual",
            optimizer_params_.lambda_yaw_residual, 2.0);
  pnh.param("optimizer/max_vel", optimizer_params_.max_vel, 1.0);
  pnh.param("optimizer/max_acc", optimizer_params_.max_acc, 2.0);
  pnh.param("optimizer/max_yaw_rate", optimizer_params_.max_yaw_rate, 1.5);
  pnh.param("optimizer/safety_margin", optimizer_params_.safety_margin, 0.05);
  pnh.param("optimizer/step_size", optimizer_params_.step_size, 0.005);

  pnh.param("hybrid_astar/step_size", hybrid_astar_params_.step_size, 0.35);
  pnh.param("hybrid_astar/wheel_base", hybrid_astar_params_.wheel_base, 0.8);
  pnh.param("hybrid_astar/max_steer", hybrid_astar_params_.max_steer, 0.6);
  pnh.param("hybrid_astar/steer_samples", hybrid_astar_params_.steer_samples, 5);
  pnh.param("hybrid_astar/goal_tolerance_pos",
            hybrid_astar_params_.goal_tolerance_pos, 0.35);
  pnh.param("hybrid_astar/goal_tolerance_yaw",
            hybrid_astar_params_.goal_tolerance_yaw, 0.4);
  pnh.param("hybrid_astar/reverse_penalty",
            hybrid_astar_params_.reverse_penalty, 2.0);
  pnh.param("hybrid_astar/steer_penalty",
            hybrid_astar_params_.steer_penalty, 0.2);
  pnh.param("hybrid_astar/steer_change_penalty",
            hybrid_astar_params_.steer_change_penalty, 0.2);
  pnh.param("hybrid_astar/switch_penalty",
            hybrid_astar_params_.switch_penalty, 2.0);
  pnh.param("hybrid_astar/max_expansions",
            hybrid_astar_params_.max_expansions, 50000);
  pnh.param("hybrid_astar/fast_pass_max_expansions",
            hybrid_astar_params_.fast_pass_max_expansions, 8000);
  pnh.param("hybrid_astar/fast_pass_steer_samples",
            hybrid_astar_params_.fast_pass_steer_samples, 3);
  pnh.param("hybrid_astar/pose_recovery_max_radius",
            hybrid_astar_params_.pose_recovery_max_radius, 0.6);
  pnh.param("hybrid_astar/pose_recovery_radial_step",
            hybrid_astar_params_.pose_recovery_radial_step, 0.05);
  pnh.param("hybrid_astar/pose_recovery_angular_samples",
            hybrid_astar_params_.pose_recovery_angular_samples, 24);
}

bool SvsdfRuntime::UpdateMap(const nav_msgs::OccupancyGrid& map) {
  if (!initialized_) {
    return false;
  }

  latest_map_ = map;
  nav_msgs::OccupancyGrid::Ptr map_ptr(new nav_msgs::OccupancyGrid(map));
  grid_map_.fromOccupancyGrid(map_ptr);

  collision_checker_.init(grid_map_, footprint_, yaw_bins_);
  se2_generator_.init(grid_map_, collision_checker_, se2_disc_step_,
                      max_push_attempts_);
  hybrid_astar_.init(grid_map_, collision_checker_, hybrid_astar_params_);
  const double topo_safe_dist = 0.25;
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_, topo_safe_dist);
  svsdf_evaluator_.initGridEsdf(grid_map_, footprint_);
  optimizer_.init(grid_map_, svsdf_evaluator_, optimizer_params_);
  if (!strict_solver_.UpdateMap(map)) {
    map_ready_ = false;
    return false;
  }
  strict_solver_.setGridMapForMidend(&grid_map_);

  map_ready_ = true;
  return true;
}





bool SvsdfRuntime::SolveStrictSegment(const std::vector<SE2State>& waypoints,
                                      Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }
  const double raw_clearance = svsdf_evaluator_.segmentClearance(waypoints, 0.10);
  const bool preserve_dense_input =
      std::isfinite(raw_clearance) && raw_clearance > -0.051;
  std::vector<SE2State> sparse_waypoints =
      preserve_dense_input ? waypoints : SparsifyStrictWaypoints(waypoints, grid_map_);
  ROS_INFO(
      "Runtime stage: strict SE(2) input %s from %zu to %zu points (raw_clearance=%.3f)",
      preserve_dense_input ? "preserved" : "sparsified", waypoints.size(),
      sparse_waypoints.size(), raw_clearance);

  // Push interior waypoints away from obstacles along ESDF gradient
  const double push_margin = 0.40;
  const double push_step = 0.10;
  const int max_push_iters = 15;
  if (!preserve_dense_input) {
    for (size_t i = 1; i + 1 < sparse_waypoints.size(); ++i) {
      Eigen::Vector2d pos(sparse_waypoints[i].x, sparse_waypoints[i].y);
      math::PushPointFromObstacle(grid_map_, pos, push_margin, push_step, max_push_iters);
      sparse_waypoints[i].x = pos.x();
      sparse_waypoints[i].y = pos.y();
    }
  }

  if (!strict_solver_.ready() ||
      !strict_solver_.Solve(sparse_waypoints, trajectory, preserve_dense_input) ||
      trajectory->empty()) {
    return false;
  }

  if (preserve_dense_input) {
    double solved_clearance = -kInf;
    if (!IsTrajectoryFeasible(*trajectory, &solved_clearance, nullptr, nullptr)) {
      ROS_WARN(
          "Strict SE(2) solver degraded a feasible waypoint chain: raw_clearance=%.3f solved_clearance=%.3f",
          raw_clearance, solved_clearance);
    }
  }

  return true;
}

bool SvsdfRuntime::OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                                          double seg_time,
                                          Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }

  // Use a balanced sparsification for low-risk segments to maintain
  // continuity at stitching points while keeping open space smooth.
  std::vector<SE2State> sparse_waypoints;
  if (waypoints.size() > 4) {
    sparse_waypoints.push_back(waypoints.front());
    sparse_waypoints.push_back(waypoints[waypoints.size() / 2]);
    sparse_waypoints.push_back(waypoints.back());
  } else {
    sparse_waypoints = waypoints;
  }

  *trajectory = optimizer_.optimizeR2(sparse_waypoints, seg_time);
  if (!trajectory->empty()) {
    return true;
  }
  return SolveStrictSegment(waypoints, trajectory);
}

bool SvsdfRuntime::IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                                        double* max_vel, double* max_acc) const {
  if (traj.empty()) {
    return false;
  }

  double clearance = svsdf_evaluator_.evaluateTrajectory(traj);
  double vmax = 0.0;
  double amax = 0.0;
  const double total = traj.totalDuration();
  for (double t = 0.0; t <= total; t += 0.02) {
    vmax = std::max(vmax, traj.sampleVelocity(t).norm());
    amax = std::max(amax, traj.sampleAcceleration(t).norm());
  }

  if (min_clearance) {
    *min_clearance = clearance;
  }
  if (max_vel) {
    *max_vel = vmax;
  }
  if (max_acc) {
    *max_acc = amax;
  }

  return std::isfinite(clearance) && clearance > -0.051;
}

bool SvsdfRuntime::SolveStrictCached(size_t idx, const std::vector<SE2State>& wp,
                                     Trajectory* traj, SegmentCache& cache) {
  SegmentCacheKey key(idx, true);
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = SolveStrictSegment(wp, traj);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
}

bool SvsdfRuntime::OptimizeLowRiskCached(size_t idx, const std::vector<SE2State>& wp,
                                         double t, Trajectory* traj, SegmentCache& cache) {
  SegmentCacheKey key(idx, false);
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = OptimizeLowRiskSegment(wp, t, traj);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
}

double SvsdfRuntime::EstimateSegmentTime(
    const std::vector<SE2State>& waypoints) const {
  if (waypoints.size() < 2) {
    return 0.5;
  }
  double length = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double dx = waypoints[i].x - waypoints[i - 1].x;
    const double dy = waypoints[i].y - waypoints[i - 1].y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return std::max(0.5, length / std::max(0.10, optimizer_params_.max_vel));
}

}  // namespace esv_planner
