#include "esv_planner/trajectory_optimizer.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace esv_planner {

TrajectoryOptimizer::TrajectoryOptimizer() {}

void TrajectoryOptimizer::init(const GridMap& map, const SvsdfEvaluator& svsdf,
                                const OptimizerParams& params) {
  map_ = &map;
  svsdf_ = &svsdf;
  params_ = params;
}

void TrajectoryOptimizer::initMincoFromWaypoints(const std::vector<SE2State>& wps,
                                                   double total_time,
                                                   std::vector<PolyPiece>& pieces) {
  // Initialize MINCO pieces with linear interpolation between waypoints
  int n_pieces = static_cast<int>(wps.size()) - 1;
  if (n_pieces <= 0) return;

  double dt = total_time / n_pieces;
  pieces.resize(n_pieces);

  for (int i = 0; i < n_pieces; ++i) {
    pieces[i].duration = dt;
    pieces[i].coeffs = Eigen::Matrix<double, 2, 6>::Zero();

    Eigen::Vector2d p0(wps[i].x, wps[i].y);
    Eigen::Vector2d p1(wps[i + 1].x, wps[i + 1].y);

    // Linear: p(t) = p0 + (p1 - p0) * t / dt
    pieces[i].coeffs.col(0) = p0;
    if (dt > 1e-9) {
      pieces[i].coeffs.col(1) = (p1 - p0) / dt;
    }
  }
}

Trajectory TrajectoryOptimizer::optimizeSE2(const std::vector<SE2State>& waypoints,
                                             double total_time) {
  Trajectory traj;
  if (waypoints.size() < 2) return traj;

  // Initialize MINCO from waypoints
  initMincoFromWaypoints(waypoints, total_time, traj.pos_pieces);

  // Initialize yaw pieces
  int n_pieces = static_cast<int>(waypoints.size()) - 1;
  traj.yaw_pieces.resize(n_pieces);
  double dt = total_time / n_pieces;
  for (int i = 0; i < n_pieces; ++i) {
    traj.yaw_pieces[i].duration = dt;
    traj.yaw_pieces[i].coeffs = Eigen::Matrix<double, 1, 6>::Zero();
    traj.yaw_pieces[i].coeffs(0, 0) = waypoints[i].yaw;
    if (dt > 1e-9) {
      double dyaw = normalizeAngle(waypoints[i + 1].yaw - waypoints[i].yaw);
      traj.yaw_pieces[i].coeffs(0, 1) = dyaw / dt;
    }
  }

  // Gradient descent optimization (Eq. 3)
  double step_size = 0.01;
  std::vector<SE2State> opt_wps(waypoints);

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    double total_cost = 0.0;

    for (int i = 1; i < static_cast<int>(opt_wps.size()) - 1; ++i) {
      // Safety cost gradient (SVSDF)
      Eigen::Vector2d grad_pos;
      double grad_yaw;
      svsdf_->gradient(opt_wps[i], grad_pos, grad_yaw);

      double svsdf_val = svsdf_->evaluate(opt_wps[i]);
      double safety_cost = 0.0;
      Eigen::Vector2d safety_grad = Eigen::Vector2d::Zero();
      double safety_grad_yaw = 0.0;

      if (svsdf_val < 0.1) {
        safety_cost = params_.lambda_safety * (0.1 - svsdf_val) * (0.1 - svsdf_val);
        safety_grad = -params_.lambda_safety * 2.0 * (0.1 - svsdf_val) * grad_pos;
        safety_grad_yaw = -params_.lambda_safety * 2.0 * (0.1 - svsdf_val) * grad_yaw;
      }

      // Smoothness cost gradient (second-order finite difference)
      Eigen::Vector2d smooth_grad = Eigen::Vector2d::Zero();
      if (i > 0 && i < static_cast<int>(opt_wps.size()) - 1) {
        Eigen::Vector2d pi(opt_wps[i].x, opt_wps[i].y);
        Eigen::Vector2d pi_prev(opt_wps[i - 1].x, opt_wps[i - 1].y);
        Eigen::Vector2d pi_next(opt_wps[i + 1].x, opt_wps[i + 1].y);
        smooth_grad = params_.lambda_smooth * 2.0 * (2.0 * pi - pi_prev - pi_next);
      }

      // Update
      opt_wps[i].x -= step_size * (smooth_grad.x() + safety_grad.x());
      opt_wps[i].y -= step_size * (smooth_grad.y() + safety_grad.y());
      opt_wps[i].yaw -= step_size * safety_grad_yaw;
      opt_wps[i].normalizeYaw();

      total_cost += safety_cost + smooth_grad.squaredNorm();
    }

    // Early termination
    if (total_cost < 1e-6) break;
  }

  // Rebuild trajectory from optimized waypoints
  initMincoFromWaypoints(opt_wps, total_time, traj.pos_pieces);

  // Rebuild yaw
  for (int i = 0; i < n_pieces; ++i) {
    traj.yaw_pieces[i].coeffs(0, 0) = opt_wps[i].yaw;
    if (dt > 1e-9) {
      double dyaw = normalizeAngle(opt_wps[i + 1].yaw - opt_wps[i].yaw);
      traj.yaw_pieces[i].coeffs(0, 1) = dyaw / dt;
    }
  }

  return traj;
}

Trajectory TrajectoryOptimizer::optimizeR2(const std::vector<SE2State>& waypoints,
                                            double total_time) {
  Trajectory traj;
  if (waypoints.size() < 2) return traj;

  // R² optimization: position only, with position/rotation residual penalties (Eq. 4)
  initMincoFromWaypoints(waypoints, total_time, traj.pos_pieces);

  int n_pieces = static_cast<int>(waypoints.size()) - 1;
  traj.yaw_pieces.resize(n_pieces);
  double dt = total_time / n_pieces;

  // Simple smoothing for low-risk segments
  std::vector<SE2State> opt_wps(waypoints);

  for (int iter = 0; iter < params_.max_iterations / 2; ++iter) {
    for (int i = 1; i < static_cast<int>(opt_wps.size()) - 1; ++i) {
      Eigen::Vector2d pi(opt_wps[i].x, opt_wps[i].y);
      Eigen::Vector2d pi_prev(opt_wps[i - 1].x, opt_wps[i - 1].y);
      Eigen::Vector2d pi_next(opt_wps[i + 1].x, opt_wps[i + 1].y);

      // Smoothness gradient
      Eigen::Vector2d smooth_grad = params_.lambda_smooth * 2.0 * (2.0 * pi - pi_prev - pi_next);

      // Position residual (stay close to original)
      Eigen::Vector2d orig(waypoints[i].x, waypoints[i].y);
      Eigen::Vector2d pos_residual = 2.0 * (pi - orig);

      opt_wps[i].x -= 0.01 * (smooth_grad.x() + pos_residual.x());
      opt_wps[i].y -= 0.01 * (smooth_grad.y() + pos_residual.y());
    }
  }

  initMincoFromWaypoints(opt_wps, total_time, traj.pos_pieces);

  // Yaw: interpolate from velocity direction
  for (int i = 0; i < n_pieces; ++i) {
    traj.yaw_pieces[i].duration = dt;
    traj.yaw_pieces[i].coeffs = Eigen::Matrix<double, 1, 6>::Zero();

    Eigen::Vector2d dir(opt_wps[i + 1].x - opt_wps[i].x,
                        opt_wps[i + 1].y - opt_wps[i].y);
    double yaw = (dir.norm() > 1e-9) ? std::atan2(dir.y(), dir.x()) : opt_wps[i].yaw;
    traj.yaw_pieces[i].coeffs(0, 0) = yaw;
  }

  return traj;
}

Trajectory TrajectoryOptimizer::stitch(const std::vector<MotionSegment>& segments,
                                        const std::vector<Trajectory>& optimized) {
  Trajectory result;
  for (size_t i = 0; i < optimized.size(); ++i) {
    for (const auto& pp : optimized[i].pos_pieces) {
      result.pos_pieces.push_back(pp);
    }
    for (const auto& yp : optimized[i].yaw_pieces) {
      result.yaw_pieces.push_back(yp);
    }
  }
  return result;
}

Trajectory TrajectoryOptimizer::selectBest(const std::vector<Trajectory>& candidates) {
  if (candidates.empty()) return Trajectory();

  double best_cost = kInf;
  int best_idx = 0;

  for (size_t i = 0; i < candidates.size(); ++i) {
    // Compute control cost: integral of acceleration squared
    double cost = 0.0;
    double dt_sample = 0.05;
    double total = candidates[i].totalDuration();

    for (double t = 0.0; t <= total; t += dt_sample) {
      // Find piece
      double acc_time = 0.0;
      for (size_t p = 0; p < candidates[i].pos_pieces.size(); ++p) {
        if (t <= acc_time + candidates[i].pos_pieces[p].duration ||
            p + 1 == candidates[i].pos_pieces.size()) {
          double local_t = t - acc_time;
          Eigen::Vector2d a = candidates[i].pos_pieces[p].acceleration(local_t);
          cost += a.squaredNorm() * dt_sample;
          break;
        }
        acc_time += candidates[i].pos_pieces[p].duration;
      }
    }

    if (cost < best_cost) {
      best_cost = cost;
      best_idx = static_cast<int>(i);
    }
  }

  return candidates[best_idx];
}

}  // namespace esv_planner
