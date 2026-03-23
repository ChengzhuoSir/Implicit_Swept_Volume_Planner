#include "isweep_planner/optimization/backend/trajectory_optimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Dense>

namespace isweep_planner {

TrajectoryOptimizer::TrajectoryOptimizer() {}

void TrajectoryOptimizer::init(const GridMap& map, const SvsdfEvaluator& svsdf,
                               const OptimizerParams& params) {
  map_ = &map;
  svsdf_ = &svsdf;
  params_ = params;
}

std::vector<double> TrajectoryOptimizer::allocateTime(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  std::vector<double> dists(piece_count, 0.0);
  double total_dist = 0.0;
  for (int i = 0; i < piece_count; ++i) {
    const double dx = waypoints[i + 1].x - waypoints[i].x;
    const double dy = waypoints[i + 1].y - waypoints[i].y;
    dists[i] = std::sqrt(dx * dx + dy * dy);
    if (dists[i] < 1e-9) {
      dists[i] = 1e-9;
    }
    total_dist += dists[i];
  }
  std::vector<double> durations(piece_count, 0.01);
  for (int i = 0; i < piece_count; ++i) {
    durations[i] = total_time * dists[i] / total_dist;
    if (durations[i] < 0.01) {
      durations[i] = 0.01;
    }
  }
  return durations;
}

void TrajectoryOptimizer::fitMincoQuintic(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& durations,
    const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
    const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
    std::vector<PolyPiece>& pieces) {
  const int piece_count = static_cast<int>(positions.size()) - 1;
  if (piece_count <= 0) {
    pieces.clear();
    return;
  }

  std::vector<Eigen::Vector2d> velocities(piece_count + 1);
  std::vector<Eigen::Vector2d> accelerations(piece_count + 1);
  velocities[0] = v0;
  velocities[piece_count] = vf;
  accelerations[0] = a0;
  accelerations[piece_count] = af;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) {
      dt_sum = 1e-6;
    }
    velocities[i] = (positions[i + 1] - positions[i - 1]) / dt_sum;
    const double vel_norm = velocities[i].norm();
    const double max_vel = 2.0;
    if (vel_norm > max_vel) {
      velocities[i] *= max_vel / vel_norm;
    }
  }

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) {
      dt_sum = 1e-6;
    }
    accelerations[i] =
        (velocities[std::min(i + 1, piece_count)] -
         velocities[std::max(i - 1, 0)]) /
        dt_sum;
    const double acc_norm = accelerations[i].norm();
    const double max_acc = 2.0;
    if (acc_norm > max_acc) {
      accelerations[i] *= max_acc / acc_norm;
    }
  }

  pieces.resize(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    const double t = durations[i];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    Eigen::Matrix<double, 6, 6> system;
    system << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, t,
        t2, t3, t4, t5, 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4, 0, 0, 2, 6 * t,
        12 * t2, 20 * t3;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 6>> qr =
        system.colPivHouseholderQr();

    PolyPiece piece;
    piece.duration = t;
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Matrix<double, 6, 1> rhs;
      rhs(0) = positions[i](axis);
      rhs(1) = velocities[i](axis);
      rhs(2) = accelerations[i](axis);
      rhs(3) = positions[i + 1](axis);
      rhs(4) = velocities[i + 1](axis);
      rhs(5) = accelerations[i + 1](axis);

      const Eigen::Matrix<double, 6, 1> coeffs = qr.solve(rhs);
      for (int j = 0; j < 6; ++j) {
        const double value = coeffs(j);
        piece.coeffs(axis, j) = std::isfinite(value) ? value : 0.0;
      }
    }
    pieces[i] = piece;
  }
}

void TrajectoryOptimizer::fitYawQuintic(
    const std::vector<double>& yaws,
    const std::vector<double>& durations,
    double omega0, double omegaf,
    std::vector<YawPolyPiece>& pieces) {
  const int piece_count = static_cast<int>(yaws.size()) - 1;
  if (piece_count <= 0) {
    pieces.clear();
    return;
  }

  std::vector<double> omegas(piece_count + 1, 0.0);
  std::vector<double> alphas(piece_count + 1, 0.0);
  omegas[0] = omega0;
  omegas[piece_count] = omegaf;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-12) {
      dt_sum = 1e-12;
    }
    const double yaw_delta = normalizeAngle(yaws[i + 1] - yaws[i - 1]);
    omegas[i] = yaw_delta / dt_sum;
    alphas[i] = 0.0;
  }

  pieces.resize(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    const double t = durations[i];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    Eigen::Matrix<double, 6, 6> system;
    system << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, t,
        t2, t3, t4, t5, 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4, 0, 0, 2, 6 * t,
        12 * t2, 20 * t3;

    Eigen::Matrix<double, 6, 1> rhs;
    rhs(0) = yaws[i];
    rhs(1) = omegas[i];
    rhs(2) = alphas[i];
    rhs(3) = yaws[i] + normalizeAngle(yaws[i + 1] - yaws[i]);
    rhs(4) = omegas[i + 1];
    rhs(5) = alphas[i + 1];

    const Eigen::Matrix<double, 6, 1> coeffs =
        system.colPivHouseholderQr().solve(rhs);

    YawPolyPiece piece;
    piece.duration = t;
    for (int j = 0; j < 6; ++j) {
      piece.coeffs(0, j) = coeffs(j);
    }
    pieces[i] = piece;
  }
}

Trajectory TrajectoryOptimizer::optimizeSE2(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int waypoint_count = static_cast<int>(waypoints.size());
  if (waypoint_count < 2) {
    return Trajectory();
  }

  std::vector<SE2State> optimized_waypoints = waypoints;
  std::vector<double> durations = allocateTime(optimized_waypoints, total_time);
  const double step = params_.step_size;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(waypoint_count);
    std::vector<double> yaws(waypoint_count, 0.0);
    for (int i = 0; i < waypoint_count; ++i) {
      positions[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
      yaws[i] = optimized_waypoints[i].yaw;
    }

    double total_grad_norm = 0.0;
    for (int i = 1; i < waypoint_count - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;

      const double dt_prev = durations[i - 1];
      const double dt_next = durations[i];
      const double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      const Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      const double smooth_norm = grad_smooth.norm();
      if (smooth_norm > 100.0) {
        grad_smooth *= 100.0 / smooth_norm;
      }
      grad_pos += params_.lambda_smooth * grad_smooth;

      const double yaw_dd =
          (normalizeAngle(yaws[i + 1] - yaws[i]) -
           normalizeAngle(yaws[i] - yaws[i - 1])) /
          (dt_avg * dt_avg);
      double yaw_smooth_grad =
          params_.lambda_smooth * (-4.0 * yaw_dd / (dt_avg * dt_avg));
      if (std::abs(yaw_smooth_grad) > 100.0) {
        yaw_smooth_grad = (yaw_smooth_grad > 0.0) ? 100.0 : -100.0;
      }
      grad_yaw += yaw_smooth_grad;

      if (svsdf_) {
        const double sdf_value = svsdf_->evaluate(optimized_waypoints[i]);
        if (sdf_value < params_.safety_margin) {
          Eigen::Vector2d grad_pos_sdf;
          double grad_yaw_sdf = 0.0;
          svsdf_->gradient(optimized_waypoints[i], grad_pos_sdf, grad_yaw_sdf);
          const double penalty = params_.safety_margin - sdf_value;
          grad_pos -= params_.lambda_safety * 2.0 * penalty * grad_pos_sdf;
          grad_yaw -= params_.lambda_safety * 2.0 * penalty * grad_yaw_sdf;
        }
      }

      double dt_span = dt_prev + dt_next;
      if (dt_span < 1e-12) {
        dt_span = 1e-12;
      }
      const Eigen::Vector2d vel_est = (positions[i + 1] - positions[i - 1]) / dt_span;
      const double vel_norm = vel_est.norm();
      if (vel_norm > params_.max_vel) {
        const double excess = vel_norm - params_.max_vel;
        const Eigen::Vector2d midpoint = 0.5 * (positions[i - 1] + positions[i + 1]);
        grad_pos += params_.lambda_dynamics * excess *
                    (positions[i] - midpoint) / (vel_norm + 1e-12);
      }

      const double acc_norm = acc_approx.norm();
      if (acc_norm > params_.max_acc) {
        const double excess = acc_norm - params_.max_acc;
        grad_pos += params_.lambda_dynamics * excess *
                    (-2.0 * acc_approx) /
                    (acc_norm * dt_avg * dt_avg + 1e-12);
      }

      const double grad_norm = grad_pos.norm();
      if (grad_norm > 50.0) {
        grad_pos *= 50.0 / grad_norm;
      }
      if (std::abs(grad_yaw) > 50.0) {
        grad_yaw = (grad_yaw > 0.0) ? 50.0 : -50.0;
      }

      optimized_waypoints[i].x -= step * grad_pos.x();
      optimized_waypoints[i].y -= step * grad_pos.y();
      optimized_waypoints[i].yaw =
          normalizeAngle(optimized_waypoints[i].yaw - step * grad_yaw);

      total_grad_norm += grad_pos.squaredNorm() + grad_yaw * grad_yaw;
    }

    durations = allocateTime(optimized_waypoints, total_time);
    if (total_grad_norm < 1e-10) {
      break;
    }
  }

  const double min_piece_len = 1.5;
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(Eigen::Vector2d(optimized_waypoints[0].x, optimized_waypoints[0].y));
  final_yaws.push_back(optimized_waypoints[0].yaw);

  double accum_dist = 0.0;
  for (int i = 1; i < waypoint_count; ++i) {
    const double dx = optimized_waypoints[i].x - optimized_waypoints[i - 1].x;
    const double dy = optimized_waypoints[i].y - optimized_waypoints[i - 1].y;
    accum_dist += std::sqrt(dx * dx + dy * dy);
    if (accum_dist >= min_piece_len || i == waypoint_count - 1) {
      final_pos.push_back(Eigen::Vector2d(optimized_waypoints[i].x,
                                          optimized_waypoints[i].y));
      final_yaws.push_back(optimized_waypoints[i].yaw);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> downsampled(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    downsampled[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  const std::vector<double> downsampled_durations = allocateTime(downsampled, total_time);

  Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d vf = Eigen::Vector2d::Zero();
  if (final_pos.size() >= 2) {
    double dt0 = downsampled_durations.empty() ? 1.0 : downsampled_durations[0];
    if (dt0 < 0.1) {
      dt0 = 0.1;
    }
    v0 = (final_pos[1] - final_pos[0]) / dt0;
    const double v0_norm = v0.norm();
    if (v0_norm > params_.max_vel) {
      v0 *= params_.max_vel / v0_norm;
    }

    double dtf = downsampled_durations.empty() ? 1.0 : downsampled_durations.back();
    if (dtf < 0.1) {
      dtf = 0.1;
    }
    vf = (final_pos.back() - final_pos[final_pos.size() - 2]) / dtf;
    const double vf_norm = vf.norm();
    if (vf_norm > params_.max_vel) {
      vf *= params_.max_vel / vf_norm;
    }
  }

  Trajectory traj;
  fitMincoQuintic(final_pos, downsampled_durations, v0, vf, Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), traj.pos_pieces);
  fitYawQuintic(final_yaws, downsampled_durations, 0.0, 0.0, traj.yaw_pieces);
  return traj;
}

Trajectory TrajectoryOptimizer::optimizeR2(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int waypoint_count = static_cast<int>(waypoints.size());
  if (waypoint_count < 2) {
    return Trajectory();
  }

  std::vector<SE2State> optimized_waypoints = waypoints;
  const std::vector<SE2State> reference = waypoints;
  std::vector<double> durations = allocateTime(optimized_waypoints, total_time);
  const double step = params_.step_size;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(waypoint_count);
    for (int i = 0; i < waypoint_count; ++i) {
      positions[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
    }

    double total_grad_norm = 0.0;
    for (int i = 1; i < waypoint_count - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();

      const double dt_prev = durations[i - 1];
      const double dt_next = durations[i];
      const double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      const Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      const double smooth_norm = grad_smooth.norm();
      if (smooth_norm > 100.0) {
        grad_smooth *= 100.0 / smooth_norm;
      }
      grad_pos += params_.lambda_smooth * grad_smooth;

      if (svsdf_) {
        const SE2State state(optimized_waypoints[i].x, optimized_waypoints[i].y,
                             optimized_waypoints[i].yaw);
        const double sdf_value = svsdf_->evaluate(state);
        if (sdf_value < params_.safety_margin) {
          Eigen::Vector2d grad_pos_sdf;
          double grad_yaw_sdf = 0.0;
          svsdf_->gradient(state, grad_pos_sdf, grad_yaw_sdf);
          const double penalty = params_.safety_margin - sdf_value;
          grad_pos -= params_.lambda_safety * 2.0 * penalty * grad_pos_sdf;
        }
      }

      const Eigen::Vector2d ref_pos(reference[i].x, reference[i].y);
      grad_pos += params_.lambda_pos_residual * 2.0 * (positions[i] - ref_pos);

      const Eigen::Vector2d vel_dir = positions[i + 1] - positions[i - 1];
      const double vel_norm = vel_dir.norm();
      if (vel_norm > 1e-9) {
        const double yaw_curr = std::atan2(vel_dir.y(), vel_dir.x());

        double yaw_prev = optimized_waypoints[0].yaw;
        if (i >= 2) {
          const Eigen::Vector2d prev_vel = positions[i] - positions[i - 2];
          if (prev_vel.norm() > 1e-9) {
            yaw_prev = std::atan2(prev_vel.y(), prev_vel.x());
          }
        }

        double yaw_next = optimized_waypoints[waypoint_count - 1].yaw;
        if (i + 2 < waypoint_count) {
          const Eigen::Vector2d next_vel = positions[i + 2] - positions[i];
          if (next_vel.norm() > 1e-9) {
            yaw_next = std::atan2(next_vel.y(), next_vel.x());
          }
        }

        const double yaw_dd = normalizeAngle(yaw_next - yaw_curr) -
                              normalizeAngle(yaw_curr - yaw_prev);
        Eigen::Vector2d perp(-vel_dir.y(), vel_dir.x());
        perp /= vel_norm;
        grad_pos += params_.lambda_yaw_residual * yaw_dd * perp;
      }

      optimized_waypoints[i].x -= step * grad_pos.x();
      optimized_waypoints[i].y -= step * grad_pos.y();
      total_grad_norm += grad_pos.squaredNorm();
    }

    durations = allocateTime(optimized_waypoints, total_time);
    if (total_grad_norm < 1e-10) {
      break;
    }
  }

  std::vector<Eigen::Vector2d> all_pos(waypoint_count);
  std::vector<double> all_yaws(waypoint_count, 0.0);
  for (int i = 0; i < waypoint_count; ++i) {
    all_pos[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
  }
  all_yaws[0] = optimized_waypoints[0].yaw;
  all_yaws[waypoint_count - 1] = optimized_waypoints[waypoint_count - 1].yaw;
  for (int i = 1; i < waypoint_count - 1; ++i) {
    const Eigen::Vector2d vel_dir = all_pos[i + 1] - all_pos[i - 1];
    if (vel_dir.norm() > 1e-9) {
      all_yaws[i] = std::atan2(vel_dir.y(), vel_dir.x());
    } else {
      all_yaws[i] = all_yaws[i - 1];
    }
  }

  const double min_piece_len = 1.0;
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(all_pos[0]);
  final_yaws.push_back(all_yaws[0]);

  double accum_dist = 0.0;
  for (int i = 1; i < waypoint_count; ++i) {
    accum_dist += (all_pos[i] - all_pos[i - 1]).norm();
    if (accum_dist >= min_piece_len || i == waypoint_count - 1) {
      final_pos.push_back(all_pos[i]);
      final_yaws.push_back(all_yaws[i]);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> downsampled(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    downsampled[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  const std::vector<double> downsampled_durations = allocateTime(downsampled, total_time);

  Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d vf = Eigen::Vector2d::Zero();
  if (final_pos.size() >= 2) {
    double dt0 = downsampled_durations.empty() ? 1.0 : downsampled_durations[0];
    if (dt0 < 0.1) {
      dt0 = 0.1;
    }
    v0 = (final_pos[1] - final_pos[0]) / dt0;
    const double v0_norm = v0.norm();
    if (v0_norm > params_.max_vel) {
      v0 *= params_.max_vel / v0_norm;
    }

    double dtf = downsampled_durations.empty() ? 1.0 : downsampled_durations.back();
    if (dtf < 0.1) {
      dtf = 0.1;
    }
    vf = (final_pos.back() - final_pos[final_pos.size() - 2]) / dtf;
    const double vf_norm = vf.norm();
    if (vf_norm > params_.max_vel) {
      vf *= params_.max_vel / vf_norm;
    }
  }

  Trajectory traj;
  fitMincoQuintic(final_pos, downsampled_durations, v0, vf, Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), traj.pos_pieces);
  fitYawQuintic(final_yaws, downsampled_durations, 0.0, 0.0, traj.yaw_pieces);
  return traj;
}

Trajectory TrajectoryOptimizer::stitch(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& optimized) {
  Trajectory traj;
  const size_t count = std::min(segments.size(), optimized.size());
  for (size_t i = 0; i < count; ++i) {
    const Trajectory& seg_traj = optimized[i];
    if (seg_traj.empty()) {
      continue;
    }
    traj.pos_pieces.insert(traj.pos_pieces.end(), seg_traj.pos_pieces.begin(),
                           seg_traj.pos_pieces.end());
    traj.yaw_pieces.insert(traj.yaw_pieces.end(), seg_traj.yaw_pieces.begin(),
                           seg_traj.yaw_pieces.end());
  }
  return traj;
}

Trajectory TrajectoryOptimizer::selectBest(const std::vector<Trajectory>& candidates) {
  if (candidates.empty()) {
    return Trajectory();
  }

  double best_cost = kInf;
  size_t best_idx = 0;
  bool found = false;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const Trajectory& traj = candidates[i];
    if (traj.empty()) {
      continue;
    }

    if (svsdf_) {
      const double min_sdf = svsdf_->evaluateTrajectory(traj);
      if (min_sdf < -params_.safety_margin) {
        continue;
      }
    }

    double cost = 0.0;
    for (size_t j = 0; j < traj.pos_pieces.size(); ++j) {
      const PolyPiece& piece = traj.pos_pieces[j];
      const double duration = piece.duration;
      const int n_sub = 20;
      const double h = duration / static_cast<double>(n_sub);
      for (int s = 0; s <= n_sub; ++s) {
        const double t = s * h;
        const Eigen::Vector2d acc = piece.acceleration(t);
        const double weight =
            (s == 0 || s == n_sub) ? 1.0 : ((s % 2 == 1) ? 4.0 : 2.0);
        cost += weight * acc.squaredNorm() * h / 3.0;
      }
    }

    if (cost < best_cost) {
      best_cost = cost;
      best_idx = i;
      found = true;
    }
  }

  if (!found) {
    return Trajectory();
  }
  return candidates[best_idx];
}

double TrajectoryOptimizer::computeCost(
    const Trajectory& traj, const std::vector<SE2State>& ref_waypoints,
    bool include_svsdf, bool include_residual) {
  if (traj.empty()) {
    return kInf;
  }

  double cost = 0.0;
  const double total_duration = traj.totalDuration();
  const double sample_dt = 0.05;
  const int sample_count = std::max(1, static_cast<int>(total_duration / sample_dt));
  const double dt = total_duration / static_cast<double>(sample_count);

  for (int s = 0; s <= sample_count; ++s) {
    const double t = std::min(static_cast<double>(s) * dt, total_duration);
    double accum_time = 0.0;
    for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
      const double piece_duration = traj.pos_pieces[i].duration;
      if (t <= accum_time + piece_duration || i + 1 == traj.pos_pieces.size()) {
        const double local_t = t - accum_time;
        const Eigen::Vector2d acc = traj.pos_pieces[i].acceleration(local_t);
        cost += params_.lambda_smooth * acc.squaredNorm() * dt;

        if (include_svsdf && svsdf_) {
          const SE2State state = traj.sample(t);
          const double sdf_value = svsdf_->evaluate(state);
          if (sdf_value < params_.safety_margin) {
            const double penalty = params_.safety_margin - sdf_value;
            cost += params_.lambda_safety * penalty * penalty * dt;
          }
        }
        break;
      }
      accum_time += piece_duration;
    }
  }

  if (include_residual && !ref_waypoints.empty()) {
    const int ref_count = static_cast<int>(ref_waypoints.size());
    const double ref_dt =
        (ref_count > 1) ? total_duration / static_cast<double>(ref_count - 1) : 0.0;
    for (int i = 0; i < ref_count; ++i) {
      const double t_ref = static_cast<double>(i) * ref_dt;
      const SE2State sample = traj.sample(t_ref);
      const double dx = sample.x - ref_waypoints[i].x;
      const double dy = sample.y - ref_waypoints[i].y;
      cost += params_.lambda_pos_residual * (dx * dx + dy * dy);

      const double dyaw = normalizeAngle(sample.yaw - ref_waypoints[i].yaw);
      cost += params_.lambda_yaw_residual * dyaw * dyaw;
    }
  }

  cost += params_.lambda_time * total_duration;
  return cost;
}

bool TrajectoryOptimizer::checkDynamics(const Trajectory& traj) {
  const double sample_dt = 0.02;
  for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
    const PolyPiece& piece = traj.pos_pieces[i];
    const double duration = piece.duration;
    const int sample_count = std::max(1, static_cast<int>(duration / sample_dt));
    const double dt = duration / static_cast<double>(sample_count);
    for (int s = 0; s <= sample_count; ++s) {
      const double t = static_cast<double>(s) * dt;
      if (piece.velocity(t).norm() > params_.max_vel * 1.10) {
        return false;
      }
      if (piece.acceleration(t).norm() > params_.max_acc * 1.10) {
        return false;
      }
    }
  }

  for (size_t i = 0; i < traj.yaw_pieces.size(); ++i) {
    const YawPolyPiece& piece = traj.yaw_pieces[i];
    const double duration = piece.duration;
    const int sample_count = std::max(1, static_cast<int>(duration / sample_dt));
    const double dt = duration / static_cast<double>(sample_count);
    for (int s = 0; s <= sample_count; ++s) {
      const double t = static_cast<double>(s) * dt;
      if (std::abs(piece.velocity(t)) > params_.max_yaw_rate * 1.10) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace isweep_planner
