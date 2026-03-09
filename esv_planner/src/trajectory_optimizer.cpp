#include "esv_planner/trajectory_optimizer.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace esv_planner {

namespace {

double waypointChainClearance(const std::vector<SE2State>& waypoints,
                              const SvsdfEvaluator* svsdf) {
  if (!svsdf || waypoints.empty()) return kInf;
  double min_clearance = kInf;
  for (const auto& wp : waypoints) {
    min_clearance = std::min(min_clearance, svsdf->evaluate(wp));
  }
  return min_clearance;
}

double continuousWaypointChainClearance(const std::vector<SE2State>& waypoints,
                                        const SvsdfEvaluator* svsdf,
                                        double sample_step) {
  if (!svsdf || waypoints.empty()) return kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double len = (b.position() - a.position()).norm();
    const int n_steps = std::max(
        1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
    for (int s = 0; s <= n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      SE2State st;
      st.x = a.x + (b.x - a.x) * t;
      st.y = a.y + (b.y - a.y) * t;
      st.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
      min_clearance = std::min(min_clearance, svsdf->evaluate(st));
    }
  }
  return min_clearance;
}

double transitionClearance(const SE2State& from,
                           const SE2State& to,
                           const SvsdfEvaluator* svsdf,
                           double sample_step) {
  if (!svsdf) return kInf;
  const double len = (to.position() - from.position()).norm();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = from.x + (to.x - from.x) * t;
    st.y = from.y + (to.y - from.y) * t;
    st.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

double constantYawTranslationClearance(const SE2State& from,
                                       const SE2State& to,
                                       double yaw,
                                       const SvsdfEvaluator* svsdf,
                                       double sample_step) {
  if (!svsdf) return kInf;
  const double len = (to.position() - from.position()).norm();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = from.x + (to.x - from.x) * t;
    st.y = from.y + (to.y - from.y) * t;
    st.yaw = yaw;
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

double inPlaceRotationClearance(const Eigen::Vector2d& pos,
                                double yaw0,
                                double yaw1,
                                const SvsdfEvaluator* svsdf,
                                double sample_step) {
  if (!svsdf) return kInf;
  const double yaw_delta = std::abs(normalizeAngle(yaw1 - yaw0));
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(yaw_delta / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = pos.x();
    st.y = pos.y();
    st.yaw = normalizeAngle(yaw0 + normalizeAngle(yaw1 - yaw0) * t);
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer() {}

void TrajectoryOptimizer::init(const GridMap& map, const SvsdfEvaluator& svsdf,
                               const OptimizerParams& params) {
  map_ = &map;
  svsdf_ = &svsdf;
  params_ = params;
}

// ---------------------------------------------------------------------------
// allocateTime: distribute total_time proportional to segment distances
// ---------------------------------------------------------------------------
std::vector<double> TrajectoryOptimizer::allocateTime(
    const std::vector<SE2State>& wps, double total_time) const {
  int n = static_cast<int>(wps.size()) - 1;
  std::vector<double> dists(n, 0.0);
  double total_dist = 0.0;
  for (int i = 0; i < n; ++i) {
    double dx = wps[i + 1].x - wps[i].x;
    double dy = wps[i + 1].y - wps[i].y;
    dists[i] = std::sqrt(dx * dx + dy * dy);
    if (dists[i] < 1e-9) dists[i] = 1e-9;
    total_dist += dists[i];
  }
  std::vector<double> durations(n);
  for (int i = 0; i < n; ++i) {
    durations[i] = total_time * dists[i] / total_dist;
    if (durations[i] < 0.01) durations[i] = 0.01;
  }
  return durations;
}

// ---------------------------------------------------------------------------
// fitMincoQuintic: fit 5th-order polynomial pieces (2D) through waypoints
// with C2 continuity at boundaries.
//
// For each piece k over [0, T_k]:
//   p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
//
// Boundary conditions per piece (6 equations, 6 unknowns per axis):
//   p(0)   = p_start        =>  c0 = p_start
//   p'(0)  = v_start        =>  c1 = v_start
//   p''(0) = a_start        =>  2*c2 = a_start
//   p(T)   = p_end
//   p'(T)  = v_end
//   p''(T) = a_end
//
// Interior velocities are estimated via Catmull-Rom style averaging.
// Interior accelerations are set to zero (natural spline).
// ---------------------------------------------------------------------------
void TrajectoryOptimizer::fitMincoQuintic(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& durations,
    const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
    const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
    std::vector<PolyPiece>& pieces) {

  int N = static_cast<int>(positions.size()) - 1;
  if (N <= 0) {
    pieces.clear();
    return;
  }

  // Compute boundary velocities and accelerations at each waypoint.
  std::vector<Eigen::Vector2d> vels(N + 1);
  std::vector<Eigen::Vector2d> accs(N + 1);
  vels[0] = v0;
  vels[N] = vf;
  accs[0] = a0;
  accs[N] = af;

  // Interior velocities: Catmull-Rom estimate, clamped.
  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) dt_sum = 1e-6;
    vels[i] = (positions[i + 1] - positions[i - 1]) / dt_sum;
    // Clamp velocity magnitude
    double v_norm = vels[i].norm();
    double max_v = 2.0;  // match max_vel param
    if (v_norm > max_v) vels[i] *= max_v / v_norm;
  }

  // Interior accelerations: estimated from velocities for continuity.
  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) dt_sum = 1e-6;
    accs[i] = (vels[std::min(i + 1, N)] - vels[std::max(i - 1, 0)]) / dt_sum;
    // Clamp acceleration
    double a_norm = accs[i].norm();
    double max_a = 2.0;  // match max_acc param
    if (a_norm > max_a) accs[i] *= max_a / a_norm;
  }

  // Solve 6x6 system per piece, per axis.
  pieces.resize(N);
  for (int k = 0; k < N; ++k) {
    double T = durations[k];
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0,   0,     0,      0,      0,
         0, 1,   0,     0,      0,      0,
         0, 0,   2,     0,      0,      0,
         1, T,   T2,    T3,     T4,     T5,
         0, 1, 2*T,   3*T2,   4*T3,   5*T4,
         0, 0,   2,   6*T,   12*T2,  20*T3;

    // Use a single factorisation for both axes.
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 6>> qr = A.colPivHouseholderQr();

    PolyPiece piece;
    piece.duration = T;

    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Matrix<double, 6, 1> b;
      b(0) = positions[k](axis);
      b(1) = vels[k](axis);
      b(2) = accs[k](axis);
      b(3) = positions[k + 1](axis);
      b(4) = vels[k + 1](axis);
      b(5) = accs[k + 1](axis);

      Eigen::Matrix<double, 6, 1> c = qr.solve(b);
      for (int j = 0; j < 6; ++j) {
        double val = c(j);
        piece.coeffs(axis, j) = std::isfinite(val) ? val : 0.0;
      }
    }
    pieces[k] = piece;
  }
}

// ---------------------------------------------------------------------------
// fitYawQuintic: 1D version of fitMincoQuintic for yaw channel.
// Angle differences are normalised to [-pi, pi].
// ---------------------------------------------------------------------------
void TrajectoryOptimizer::fitYawQuintic(
    const std::vector<double>& yaws,
    const std::vector<double>& durations,
    double omega0, double omegaf,
    std::vector<YawPolyPiece>& pieces) {

  int N = static_cast<int>(yaws.size()) - 1;
  if (N <= 0) {
    pieces.clear();
    return;
  }

  std::vector<double> omegas(N + 1, 0.0);
  std::vector<double> alphas(N + 1, 0.0);
  omegas[0] = omega0;
  omegas[N] = omegaf;

  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-12) dt_sum = 1e-12;
    double dy = normalizeAngle(yaws[i + 1] - yaws[i - 1]);
    omegas[i] = dy / dt_sum;
    alphas[i] = 0.0;
  }

  pieces.resize(N);
  for (int k = 0; k < N; ++k) {
    double T = durations[k];
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0,   0,     0,      0,      0,
         0, 1,   0,     0,      0,      0,
         0, 0,   2,     0,      0,      0,
         1, T,   T2,    T3,     T4,     T5,
         0, 1, 2*T,   3*T2,   4*T3,   5*T4,
         0, 0,   2,   6*T,   12*T2,  20*T3;

    Eigen::Matrix<double, 6, 1> b;
    b(0) = yaws[k];
    b(1) = omegas[k];
    b(2) = alphas[k];
    // Use unwrapped target so the polynomial doesn't wrap around.
    b(3) = yaws[k] + normalizeAngle(yaws[k + 1] - yaws[k]);
    b(4) = omegas[k + 1];
    b(5) = alphas[k + 1];

    Eigen::Matrix<double, 6, 1> c = A.colPivHouseholderQr().solve(b);

    YawPolyPiece yp;
    yp.duration = T;
    for (int j = 0; j < 6; ++j) {
      yp.coeffs(0, j) = c(j);
    }
    pieces[k] = yp;
  }
}

// ---------------------------------------------------------------------------
// optimizeSE2 — Eq.(3): gradient descent on interior waypoint positions and
// yaws.  Cost = smoothness + safety (SVSDF penalty) + dynamics penalty.
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::optimizeSE2(
    const std::vector<SE2State>& waypoints, double total_time) {

  int n_wps = static_cast<int>(waypoints.size());
  if (n_wps < 2) return Trajectory();

  std::vector<SE2State> wps = waypoints;
  std::vector<double> durations = allocateTime(wps, total_time);
  double step = params_.step_size;
  const double chain_clearance = continuousWaypointChainClearance(
      waypoints, svsdf_, map_ ? 0.5 * map_->resolution() : 0.05);
  if (svsdf_ && chain_clearance < 0.0) {
    Trajectory conservative = buildRotateTranslateTrajectory(waypoints, total_time);
    if (!conservative.empty() &&
        svsdf_->evaluateTrajectory(conservative, 0.05) >= 0.0) {
      return conservative;
    }
  }

  if (svsdf_ && map_ && n_wps > 2 && chain_clearance < 0.0) {
    const double sample_step = 0.5 * map_->resolution();
    for (int pass = 0; pass < 8; ++pass) {
      bool changed = false;
      bool all_safe = true;
      for (int i = 1; i < n_wps - 1; ++i) {
        const double state_clearance = svsdf_->evaluate(wps[i]);
        const double left_clearance =
            transitionClearance(wps[i - 1], wps[i], svsdf_, sample_step);
        const double right_clearance =
            transitionClearance(wps[i], wps[i + 1], svsdf_, sample_step);
        const double min_clearance =
            std::min(state_clearance, std::min(left_clearance, right_clearance));
        if (min_clearance >= 0.0) {
          continue;
        }

        all_safe = false;
        Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
        double grad_yaw = 0.0;
        svsdf_->gradient(wps[i], grad_pos, grad_yaw);
        if (grad_pos.norm() < 1e-6) {
          grad_pos = wps[i].position() -
                     0.5 * (wps[i - 1].position() + wps[i + 1].position());
        }
        if (grad_pos.norm() < 1e-6) {
          continue;
        }

        const Eigen::Vector2d dir = grad_pos.normalized();
        const double push = std::max(map_->resolution(),
                                     -min_clearance + map_->resolution());
        wps[i].x += push * dir.x();
        wps[i].y += push * dir.y();
        wps[i].yaw = normalizeAngle(wps[i].yaw + 0.1 * grad_yaw);
        changed = true;
      }
      if (all_safe || !changed) {
        break;
      }
    }
    durations = allocateTime(wps, total_time);
  }

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(n_wps);
    std::vector<double> yaws(n_wps);
    for (int i = 0; i < n_wps; ++i) {
      positions[i] = Eigen::Vector2d(wps[i].x, wps[i].y);
      yaws[i] = wps[i].yaw;
    }

    double total_grad_norm = 0.0;

    for (int i = 1; i < n_wps - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;

      // --- Smoothness: time-aware finite-difference (paper Eq.3 J_s) ---
      double dt_prev = durations[i - 1];
      double dt_next = durations[i];
      double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      // Acceleration approximation at waypoint i
      Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      // Gradient of ||acc||^2 w.r.t. positions[i]: -4*acc / dt^2
      // Clamp to prevent blow-up
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      double gs_norm = grad_smooth.norm();
      if (gs_norm > 100.0) grad_smooth *= 100.0 / gs_norm;
      grad_pos += params_.lambda_smooth * grad_smooth;

      // Yaw smoothness
      double yaw_dd = (normalizeAngle(yaws[i + 1] - yaws[i]) -
                       normalizeAngle(yaws[i] - yaws[i - 1])) /
                      (dt_avg * dt_avg);
      double yaw_grad = params_.lambda_smooth * (-4.0 * yaw_dd / (dt_avg * dt_avg));
      if (std::abs(yaw_grad) > 100.0) yaw_grad = (yaw_grad > 0) ? 100.0 : -100.0;
      grad_yaw += yaw_grad;

      // --- Safety: SVSDF penalty when distance < safety_margin ---
      if (svsdf_) {
        double sdf_val = svsdf_->evaluate(wps[i]);
        if (sdf_val < params_.safety_margin) {
          Eigen::Vector2d gp;
          double gy;
          svsdf_->gradient(wps[i], gp, gy);
          double pen = params_.safety_margin - sdf_val;
          // Gradient of pen^2 w.r.t. state, pushing away from obstacle.
          grad_pos -= params_.lambda_safety * 2.0 * pen * gp;
          grad_yaw -= params_.lambda_safety * 2.0 * pen * gy;
        }
      }

      // --- Dynamics: penalise velocity exceeding max_vel ---
      double dt_span = dt_prev + dt_next;
      if (dt_span < 1e-12) dt_span = 1e-12;
      Eigen::Vector2d vel_est =
          (positions[i + 1] - positions[i - 1]) / dt_span;
      double vel_norm = vel_est.norm();
      if (vel_norm > params_.max_vel) {
        double excess = vel_norm - params_.max_vel;
        // d/d(p_i) of vel_est is zero (p_i cancels), but we add a soft
        // centering penalty that pulls the waypoint toward the midpoint.
        Eigen::Vector2d mid = 0.5 * (positions[i - 1] + positions[i + 1]);
        grad_pos += params_.lambda_dynamics * excess *
                    (positions[i] - mid) / (vel_norm + 1e-12);
      }

      // --- Dynamics: penalise acceleration exceeding max_acc ---
      Eigen::Vector2d acc_est = acc_approx;  // reuse from smoothness
      double acc_norm = acc_est.norm();
      if (acc_norm > params_.max_acc) {
        double excess_a = acc_norm - params_.max_acc;
        // Push waypoint to reduce curvature
        grad_pos += params_.lambda_dynamics * excess_a *
                    (-2.0 * acc_est) / (acc_norm * dt_avg * dt_avg + 1e-12);
      }

      // Clamp total gradient to prevent instability
      double total_g = grad_pos.norm();
      if (total_g > 50.0) grad_pos *= 50.0 / total_g;
      if (std::abs(grad_yaw) > 50.0) grad_yaw = (grad_yaw > 0) ? 50.0 : -50.0;

      // Apply gradient step.
      wps[i].x -= step * grad_pos.x();
      wps[i].y -= step * grad_pos.y();
      wps[i].yaw = normalizeAngle(wps[i].yaw - step * grad_yaw);

      total_grad_norm += grad_pos.squaredNorm() + grad_yaw * grad_yaw;
    }

    // Re-allocate time after position update.
    durations = allocateTime(wps, total_time);

    if (total_grad_norm < 1e-10) break;
  }

  // Downsample optimised waypoints for MINCO fitting.
  // Keep first, last, and every N-th point to avoid overfitting.
  double min_piece_len = (chain_clearance <= params_.safety_margin + 0.02) ? 0.20 : 1.5;
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(Eigen::Vector2d(wps[0].x, wps[0].y));
  final_yaws.push_back(wps[0].yaw);

  double accum_dist = 0.0;
  for (int i = 1; i < n_wps; ++i) {
    double dx = wps[i].x - wps[i - 1].x;
    double dy = wps[i].y - wps[i - 1].y;
    accum_dist += std::sqrt(dx * dx + dy * dy);
    if (accum_dist >= min_piece_len || i == n_wps - 1) {
      final_pos.push_back(Eigen::Vector2d(wps[i].x, wps[i].y));
      final_yaws.push_back(wps[i].yaw);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> ds_wps(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    ds_wps[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  std::vector<double> ds_durations = allocateTime(ds_wps, total_time);

  // Estimate boundary velocities from waypoint direction (not zero!)
  Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d vf = Eigen::Vector2d::Zero();
  if (final_pos.size() >= 2) {
    double dt0 = ds_durations.empty() ? 1.0 : ds_durations[0];
    if (dt0 < 0.1) dt0 = 0.1;
    v0 = (final_pos[1] - final_pos[0]) / dt0;
    double v0n = v0.norm();
    if (v0n > params_.max_vel) v0 *= params_.max_vel / v0n;

    double dtf = ds_durations.empty() ? 1.0 : ds_durations.back();
    if (dtf < 0.1) dtf = 0.1;
    vf = (final_pos.back() - final_pos[final_pos.size() - 2]) / dtf;
    double vfn = vf.norm();
    if (vfn > params_.max_vel) vf *= params_.max_vel / vfn;
  }

  Trajectory traj;
  fitMincoQuintic(final_pos, ds_durations,
                  v0, vf,
                  Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  traj.pos_pieces);
  fitYawQuintic(final_yaws, ds_durations, 0.0, 0.0, traj.yaw_pieces);
  traj = retimeToDynamicLimits(traj);

  if (svsdf_ && !traj.empty()) {
    const double traj_clearance = svsdf_->evaluateTrajectory(traj, 0.05);
    if (chain_clearance >= 0.0 && traj_clearance < 0.0) {
      Trajectory conservative = buildRotateTranslateTrajectory(waypoints, total_time);
      if (!conservative.empty() &&
          svsdf_->evaluateTrajectory(conservative, 0.05) >= 0.0) {
        return conservative;
      }
      return buildConservativePolylineTrajectory(waypoints, total_time);
    }
  }

  return traj;
}

// ---------------------------------------------------------------------------
// optimizeR2 — Eq.(4): gradient descent on interior waypoint positions only.
// Cost = smoothness + position residual + rotation residual.
// Yaw is derived from velocity direction after optimisation.
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::optimizeR2(
    const std::vector<SE2State>& waypoints, double total_time) {

  int n_wps = static_cast<int>(waypoints.size());
  if (n_wps < 2) return Trajectory();

  std::vector<SE2State> wps = waypoints;
  std::vector<SE2State> ref = waypoints;
  std::vector<double> durations = allocateTime(wps, total_time);
  double step = params_.step_size;
  const double chain_clearance = continuousWaypointChainClearance(
      waypoints, svsdf_, map_ ? 0.5 * map_->resolution() : 0.05);

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(n_wps);
    for (int i = 0; i < n_wps; ++i) {
      positions[i] = Eigen::Vector2d(wps[i].x, wps[i].y);
    }

    double total_grad_norm = 0.0;

    for (int i = 1; i < n_wps - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();

      double dt_prev = durations[i - 1];
      double dt_next = durations[i];
      double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      // --- Smoothness (time-aware) ---
      Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      double gs_norm = grad_smooth.norm();
      if (gs_norm > 100.0) grad_smooth *= 100.0 / gs_norm;
      grad_pos += params_.lambda_smooth * grad_smooth;

      // --- Position residual: stay near reference waypoint ---
      Eigen::Vector2d ref_pos(ref[i].x, ref[i].y);
      grad_pos += params_.lambda_pos_residual * 2.0 *
                  (positions[i] - ref_pos);

      // --- Rotation residual: heading from velocity should be smooth ---
      Eigen::Vector2d vel_dir = positions[i + 1] - positions[i - 1];
      double vel_norm = vel_dir.norm();
      if (vel_norm > 1e-9) {
        double yaw_curr = std::atan2(vel_dir.y(), vel_dir.x());

        // Heading at neighbours.
        double yaw_prev = wps[0].yaw;
        if (i >= 2) {
          Eigen::Vector2d vp = positions[i] - positions[i - 2];
          if (vp.norm() > 1e-9) yaw_prev = std::atan2(vp.y(), vp.x());
        }
        double yaw_next = wps[n_wps - 1].yaw;
        if (i + 2 < n_wps) {
          Eigen::Vector2d vn = positions[i + 2] - positions[i];
          if (vn.norm() > 1e-9) yaw_next = std::atan2(vn.y(), vn.x());
        }

        double yaw_dd = normalizeAngle(yaw_next - yaw_curr) -
                         normalizeAngle(yaw_curr - yaw_prev);

        // Gradient direction: perpendicular to velocity.
        Eigen::Vector2d perp(-vel_dir.y(), vel_dir.x());
        perp /= vel_norm;
        grad_pos += params_.lambda_yaw_residual * yaw_dd * perp;
      }

      wps[i].x -= step * grad_pos.x();
      wps[i].y -= step * grad_pos.y();

      total_grad_norm += grad_pos.squaredNorm();
    }

    durations = allocateTime(wps, total_time);
    if (total_grad_norm < 1e-10) break;
  }

  // Derive yaw from velocity direction, then downsample for MINCO.
  std::vector<Eigen::Vector2d> all_pos(n_wps);
  std::vector<double> all_yaws(n_wps);
  for (int i = 0; i < n_wps; ++i) {
    all_pos[i] = Eigen::Vector2d(wps[i].x, wps[i].y);
  }
  all_yaws[0] = wps[0].yaw;
  all_yaws[n_wps - 1] = wps[n_wps - 1].yaw;
  for (int i = 1; i < n_wps - 1; ++i) {
    Eigen::Vector2d vel_dir = all_pos[i + 1] - all_pos[i - 1];
    if (vel_dir.norm() > 1e-9) {
      all_yaws[i] = std::atan2(vel_dir.y(), vel_dir.x());
    } else {
      all_yaws[i] = all_yaws[i - 1];
    }
  }

  // Downsample: keep first, last, and points every ~0.5m
  double min_piece_len = (chain_clearance <= params_.safety_margin + 0.02) ? 0.15 : 0.5;
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(all_pos[0]);
  final_yaws.push_back(all_yaws[0]);

  double accum_dist = 0.0;
  for (int i = 1; i < n_wps; ++i) {
    accum_dist += (all_pos[i] - all_pos[i - 1]).norm();
    if (accum_dist >= min_piece_len || i == n_wps - 1) {
      final_pos.push_back(all_pos[i]);
      final_yaws.push_back(all_yaws[i]);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> ds_wps(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    ds_wps[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  std::vector<double> ds_durations = allocateTime(ds_wps, total_time);

  // Estimate boundary velocities from waypoint direction
  Eigen::Vector2d v0r = Eigen::Vector2d::Zero();
  Eigen::Vector2d vfr = Eigen::Vector2d::Zero();
  if (final_pos.size() >= 2) {
    double dt0 = ds_durations.empty() ? 1.0 : ds_durations[0];
    if (dt0 < 0.1) dt0 = 0.1;
    v0r = (final_pos[1] - final_pos[0]) / dt0;
    double v0n = v0r.norm();
    if (v0n > params_.max_vel) v0r *= params_.max_vel / v0n;

    double dtf = ds_durations.empty() ? 1.0 : ds_durations.back();
    if (dtf < 0.1) dtf = 0.1;
    vfr = (final_pos.back() - final_pos[final_pos.size() - 2]) / dtf;
    double vfn = vfr.norm();
    if (vfn > params_.max_vel) vfr *= params_.max_vel / vfn;
  }

  Trajectory traj;
  fitMincoQuintic(final_pos, ds_durations,
                  v0r, vfr,
                  Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  traj.pos_pieces);
  fitYawQuintic(final_yaws, ds_durations, 0.0, 0.0, traj.yaw_pieces);
  if (svsdf_ && !traj.empty()) {
    const double traj_clearance = svsdf_->evaluateTrajectory(traj, 0.05);
    if (chain_clearance >= 0.0 && traj_clearance < 0.0) {
      return buildConservativePolylineTrajectory(waypoints, total_time);
    }
  }
  return traj;
}

// ---------------------------------------------------------------------------
// stitch: collect all optimised waypoints and do ONE global MINCO fit
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::stitch(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& optimized) {

  // Instead of concatenating per-segment pieces (which have v0=vf=0 at
  // boundaries), we extract the waypoints from each segment's optimised
  // trajectory by sampling, then do a single global MINCO fit.

  size_t count = std::min(segments.size(), optimized.size());
  if (count == 0) return Trajectory();
  if (count == 1 && !optimized[0].empty()) {
    return retimeToDynamicLimits(optimized[0]);
  }

  // First try exact concatenation of already validated per-segment trajectories.
  // This avoids re-fitting a new global polynomial through narrow passages.
  Trajectory concatenated;
  for (size_t i = 0; i < count; ++i) {
    if (optimized[i].empty()) return Trajectory();
    concatenated.pos_pieces.insert(concatenated.pos_pieces.end(),
                                   optimized[i].pos_pieces.begin(),
                                   optimized[i].pos_pieces.end());
    concatenated.yaw_pieces.insert(concatenated.yaw_pieces.end(),
                                   optimized[i].yaw_pieces.begin(),
                                   optimized[i].yaw_pieces.end());
  }
  concatenated = retimeToDynamicLimits(concatenated);
  if (!concatenated.empty()) {
    const bool collision_free =
        !svsdf_ || svsdf_->evaluateTrajectory(concatenated, 0.05) >= 0.0;
    if (collision_free && dynamicsFeasible(concatenated)) {
      return concatenated;
    }
  }

  // Collect optimised waypoints by sampling per-segment trajectories
  std::vector<Eigen::Vector2d> all_pos;
  std::vector<double> all_yaws;

  for (size_t i = 0; i < count; ++i) {
    const Trajectory& seg_traj = optimized[i];
    if (seg_traj.empty()) continue;
    double dur = seg_traj.totalDuration();
    double sample_step = 0.15;  // match disc_step
    int n_samples = std::max(2, static_cast<int>(std::ceil(dur / sample_step)) + 1);
    for (int s = 0; s < n_samples; ++s) {
      double t = (s == n_samples - 1) ? dur : s * sample_step;
      // Skip first sample of subsequent segments (shared endpoint)
      if (i > 0 && s == 0) continue;
      SE2State st = seg_traj.sample(t);
      all_pos.push_back(Eigen::Vector2d(st.x, st.y));
      all_yaws.push_back(st.yaw);
    }
  }

  if (all_pos.size() < 2) return Trajectory();

  // Allocate time
  double total_time = 0.0;
  for (const auto& t : optimized) total_time += t.totalDuration();

  auto buildFromSpacing = [&](double min_piece_len) {
    std::vector<Eigen::Vector2d> ds_pos;
    std::vector<double> ds_yaws;
    ds_pos.push_back(all_pos[0]);
    ds_yaws.push_back(all_yaws[0]);

    double accum = 0.0;
    for (size_t i = 1; i < all_pos.size(); ++i) {
      accum += (all_pos[i] - all_pos[i - 1]).norm();
      if (accum >= min_piece_len || i == all_pos.size() - 1) {
        ds_pos.push_back(all_pos[i]);
        ds_yaws.push_back(all_yaws[i]);
        accum = 0.0;
      }
    }

    std::vector<SE2State> ds_wps(ds_pos.size());
    for (size_t i = 0; i < ds_pos.size(); ++i) {
      ds_wps[i] = SE2State(ds_pos[i].x(), ds_pos[i].y(), ds_yaws[i]);
    }
    std::vector<double> ds_durations = allocateTime(ds_wps, total_time);

    Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d vf = Eigen::Vector2d::Zero();
    if (ds_pos.size() >= 2) {
      double dt0 = ds_durations[0];
      if (dt0 < 0.1) dt0 = 0.1;
      v0 = (ds_pos[1] - ds_pos[0]) / dt0;
      double v0n = v0.norm();
      if (v0n > params_.max_vel) v0 *= params_.max_vel / v0n;

      double dtf = ds_durations.back();
      if (dtf < 0.1) dtf = 0.1;
      vf = (ds_pos.back() - ds_pos[ds_pos.size() - 2]) / dtf;
      double vfn = vf.norm();
      if (vfn > params_.max_vel) vf *= params_.max_vel / vfn;
    }

    Trajectory result;
    fitMincoQuintic(ds_pos, ds_durations, v0, vf,
                    Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                    result.pos_pieces);
    fitYawQuintic(ds_yaws, ds_durations, 0.0, 0.0, result.yaw_pieces);
    return retimeToDynamicLimits(result);
  };

  Trajectory result = buildFromSpacing(1.5);
  if (svsdf_ && !result.empty() && svsdf_->evaluateTrajectory(result, 0.05) < 0.0) {
    result = buildFromSpacing(0.45);
  }
  if (svsdf_ && !result.empty() && svsdf_->evaluateTrajectory(result, 0.05) < 0.0) {
    result = buildFromSpacing(0.20);
  }
  return result;
}

// ---------------------------------------------------------------------------
// selectBest: prefer larger clearance, then minimum integral of acceleration squared
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::selectBest(
    const std::vector<Trajectory>& candidates) {

  if (candidates.empty()) return Trajectory();

  double best_clearance = -kInf;
  double best_cost = kInf;
  size_t best_idx = 0;
  bool found = false;

  for (size_t c = 0; c < candidates.size(); ++c) {
    const Trajectory& traj = candidates[c];
    if (traj.empty()) continue;

    double min_sdf = kInf;
    if (svsdf_) {
      min_sdf = svsdf_->evaluateTrajectory(traj, 0.05);
      if (min_sdf < 0.0) continue;
    }
    if (!dynamicsFeasible(traj)) continue;

    double cost = 0.0;

    for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
      const PolyPiece& pp = traj.pos_pieces[i];
      double T = pp.duration;
      // Simpson's rule with 20 sub-intervals.
      int n_sub = 20;
      double h = T / n_sub;
      for (int s = 0; s <= n_sub; ++s) {
        double t = s * h;
        Eigen::Vector2d a = pp.acceleration(t);
        double w = (s == 0 || s == n_sub) ? 1.0 :
                   (s % 2 == 1) ? 4.0 : 2.0;
        cost += w * a.squaredNorm() * h / 3.0;
      }
    }

    const bool better_clearance = (min_sdf > best_clearance + 1e-6);
    const bool same_clearance = std::abs(min_sdf - best_clearance) <= 1e-6;
    if (!found || better_clearance || (same_clearance && cost < best_cost)) {
      best_clearance = min_sdf;
      best_cost = cost;
      best_idx = c;
      found = true;
    }
  }
  if (!found) return Trajectory();
  return candidates[best_idx];
}

// ---------------------------------------------------------------------------
// computeCost: evaluate total cost for a trajectory
// ---------------------------------------------------------------------------
double TrajectoryOptimizer::computeCost(
    const Trajectory& traj, const std::vector<SE2State>& ref_wps,
    bool include_svsdf, bool include_residual) {

  if (traj.empty()) return kInf;

  double cost = 0.0;
  double total_dur = traj.totalDuration();
  double dt_sample = 0.05;
  int n_samples = std::max(1, static_cast<int>(total_dur / dt_sample));
  double dt = total_dur / n_samples;

  for (int s = 0; s <= n_samples; ++s) {
    double t = std::min(static_cast<double>(s) * dt, total_dur);

    // Locate piece.
    double acc_time = 0.0;
    for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
      double T_i = traj.pos_pieces[i].duration;
      if (t <= acc_time + T_i || i + 1 == traj.pos_pieces.size()) {
        double lt = t - acc_time;

        // Smoothness: acceleration squared.
        Eigen::Vector2d a = traj.pos_pieces[i].acceleration(lt);
        cost += params_.lambda_smooth * a.squaredNorm() * dt;

        // Safety (SVSDF).
        if (include_svsdf && svsdf_) {
          SE2State st = traj.sample(t);
          double sdf_val = svsdf_->evaluate(st);
          if (sdf_val < params_.safety_margin) {
            double pen = params_.safety_margin - sdf_val;
            cost += params_.lambda_safety * pen * pen * dt;
          }
        }
        break;
      }
      acc_time += T_i;
    }
  }

  // Position and yaw residual costs.
  if (include_residual && !ref_wps.empty()) {
    int n_ref = static_cast<int>(ref_wps.size());
    double ref_dt = (n_ref > 1) ? total_dur / (n_ref - 1) : 0.0;
    for (int i = 0; i < n_ref; ++i) {
      double t_ref = i * ref_dt;
      SE2State sampled = traj.sample(t_ref);
      double dx = sampled.x - ref_wps[i].x;
      double dy = sampled.y - ref_wps[i].y;
      cost += params_.lambda_pos_residual * (dx * dx + dy * dy);

      double dyaw = normalizeAngle(sampled.yaw - ref_wps[i].yaw);
      cost += params_.lambda_yaw_residual * dyaw * dyaw;
    }
  }

  // Time cost.
  cost += params_.lambda_time * total_dur;

  return cost;
}

Trajectory TrajectoryOptimizer::scaleTrajectoryTime(const Trajectory& traj,
                                                    double scale) const {
  if (traj.empty()) return traj;
  if (scale <= 1e-9) return traj;
  if (std::abs(scale - 1.0) < 1e-9) return traj;

  Trajectory scaled = traj;
  for (auto& pp : scaled.pos_pieces) {
    double scale_pow = scale;
    for (int i = 1; i < 6; ++i) {
      pp.coeffs.col(i) /= scale_pow;
      scale_pow *= scale;
    }
    pp.duration *= scale;
  }

  for (auto& yp : scaled.yaw_pieces) {
    double scale_pow = scale;
    for (int i = 1; i < 6; ++i) {
      yp.coeffs(0, i) /= scale_pow;
      scale_pow *= scale;
    }
    yp.duration *= scale;
  }

  return scaled;
}

Trajectory TrajectoryOptimizer::retimeToDynamicLimits(const Trajectory& traj) const {
  if (traj.empty()) return traj;

  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;
  checkDynamics(traj, &max_vel, &max_acc, &max_yaw_rate);

  double vel_scale = 1.0;
  double acc_scale = 1.0;
  double yaw_scale = 1.0;
  if (params_.max_vel > 1e-9) {
    vel_scale = max_vel / params_.max_vel;
  }
  if (params_.max_acc > 1e-9) {
    acc_scale = std::sqrt(std::max(0.0, max_acc) / params_.max_acc);
  }
  if (params_.max_yaw_rate > 1e-9) {
    yaw_scale = max_yaw_rate / params_.max_yaw_rate;
  }

  double scale = std::max(vel_scale, std::max(acc_scale, yaw_scale));
  scale = std::max(0.75, scale);
  scale *= 1.02;

  return scaleTrajectoryTime(traj, scale);
}

bool TrajectoryOptimizer::dynamicsFeasible(const Trajectory& traj,
                                           double* max_vel,
                                           double* max_acc,
                                           double* max_yaw_rate) const {
  return checkDynamics(traj, max_vel, max_acc, max_yaw_rate);
}

// ---------------------------------------------------------------------------
// checkDynamics: verify max velocity, acceleration, and yaw-rate constraints
// ---------------------------------------------------------------------------
bool TrajectoryOptimizer::checkDynamics(const Trajectory& traj,
                                        double* out_max_vel,
                                        double* out_max_acc,
                                        double* out_max_yaw_rate) const {
  double dt_sample = 0.02;
  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;

  for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
    const PolyPiece& pp = traj.pos_pieces[i];
    double T = pp.duration;
    int n_samples = std::max(1, static_cast<int>(T / dt_sample));
    double dt = T / n_samples;

    for (int s = 0; s <= n_samples; ++s) {
      double t = s * dt;
      max_vel = std::max(max_vel, pp.velocity(t).norm());
      max_acc = std::max(max_acc, pp.acceleration(t).norm());
    }
  }

  for (size_t i = 0; i < traj.yaw_pieces.size(); ++i) {
    const YawPolyPiece& yp = traj.yaw_pieces[i];
    double T = yp.duration;
    int n_samples = std::max(1, static_cast<int>(T / dt_sample));
    double dt = T / n_samples;

    for (int s = 0; s <= n_samples; ++s) {
      double t = s * dt;
      max_yaw_rate = std::max(max_yaw_rate, std::abs(yp.velocity(t)));
    }
  }

  if (out_max_vel) *out_max_vel = max_vel;
  if (out_max_acc) *out_max_acc = max_acc;
  if (out_max_yaw_rate) *out_max_yaw_rate = max_yaw_rate;

  return max_vel <= params_.max_vel * 1.10 &&
         max_acc <= params_.max_acc * 1.10 &&
         max_yaw_rate <= params_.max_yaw_rate * 1.10;
}

Trajectory TrajectoryOptimizer::buildConservativePolylineTrajectory(
    const std::vector<SE2State>& waypoints,
    double total_time) const {
  if (waypoints.size() < 2) return Trajectory();

  std::vector<double> durations = allocateTime(waypoints, total_time);
  if (durations.size() + 1 != waypoints.size()) return Trajectory();

  Trajectory traj;
  traj.pos_pieces.reserve(durations.size());
  traj.yaw_pieces.reserve(durations.size());

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double T = std::max(0.05, durations[i]);
    const Eigen::Vector2d delta = b.position() - a.position();

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = a.position();
    pp.coeffs.col(1).setZero();
    pp.coeffs.col(2).setZero();
    pp.coeffs.col(3) = delta * (10.0 / (T * T * T));
    pp.coeffs.col(4) = delta * (-15.0 / (T * T * T * T));
    pp.coeffs.col(5) = delta * (6.0 / (T * T * T * T * T));
    traj.pos_pieces.push_back(pp);

    const double yaw_delta = normalizeAngle(b.yaw - a.yaw);
    YawPolyPiece yp;
    yp.duration = T;
    yp.coeffs(0, 0) = a.yaw;
    yp.coeffs(0, 1) = 0.0;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 10.0 * yaw_delta / (T * T * T);
    yp.coeffs(0, 4) = -15.0 * yaw_delta / (T * T * T * T);
    yp.coeffs(0, 5) = 6.0 * yaw_delta / (T * T * T * T * T);
    traj.yaw_pieces.push_back(yp);
  }

  return retimeToDynamicLimits(traj);
}

Trajectory TrajectoryOptimizer::buildRotateTranslateTrajectory(
    const std::vector<SE2State>& waypoints,
    double total_time) const {
  if (waypoints.size() < 2) return Trajectory();

  struct Primitive {
    Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p1 = Eigen::Vector2d::Zero();
    double yaw0 = 0.0;
    double yaw1 = 0.0;
    double duration = 0.1;
    bool translate = false;
  };

  std::vector<Primitive> prims;
  prims.reserve(waypoints.size() * 3);
  const double pos_dt = std::max(0.10, params_.max_vel);
  const double yaw_dt = std::max(0.10, params_.max_yaw_rate);
  const double linear_sample = map_ ? 0.5 * map_->resolution() : 0.05;
  const double yaw_sample = 0.1;

  double current_yaw = waypoints.front().yaw;
  double nominal_total = 0.0;

  auto appendRotation = [&](const Eigen::Vector2d& pos, double from_yaw, double to_yaw) {
    const double delta = std::abs(normalizeAngle(to_yaw - from_yaw));
    if (delta < 1e-6) return;
    Primitive prim;
    prim.p0 = pos;
    prim.p1 = pos;
    prim.yaw0 = from_yaw;
    prim.yaw1 = to_yaw;
    prim.duration = std::max(0.05, delta / yaw_dt);
    prim.translate = false;
    nominal_total += prim.duration;
    prims.push_back(prim);
  };

  auto appendTranslation = [&](const Eigen::Vector2d& from,
                               const Eigen::Vector2d& to,
                               double yaw) {
    const double dist = (to - from).norm();
    if (dist < 1e-9) return;
    Primitive prim;
    prim.p0 = from;
    prim.p1 = to;
    prim.yaw0 = yaw;
    prim.yaw1 = yaw;
    prim.duration = std::max(0.05, dist / pos_dt);
    prim.translate = true;
    nominal_total += prim.duration;
    prims.push_back(prim);
  };

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];

    const double clearance_a = constantYawTranslationClearance(
        a, b, a.yaw, svsdf_, linear_sample);
    const double clearance_b = constantYawTranslationClearance(
        a, b, b.yaw, svsdf_, linear_sample);
    const double trans_yaw = (clearance_b > clearance_a) ? b.yaw : a.yaw;

    if (inPlaceRotationClearance(a.position(), current_yaw, trans_yaw,
                                 svsdf_, yaw_sample) < 0.0) {
      return Trajectory();
    }
    appendRotation(a.position(), current_yaw, trans_yaw);
    appendTranslation(a.position(), b.position(), trans_yaw);

    if (inPlaceRotationClearance(b.position(), trans_yaw, b.yaw,
                                 svsdf_, yaw_sample) < 0.0) {
      return Trajectory();
    }
    appendRotation(b.position(), trans_yaw, b.yaw);
    current_yaw = b.yaw;
  }

  if (prims.empty()) return Trajectory();

  const double scale = (nominal_total > 1e-6) ? total_time / nominal_total : 1.0;
  Trajectory traj;
  traj.pos_pieces.reserve(prims.size());
  traj.yaw_pieces.reserve(prims.size());

  for (const Primitive& prim : prims) {
    const double T = std::max(0.05, prim.duration * scale);

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = prim.p0;
    pp.coeffs.col(1).setZero();
    pp.coeffs.col(2).setZero();
    if (prim.translate) {
      const Eigen::Vector2d delta = prim.p1 - prim.p0;
      pp.coeffs.col(3) = delta * (10.0 / (T * T * T));
      pp.coeffs.col(4) = delta * (-15.0 / (T * T * T * T));
      pp.coeffs.col(5) = delta * (6.0 / (T * T * T * T * T));
    } else {
      pp.coeffs.col(3).setZero();
      pp.coeffs.col(4).setZero();
      pp.coeffs.col(5).setZero();
    }
    traj.pos_pieces.push_back(pp);

    YawPolyPiece yp;
    yp.duration = T;
    const double yaw_delta = normalizeAngle(prim.yaw1 - prim.yaw0);
    yp.coeffs(0, 0) = prim.yaw0;
    yp.coeffs(0, 1) = 0.0;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 10.0 * yaw_delta / (T * T * T);
    yp.coeffs(0, 4) = -15.0 * yaw_delta / (T * T * T * T);
    yp.coeffs(0, 5) = 6.0 * yaw_delta / (T * T * T * T * T);
    traj.yaw_pieces.push_back(yp);
  }

  traj = retimeToDynamicLimits(traj);
  if (svsdf_ && !traj.empty() && svsdf_->evaluateTrajectory(traj, 0.05) < 0.0) {
    return Trajectory();
  }
  return traj;
}

}  // namespace esv_planner
