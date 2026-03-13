#include "esv_planner/trajectory_optimizer.h"
#include "esv_planner/minco_parameterization.h"
#include <LBFGS.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace esv_planner {

namespace {

constexpr double kFdEps = 1e-6;
constexpr double kDynSampleDt = 0.02;
constexpr double kCostSampleDt = 0.05;

// ---------------------------------------------------------------------------
// SE2 cost functor for LBFGSpp (Eq.3)
// Decision variables: interior waypoint (x, y, yaw) with fixed time
// Cost = lambda_m * J_m + lambda_t * J_t + lambda_s * J_s + lambda_d * J_d
// ---------------------------------------------------------------------------
struct Se2CostFunctor {
  const std::vector<SE2State>& ref_wps;
  const std::vector<double>& durations;
  const SE2State& start;
  const SE2State& goal;
  const SvsdfEvaluator* svsdf;
  const OptimizerParams& params;
  int piece_count;

  Se2CostFunctor(const std::vector<SE2State>& ref_wps_,
                 const std::vector<double>& durations_,
                 const SE2State& start_, const SE2State& goal_,
                 const SvsdfEvaluator* svsdf_,
                 const OptimizerParams& params_, int piece_count_)
      : ref_wps(ref_wps_), durations(durations_),
        start(start_), goal(goal_), svsdf(svsdf_),
        params(params_), piece_count(piece_count_) {}

  double evalCost(const Eigen::VectorXd& x) const {
    Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
        x, durations, start, goal);
    if (traj.empty()) return 1e12;

    double cost = 0.0;
    const double total_dur = traj.totalDuration();

    // J_t: total time
    cost += params.lambda_time * total_dur;

    // J_m (smoothness) and J_s (safety) via sampling
    const int n_samples = std::max(1, static_cast<int>(total_dur / kCostSampleDt));
    const double dt = total_dur / n_samples;
    for (int s = 0; s <= n_samples; ++s) {
      const double t = std::min(static_cast<double>(s) * dt, total_dur);
      const Eigen::Vector2d acc = traj.sampleAcceleration(t);
      cost += params.lambda_smooth * acc.squaredNorm() * dt;

      if (svsdf) {
        const SE2State st = traj.sample(t);
        const double sdf_val = svsdf->evaluate(st);
        if (sdf_val < params.safety_margin) {
          const double pen = params.safety_margin - sdf_val;
          cost += params.lambda_safety * pen * pen * dt;
        }
      }
    }

    // J_d: dynamics penalty
    double max_vel = 0.0, max_acc = 0.0, max_yr = 0.0;
    const int n_dyn = std::max(1, static_cast<int>(total_dur / kDynSampleDt));
    const double ddt = total_dur / n_dyn;
    for (int s = 0; s <= n_dyn; ++s) {
      const double t = std::min(static_cast<double>(s) * ddt, total_dur);
      max_vel = std::max(max_vel, traj.sampleVelocity(t).norm());
      max_acc = std::max(max_acc, traj.sampleAcceleration(t).norm());
      max_yr = std::max(max_yr, std::abs(traj.sampleYawRate(t)));
    }
    if (max_vel > params.max_vel) {
      const double d = max_vel - params.max_vel;
      cost += params.lambda_dynamics * d * d;
    }
    if (max_acc > params.max_acc) {
      const double d = max_acc - params.max_acc;
      cost += params.lambda_dynamics * d * d;
    }
    if (max_yr > params.max_yaw_rate) {
      const double d = max_yr - params.max_yaw_rate;
      cost += params.lambda_dynamics * d * d;
    }

    return cost;
  }

  double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    const double fx = evalCost(x);
    const int n = static_cast<int>(x.size());
    grad.resize(n);
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd xp = x, xm = x;
      xp(i) += kFdEps;
      xm(i) -= kFdEps;
      grad(i) = (evalCost(xp) - evalCost(xm)) / (2.0 * kFdEps);
    }
    return fx;
  }
};

// ---------------------------------------------------------------------------
// R2 cost functor for LBFGSpp (Eq.4)
// Decision variables: interior waypoint (x, y) only; yaw derived from velocity
// Cost = lambda_m * J_m + lambda_t * J_t + lambda_p * G_p + lambda_R * G_R
// ---------------------------------------------------------------------------
struct R2CostFunctor {
  const std::vector<SE2State>& ref_wps;
  const std::vector<double>& durations;
  const SE2State& start;
  const SE2State& goal;
  const OptimizerParams& params;
  int piece_count;

  R2CostFunctor(const std::vector<SE2State>& ref_wps_,
                const std::vector<double>& durations_,
                const SE2State& start_, const SE2State& goal_,
                const OptimizerParams& params_, int piece_count_)
      : ref_wps(ref_wps_), durations(durations_),
        start(start_), goal(goal_),
        params(params_), piece_count(piece_count_) {}

  double evalCost(const Eigen::VectorXd& x) const {
    // Reconstruct full SE2 state variables: x packs only (x,y) for interior
    // waypoints. We derive yaw from velocity direction and pack into the
    // full state-variable vector expected by MincoParameterization.
    const int n_interior = piece_count - 1;
    std::vector<SE2State> wps;
    wps.reserve(piece_count + 1);
    wps.push_back(start);
    for (int i = 0; i < n_interior; ++i) {
      SE2State st;
      st.x = x(2 * i);
      st.y = x(2 * i + 1);
      st.yaw = 0.0;
      wps.push_back(st);
    }
    wps.push_back(goal);

    // Derive yaw from velocity direction for interior waypoints
    for (int i = 1; i < static_cast<int>(wps.size()) - 1; ++i) {
      const Eigen::Vector2d vel_dir =
          wps[i + 1].position() - wps[i - 1].position();
      if (vel_dir.norm() > 1e-9) {
        wps[i].yaw = std::atan2(vel_dir.y(), vel_dir.x());
      } else {
        wps[i].yaw = wps[i - 1].yaw;
      }
    }

    const Eigen::VectorXd full_vars =
        MincoParameterization::packSe2StateVariables(wps);
    Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
        full_vars, durations, start, goal);
    if (traj.empty()) return 1e12;

    double cost = 0.0;
    const double total_dur = traj.totalDuration();

    // J_t: total time
    cost += params.lambda_time * total_dur;

    // J_m: smoothness (integral of ||acceleration||^2)
    const int n_samples = std::max(1, static_cast<int>(total_dur / kCostSampleDt));
    const double dt = total_dur / n_samples;
    for (int s = 0; s <= n_samples; ++s) {
      const double t = std::min(static_cast<double>(s) * dt, total_dur);
      const Eigen::Vector2d acc = traj.sampleAcceleration(t);
      cost += params.lambda_smooth * acc.squaredNorm() * dt;
    }

    // G_p (Eq.5): position residual to reference waypoints
    if (!ref_wps.empty()) {
      const int n_ref = static_cast<int>(ref_wps.size());
      const double ref_dt = (n_ref > 1) ? total_dur / (n_ref - 1) : 0.0;
      for (int i = 0; i < n_ref; ++i) {
        const double t_ref = i * ref_dt;
        const SE2State sampled = traj.sample(t_ref);
        const double dx = sampled.x - ref_wps[i].x;
        const double dy = sampled.y - ref_wps[i].y;
        cost += params.lambda_pos_residual * (dx * dx + dy * dy);
      }
    }

    // G_R (Eq.6): rotation residual
    if (!ref_wps.empty()) {
      const int n_ref = static_cast<int>(ref_wps.size());
      const double ref_dt = (n_ref > 1) ? total_dur / (n_ref - 1) : 0.0;
      for (int i = 0; i < n_ref; ++i) {
        const double t_ref = i * ref_dt;
        const SE2State sampled = traj.sample(t_ref);
        const double dyaw = normalizeAngle(sampled.yaw - ref_wps[i].yaw);
        cost += params.lambda_yaw_residual * dyaw * dyaw;
      }
    }

    return cost;
  }

  double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    const double fx = evalCost(x);
    const int n = static_cast<int>(x.size());
    grad.resize(n);
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd xp = x, xm = x;
      xp(i) += kFdEps;
      xm(i) -= kFdEps;
      grad(i) = (evalCost(xp) - evalCost(xm)) / (2.0 * kFdEps);
    }
    return fx;
  }
};

}  // anonymous namespace

// ===========================================================================
// TrajectoryOptimizer public interface
// ===========================================================================

void TrajectoryOptimizer::init(const GridMap& map,
                               const SvsdfEvaluator& svsdf,
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
  const int n = static_cast<int>(wps.size()) - 1;
  if (n <= 0) return {};
  std::vector<double> dists(n, 0.0);
  double total_dist = 0.0;
  for (int i = 0; i < n; ++i) {
    const double dx = wps[i + 1].x - wps[i].x;
    const double dy = wps[i + 1].y - wps[i].y;
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
// optimizeSE2 (Eq.3): L-BFGS on interior (x, y, yaw) with fixed time
// ---------------------------------------------------------------------------
OptimizerResult TrajectoryOptimizer::optimizeSE2(
    const std::vector<SE2State>& waypoints, double total_time) {
  OptimizerResult result;
  if (waypoints.size() < 2) return result;

  const SE2State& start = waypoints.front();
  const SE2State& goal = waypoints.back();
  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  const std::vector<double> durations = allocateTime(waypoints, total_time);

  // Pack initial decision variables (interior x, y, yaw)
  Eigen::VectorXd x0 = MincoParameterization::packSe2StateVariables(waypoints);
  if (x0.size() == 0) {
    // Only 2 waypoints, no interior variables to optimize; build directly
    Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
        x0, durations, start, goal);
    traj = retimeToDynamicLimits(traj);
    result.success = !traj.empty();
    result.traj = traj;
    if (result.success) {
      result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(traj) : kInf;
      result.dynamics_ok = dynamicsFeasible(traj);
      result.cost = computeCost(traj, waypoints, true, false);
    }
    return result;
  }

  Se2CostFunctor functor(waypoints, durations, start, goal,
                         svsdf_, params_, piece_count);

  LBFGSpp::LBFGSParam<double> lbfgs_param;
  lbfgs_param.m = params_.lbfgs_m;
  lbfgs_param.epsilon = params_.lbfgs_epsilon;
  lbfgs_param.max_iterations = params_.max_iterations;
  LBFGSpp::LBFGSSolver<double> solver(lbfgs_param);

  double fx = 0.0;
  try {
    solver.minimize(functor, x0, fx);
  } catch (...) {
    // L-BFGS may throw on line-search failure; use best x0 found so far
  }

  Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
      x0, durations, start, goal);
  traj = retimeToDynamicLimits(traj);

  result.success = !traj.empty();
  result.traj = traj;
  if (result.success) {
    result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(traj) : kInf;
    result.dynamics_ok = dynamicsFeasible(traj);
    result.cost = computeCost(traj, waypoints, true, false);
  }
  return result;
}

// ---------------------------------------------------------------------------
// optimizeR2 (Eq.4): L-BFGS on interior (x, y) only; yaw from velocity
// ---------------------------------------------------------------------------
OptimizerResult TrajectoryOptimizer::optimizeR2(
    const std::vector<SE2State>& waypoints, double total_time) {
  OptimizerResult result;
  if (waypoints.size() < 2) return result;

  const SE2State& start = waypoints.front();
  const SE2State& goal = waypoints.back();
  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  const std::vector<double> durations = allocateTime(waypoints, total_time);
  const int n_interior = piece_count - 1;

  if (n_interior <= 0) {
    Eigen::VectorXd empty_vars;
    Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
        empty_vars, durations, start, goal);
    traj = retimeToDynamicLimits(traj);
    result.success = !traj.empty();
    result.traj = traj;
    if (result.success) {
      result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(traj) : kInf;
      result.dynamics_ok = dynamicsFeasible(traj);
      result.cost = computeCost(traj, waypoints, false, true);
    }
    return result;
  }

  // Pack initial (x, y) for interior waypoints
  Eigen::VectorXd x0(2 * n_interior);
  for (int i = 0; i < n_interior; ++i) {
    x0(2 * i) = waypoints[i + 1].x;
    x0(2 * i + 1) = waypoints[i + 1].y;
  }

  R2CostFunctor functor(waypoints, durations, start, goal, params_, piece_count);

  LBFGSpp::LBFGSParam<double> lbfgs_param;
  lbfgs_param.m = params_.lbfgs_m;
  lbfgs_param.epsilon = params_.lbfgs_epsilon;
  lbfgs_param.max_iterations = params_.max_iterations;
  LBFGSpp::LBFGSSolver<double> solver(lbfgs_param);

  double fx = 0.0;
  try {
    solver.minimize(functor, x0, fx);
  } catch (...) {}

  // Reconstruct full states with derived yaw
  std::vector<SE2State> opt_wps;
  opt_wps.reserve(piece_count + 1);
  opt_wps.push_back(start);
  for (int i = 0; i < n_interior; ++i) {
    opt_wps.push_back(SE2State(x0(2 * i), x0(2 * i + 1), 0.0));
  }
  opt_wps.push_back(goal);
  for (int i = 1; i < static_cast<int>(opt_wps.size()) - 1; ++i) {
    const Eigen::Vector2d vel_dir =
        opt_wps[i + 1].position() - opt_wps[i - 1].position();
    opt_wps[i].yaw = (vel_dir.norm() > 1e-9)
                         ? std::atan2(vel_dir.y(), vel_dir.x())
                         : opt_wps[i - 1].yaw;
  }

  const Eigen::VectorXd full_vars =
      MincoParameterization::packSe2StateVariables(opt_wps);
  Trajectory traj = MincoParameterization::buildSe2TrajectoryFixedTime(
      full_vars, durations, start, goal);
  traj = retimeToDynamicLimits(traj);

  result.success = !traj.empty();
  result.traj = traj;
  if (result.success) {
    result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(traj) : kInf;
    result.dynamics_ok = dynamicsFeasible(traj);
    result.cost = computeCost(traj, waypoints, false, true);
  }
  return result;
}

// ---------------------------------------------------------------------------
// stitch: concatenate segment trajectories
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::stitch(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& optimized) {
  Trajectory full;
  for (size_t i = 0; i < segments.size(); ++i) {
    if (i >= optimized.size() || optimized[i].empty()) return {};
    for (const auto& piece : optimized[i].pos_pieces) {
      full.pos_pieces.push_back(piece);
    }
    for (const auto& piece : optimized[i].yaw_pieces) {
      full.yaw_pieces.push_back(piece);
    }
  }
  return full;
}

// ---------------------------------------------------------------------------
// selectBest: prefer larger clearance, tiebreak on smoothness
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::selectBest(
    const std::vector<Trajectory>& candidates) {
  if (candidates.empty()) return {};
  if (candidates.size() == 1) return candidates[0];

  int best_idx = 0;
  double best_clearance = -kInf;
  double best_smoothness = kInf;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const double clearance = svsdf_ ? svsdf_->evaluateTrajectory(candidates[i]) : 0.0;
    double smoothness = 0.0;
    const double total = candidates[i].totalDuration();
    const int n = std::max(1, static_cast<int>(total / 0.05));
    const double dt = total / n;
    for (int s = 0; s <= n; ++s) {
      const double t = std::min(static_cast<double>(s) * dt, total);
      smoothness += candidates[i].sampleAcceleration(t).squaredNorm() * dt;
    }

    if (clearance > best_clearance + 1e-6 ||
        (std::abs(clearance - best_clearance) <= 1e-6 && smoothness < best_smoothness)) {
      best_idx = static_cast<int>(i);
      best_clearance = clearance;
      best_smoothness = smoothness;
    }
  }
  return candidates[best_idx];
}

// ---------------------------------------------------------------------------
// computeCost
// ---------------------------------------------------------------------------
double TrajectoryOptimizer::computeCost(
    const Trajectory& traj, const std::vector<SE2State>& ref_wps,
    bool include_svsdf, bool include_residual) {
  if (traj.empty()) return kInf;
  double cost = 0.0;
  const double total = traj.totalDuration();
  cost += params_.lambda_time * total;

  const int n = std::max(1, static_cast<int>(total / kCostSampleDt));
  const double dt = total / n;
  for (int s = 0; s <= n; ++s) {
    const double t = std::min(static_cast<double>(s) * dt, total);
    cost += params_.lambda_smooth * traj.sampleAcceleration(t).squaredNorm() * dt;

    if (include_svsdf && svsdf_) {
      const double sdf_val = svsdf_->evaluate(traj.sample(t));
      if (sdf_val < params_.safety_margin) {
        const double pen = params_.safety_margin - sdf_val;
        cost += params_.lambda_safety * pen * pen * dt;
      }
    }
  }

  if (include_residual && !ref_wps.empty()) {
    const int n_ref = static_cast<int>(ref_wps.size());
    const double ref_dt = (n_ref > 1) ? total / (n_ref - 1) : 0.0;
    for (int i = 0; i < n_ref; ++i) {
      const SE2State sampled = traj.sample(i * ref_dt);
      cost += params_.lambda_pos_residual *
              (sampled.position() - ref_wps[i].position()).squaredNorm();
      cost += params_.lambda_yaw_residual *
              std::pow(normalizeAngle(sampled.yaw - ref_wps[i].yaw), 2);
    }
  }
  return cost;
}

// ---------------------------------------------------------------------------
// Dynamics checking
// ---------------------------------------------------------------------------
bool TrajectoryOptimizer::checkDynamics(
    const Trajectory& traj, double* out_vel, double* out_acc,
    double* out_yr) const {
  if (traj.empty()) return false;
  double mv = 0.0, ma = 0.0, myr = 0.0;
  const double total = traj.totalDuration();
  const int n = std::max(1, static_cast<int>(total / kDynSampleDt));
  const double dt = total / n;
  for (int s = 0; s <= n; ++s) {
    const double t = std::min(static_cast<double>(s) * dt, total);
    mv = std::max(mv, traj.sampleVelocity(t).norm());
    ma = std::max(ma, traj.sampleAcceleration(t).norm());
    myr = std::max(myr, std::abs(traj.sampleYawRate(t)));
  }
  if (out_vel) *out_vel = mv;
  if (out_acc) *out_acc = ma;
  if (out_yr) *out_yr = myr;
  return mv <= params_.max_vel * 1.05 &&
         ma <= params_.max_acc * 1.05 &&
         myr <= params_.max_yaw_rate * 1.05;
}

bool TrajectoryOptimizer::dynamicsFeasible(
    const Trajectory& traj, double* max_vel, double* max_acc,
    double* max_yaw_rate) const {
  return checkDynamics(traj, max_vel, max_acc, max_yaw_rate);
}

// ---------------------------------------------------------------------------
// scaleTrajectoryTime
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::scaleTrajectoryTime(
    const Trajectory& traj, double scale) const {
  if (scale <= 0.0 || traj.empty()) return traj;
  Trajectory scaled = traj;
  for (auto& p : scaled.pos_pieces) p.duration *= scale;
  for (auto& p : scaled.yaw_pieces) p.duration *= scale;
  return scaled;
}

// ---------------------------------------------------------------------------
// retimeToDynamicLimits: scale time until dynamics are feasible
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::retimeToDynamicLimits(
    const Trajectory& traj) const {
  if (traj.empty()) return traj;
  if (checkDynamics(traj)) return traj;

  for (double scale : {1.25, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0}) {
    Trajectory scaled = scaleTrajectoryTime(traj, scale);
    if (checkDynamics(scaled)) return scaled;
  }
  return scaleTrajectoryTime(traj, 8.0);
}

}  // namespace esv_planner
