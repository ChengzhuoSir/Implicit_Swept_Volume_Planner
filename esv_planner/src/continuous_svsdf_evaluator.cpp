#include "esv_planner/continuous_svsdf_evaluator.h"

#include <algorithm>
#include <cmath>

namespace esv_planner {

namespace {

constexpr int kMaxIntervalDepth = 3;
constexpr int kMaxLeafSamples = 32;

}  // namespace

ContinuousSvsdfEvaluator::ContinuousSvsdfEvaluator() {}

void ContinuousSvsdfEvaluator::init(const GridMap& map,
                                    const FootprintModel& footprint) {
  map_ = &map;
  footprint_ = &footprint;
}

double ContinuousSvsdfEvaluator::evaluate(const SE2State& state) const {
  if (!map_ || !footprint_) {
    return -kInf;
  }

  const double res = map_->resolution();
  const auto& samples = footprint_->denseBodySamples(res, 2.0 * res);
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  double min_clearance = kInf;
  for (const auto& sample : samples) {
    const double wx = state.x + c * sample.x() - s * sample.y();
    const double wy = state.y + s * sample.x() + c * sample.y();
    min_clearance = std::min(min_clearance, map_->getEsdf(wx, wy));
  }
  return min_clearance;
}

double ContinuousSvsdfEvaluator::evaluateInterval(const Trajectory& traj,
                                                  double t0,
                                                  double t1,
                                                  int depth) const {
  const SE2State s0 = traj.sample(t0);
  const SE2State s1 = traj.sample(t1);
  double min_clearance = std::min(evaluate(s0), evaluate(s1));
  const double linear_delta = (s1.position() - s0.position()).norm();
  const double yaw_delta = std::abs(normalizeAngle(s1.yaw - s0.yaw));
  const double max_linear_step = std::max(map_->resolution(), 0.05);
  const double max_yaw_step = 0.25;

  if (depth >= kMaxIntervalDepth) {
    const int leaf_steps = std::min(
        kMaxLeafSamples,
        std::max(1, static_cast<int>(std::ceil(std::max(
            linear_delta / std::max(max_linear_step, 1e-6),
            yaw_delta / std::max(max_yaw_step, 1e-6))))));
    for (int i = 1; i < leaf_steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(leaf_steps);
      const SE2State s = traj.sample(t0 + (t1 - t0) * t);
      min_clearance = std::min(min_clearance, evaluate(s));
    }
    return min_clearance;
  }

  const double tm = 0.5 * (t0 + t1);
  const SE2State sm = traj.sample(tm);
  min_clearance = std::min(min_clearance, evaluate(sm));

  if (linear_delta <= max_linear_step && yaw_delta <= max_yaw_step) {
    return min_clearance;
  }

  const double margin_band = std::max(0.10, map_->resolution());
  if (min_clearance > margin_band &&
      linear_delta <= 2.0 * max_linear_step &&
      yaw_delta <= 1.5 * max_yaw_step) {
    return min_clearance;
  }

  min_clearance = std::min(min_clearance, evaluateInterval(traj, t0, tm, depth + 1));
  min_clearance = std::min(min_clearance, evaluateInterval(traj, tm, t1, depth + 1));
  return min_clearance;
}

double ContinuousSvsdfEvaluator::evaluateTrajectory(const Trajectory& traj) const {
  if (!map_ || !footprint_ || traj.empty()) {
    return -kInf;
  }

  const double total = traj.totalDuration();
  if (total <= 1e-9) {
    return evaluate(traj.sample(0.0));
  }
  return evaluateInterval(traj, 0.0, total, 0);
}

void ContinuousSvsdfEvaluator::gradient(const SE2State& state,
                                        Eigen::Vector2d& grad_pos,
                                        double& grad_yaw) const {
  const double eps_pos = map_->resolution() * 0.25;
  const double eps_yaw = 0.005;

  SE2State sx_plus = state;
  SE2State sx_minus = state;
  sx_plus.x += eps_pos;
  sx_minus.x -= eps_pos;
  grad_pos.x() = (evaluate(sx_plus) - evaluate(sx_minus)) / (2.0 * eps_pos);

  SE2State sy_plus = state;
  SE2State sy_minus = state;
  sy_plus.y += eps_pos;
  sy_minus.y -= eps_pos;
  grad_pos.y() = (evaluate(sy_plus) - evaluate(sy_minus)) / (2.0 * eps_pos);

  SE2State syaw_plus = state;
  SE2State syaw_minus = state;
  syaw_plus.yaw += eps_yaw;
  syaw_minus.yaw -= eps_yaw;
  grad_yaw = (evaluate(syaw_plus) - evaluate(syaw_minus)) / (2.0 * eps_yaw);
}

}  // namespace esv_planner
