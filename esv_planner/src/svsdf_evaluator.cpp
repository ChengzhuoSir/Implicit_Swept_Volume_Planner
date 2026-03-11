#include "esv_planner/svsdf_evaluator.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace esv_planner {

namespace {

double adaptiveTrajectoryClearance(const Trajectory& traj,
                                   const SvsdfEvaluator& evaluator,
                                   double t0,
                                   double t1,
                                   int depth,
                                   int max_depth,
                                   double max_linear_step,
                                   double max_yaw_step) {
  const SE2State s0 = traj.sample(t0);
  const SE2State s1 = traj.sample(t1);
  const double c0 = evaluator.evaluate(s0);
  const double c1 = evaluator.evaluate(s1);
  double min_clearance = std::min(c0, c1);

  if (depth >= max_depth) {
    return min_clearance;
  }

  const double linear_delta = (s1.position() - s0.position()).norm();
  const double yaw_delta = std::abs(normalizeAngle(s1.yaw - s0.yaw));
  const double tm = 0.5 * (t0 + t1);
  const SE2State sm = traj.sample(tm);
  const double cm = evaluator.evaluate(sm);
  min_clearance = std::min(min_clearance, cm);

  if (linear_delta <= max_linear_step && yaw_delta <= max_yaw_step) {
    return min_clearance;
  }

  min_clearance = std::min(min_clearance,
                           adaptiveTrajectoryClearance(traj, evaluator, t0, tm,
                                                       depth + 1, max_depth,
                                                       max_linear_step,
                                                       max_yaw_step));
  min_clearance = std::min(min_clearance,
                           adaptiveTrajectoryClearance(traj, evaluator, tm, t1,
                                                       depth + 1, max_depth,
                                                       max_linear_step,
                                                       max_yaw_step));
  return min_clearance;
}

}  // namespace

SvsdfEvaluator::SvsdfEvaluator() {}

void SvsdfEvaluator::init(const GridMap& map, const FootprintModel& footprint) {
  map_ = &map;
  footprint_ = &footprint;
}

// Ray-casting point-in-polygon test.
// Returns true if point (px, py) lies inside the polygon defined by verts.
static bool pointInPolygon(double px, double py,
                           const std::vector<Eigen::Vector2d>& verts) {
  bool inside = false;
  int n = static_cast<int>(verts.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double yi = verts[i].y(), yj = verts[j].y();
    double xi = verts[i].x(), xj = verts[j].x();
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

double SvsdfEvaluator::evaluate(const SE2State& state) const {
  const double res = map_->resolution();
  const auto& samples = footprint_->denseBodySamples(res, 2.0 * res);
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  double min_dist = kInf;
  for (const auto& sample : samples) {
    const double wx = state.x + c * sample.x() - s * sample.y();
    const double wy = state.y + s * sample.x() + c * sample.y();
    min_dist = std::min(min_dist, map_->getEsdf(wx, wy));
  }

  return min_dist;
}

double SvsdfEvaluator::evaluateTrajectory(const Trajectory& traj, double dt) const {
  double min_dist = kInf;
  double total = traj.totalDuration();

  // Collect time samples at regular dt intervals
  std::vector<double> times;
  times.reserve(static_cast<size_t>(total / dt) + 2);
  for (double t = 0.0; t <= total; t += dt) {
    times.push_back(t);
  }
  // Ensure the final time is included
  if (times.empty() || times.back() < total - 1e-9) {
    times.push_back(total);
  }

  // Evaluate at each sample and also at midpoints between consecutive samples
  // to catch fast-moving collisions
  for (size_t i = 0; i < times.size(); ++i) {
    SE2State st = traj.sample(times[i]);
    double d = evaluate(st);
    min_dist = std::min(min_dist, d);

    if (i + 1 < times.size()) {
      double t_mid = 0.5 * (times[i] + times[i + 1]);
      SE2State st_mid = traj.sample(t_mid);
      double d_mid = evaluate(st_mid);
      min_dist = std::min(min_dist, d_mid);
    }
  }

  return min_dist;
}

double SvsdfEvaluator::evaluateTrajectory(const Trajectory& traj) const {
  if (traj.empty()) {
    return -kInf;
  }

  const double total = traj.totalDuration();
  if (total <= 1e-9) {
    return evaluate(traj.sample(0.0));
  }

  const double max_linear_step = std::max(map_->resolution(), 1e-3);
  const double max_yaw_step = 0.15;
  const int max_depth = 12;
  return adaptiveTrajectoryClearance(traj, *this, 0.0, total, 0, max_depth,
                                     max_linear_step, max_yaw_step);
}

void SvsdfEvaluator::gradient(const SE2State& state,
                               Eigen::Vector2d& grad_pos, double& grad_yaw) const {
  double eps_pos = map_->resolution() * 0.25;
  double eps_yaw = 0.005;

  // Central finite differences for position gradient
  SE2State sx_plus = state;  sx_plus.x += eps_pos;
  SE2State sx_minus = state; sx_minus.x -= eps_pos;
  grad_pos.x() = (evaluate(sx_plus) - evaluate(sx_minus)) / (2.0 * eps_pos);

  SE2State sy_plus = state;  sy_plus.y += eps_pos;
  SE2State sy_minus = state; sy_minus.y -= eps_pos;
  grad_pos.y() = (evaluate(sy_plus) - evaluate(sy_minus)) / (2.0 * eps_pos);

  // Central finite difference for yaw gradient
  SE2State syaw_plus = state;  syaw_plus.yaw += eps_yaw;
  SE2State syaw_minus = state; syaw_minus.yaw -= eps_yaw;
  grad_yaw = (evaluate(syaw_plus) - evaluate(syaw_minus)) / (2.0 * eps_yaw);
}

}  // namespace esv_planner
