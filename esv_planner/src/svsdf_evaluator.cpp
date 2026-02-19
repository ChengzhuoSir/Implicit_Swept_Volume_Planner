#include "esv_planner/svsdf_evaluator.h"
#include <cmath>
#include <algorithm>

namespace esv_planner {

SvsdfEvaluator::SvsdfEvaluator() {}

void SvsdfEvaluator::init(const GridMap& map, const FootprintModel& footprint) {
  map_ = &map;
  footprint_ = &footprint;
}

double SvsdfEvaluator::evaluate(const SE2State& state) const {
  // Evaluate minimum signed distance for the robot footprint at given SE2 state
  auto rotated = footprint_->rotatedVertices(state.yaw);

  double min_dist = kInf;
  // Sample boundary points and interior
  for (const auto& v : rotated) {
    double wx = state.x + v.x();
    double wy = state.y + v.y();
    double d = map_->getEsdf(wx, wy);
    min_dist = std::min(min_dist, d);
  }

  // Also sample edge midpoints for better coverage
  int n = static_cast<int>(rotated.size());
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    Eigen::Vector2d mid = 0.5 * (rotated[i] + rotated[j]);
    double wx = state.x + mid.x();
    double wy = state.y + mid.y();
    double d = map_->getEsdf(wx, wy);
    min_dist = std::min(min_dist, d);
  }

  // Sample center
  double d_center = map_->getEsdf(state.x, state.y);
  min_dist = std::min(min_dist, d_center);

  return min_dist;
}

double SvsdfEvaluator::evaluateTrajectory(const Trajectory& traj, double dt) const {
  double min_dist = kInf;
  double total = traj.totalDuration();

  for (double t = 0.0; t <= total; t += dt) {
    SE2State st = traj.sample(t);
    double d = evaluate(st);
    min_dist = std::min(min_dist, d);
  }
  return min_dist;
}

void SvsdfEvaluator::gradient(const SE2State& state,
                               Eigen::Vector2d& grad_pos, double& grad_yaw) const {
  double eps_pos = map_->resolution() * 0.5;
  double eps_yaw = 0.01;

  double val = evaluate(state);

  // Position gradient (finite difference)
  SE2State sx_plus = state; sx_plus.x += eps_pos;
  SE2State sx_minus = state; sx_minus.x -= eps_pos;
  grad_pos.x() = (evaluate(sx_plus) - evaluate(sx_minus)) / (2.0 * eps_pos);

  SE2State sy_plus = state; sy_plus.y += eps_pos;
  SE2State sy_minus = state; sy_minus.y -= eps_pos;
  grad_pos.y() = (evaluate(sy_plus) - evaluate(sy_minus)) / (2.0 * eps_pos);

  // Yaw gradient
  SE2State syaw_plus = state; syaw_plus.yaw += eps_yaw;
  SE2State syaw_minus = state; syaw_minus.yaw -= eps_yaw;
  grad_yaw = (evaluate(syaw_plus) - evaluate(syaw_minus)) / (2.0 * eps_yaw);
}

}  // namespace esv_planner
