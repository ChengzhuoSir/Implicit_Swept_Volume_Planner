#include "esv_planner/unified_continuous_evaluator.h"

namespace esv_planner {

void UnifiedContinuousEvaluator::initGridEsdf(const GridMap& map,
                                              const FootprintModel& footprint) {
  grid_esdf_evaluator_.init(map, footprint);
}

double UnifiedContinuousEvaluator::evaluate(const SE2State& state) const {
  return grid_esdf_evaluator_.evaluate(state);
}

double UnifiedContinuousEvaluator::evaluateTrajectory(const Trajectory& traj) const {
  return grid_esdf_evaluator_.evaluateTrajectory(traj);
}

void UnifiedContinuousEvaluator::gradient(const SE2State& state,
                                          Eigen::Vector2d& grad_pos,
                                          double& grad_yaw) const {
  grid_esdf_evaluator_.gradient(state, grad_pos, grad_yaw);
}

}  // namespace esv_planner
