#pragma once

#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/continuous_svsdf_evaluator.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>

namespace esv_planner {

class UnifiedContinuousEvaluator : public ContinuousCollisionEvaluator {
public:
  UnifiedContinuousEvaluator() = default;

  void initGridEsdf(const GridMap& map, const FootprintModel& footprint);

  double evaluate(const SE2State& state) const override;
  double evaluateTrajectory(const Trajectory& traj) const override;
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos,
                double& grad_yaw) const override;

private:
  ContinuousSvsdfEvaluator grid_esdf_evaluator_;
};

}  // namespace esv_planner
