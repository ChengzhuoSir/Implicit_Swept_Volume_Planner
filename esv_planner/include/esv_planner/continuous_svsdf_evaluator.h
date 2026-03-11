#pragma once

#include <esv_planner/common.h>
#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>

namespace esv_planner {

class ContinuousSvsdfEvaluator : public ContinuousCollisionEvaluator {
public:
  ContinuousSvsdfEvaluator();

  void init(const GridMap& map, const FootprintModel& footprint);

  double evaluate(const SE2State& state) const override;
  double evaluateTrajectory(const Trajectory& traj) const override;
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos,
                double& grad_yaw) const override;

private:
  double evaluateInterval(const Trajectory& traj,
                          double t0,
                          double t1,
                          int depth) const;

  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
};

}  // namespace esv_planner
