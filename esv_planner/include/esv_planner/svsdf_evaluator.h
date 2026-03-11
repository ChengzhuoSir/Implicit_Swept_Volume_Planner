#pragma once

#include <esv_planner/common.h>
#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <Eigen/Dense>

namespace esv_planner {

class SvsdfEvaluator : public ContinuousCollisionEvaluator {
public:
  SvsdfEvaluator();

  void init(const GridMap& map, const FootprintModel& footprint);

  // Evaluate Swept Volume SDF at a trajectory sample
  // Returns the minimum signed distance (negative = collision)
  double evaluate(const SE2State& state) const override;

  // Legacy sampled evaluation entry point.
  double evaluateTrajectory(const Trajectory& traj, double dt) const;

  // Preferred entry point for continuous collision queries. The current
  // implementation uses adaptive interval subdivision instead of fixed dt.
  double evaluateTrajectory(const Trajectory& traj) const override;

  // Gradient of SVSDF w.r.t. position and yaw
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos, double& grad_yaw) const override;

private:
  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
};

}  // namespace esv_planner
