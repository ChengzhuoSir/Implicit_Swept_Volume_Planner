#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <Eigen/Dense>

namespace esv_planner {

class SvsdfEvaluator {
public:
  SvsdfEvaluator();

  void init(const GridMap& map, const FootprintModel& footprint);

  // Evaluate Swept Volume SDF at a trajectory sample
  // Returns the minimum signed distance (negative = collision)
  double evaluate(const SE2State& state) const;

  // Evaluate SVSDF along a trajectory segment
  double evaluateTrajectory(const Trajectory& traj, double dt) const;

  // Gradient of SVSDF w.r.t. position and yaw
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos, double& grad_yaw) const;

private:
  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
};

}  // namespace esv_planner
