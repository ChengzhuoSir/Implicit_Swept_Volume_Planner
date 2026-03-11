#pragma once

#include <esv_planner/common.h>
#include <Eigen/Dense>

namespace esv_planner {

class ContinuousCollisionEvaluator {
public:
  virtual ~ContinuousCollisionEvaluator() = default;

  virtual double evaluate(const SE2State& state) const = 0;
  virtual double evaluateTrajectory(const Trajectory& traj) const = 0;
  virtual void gradient(const SE2State& state,
                        Eigen::Vector2d& grad_pos,
                        double& grad_yaw) const = 0;
};

}  // namespace esv_planner
