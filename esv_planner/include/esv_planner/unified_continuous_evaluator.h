#pragma once

#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/continuous_svsdf_evaluator.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/geometry_map.h>
#include <esv_planner/grid_map.h>

namespace esv_planner {

class UnifiedContinuousEvaluator : public ContinuousCollisionEvaluator {
public:
  UnifiedContinuousEvaluator() = default;

  enum class BackendMode {
    GridEsdf,
    GeometryBodyFrame,
  };

  void initGridEsdf(const GridMap& map, const FootprintModel& footprint);
  void initGeometryBodyFrame(const GeometryMap& geometry_map,
                             const FootprintModel& footprint);

  double evaluate(const SE2State& state) const override;
  double evaluateTrajectory(const Trajectory& traj) const override;
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos,
                double& grad_yaw) const override;

  BackendMode mode() const { return mode_; }

private:
  double evaluateGeometryState(const SE2State& state) const;
  double evaluateGeometryTrajectory(const Trajectory& traj) const;

  BackendMode mode_ = BackendMode::GridEsdf;
  ContinuousSvsdfEvaluator grid_esdf_evaluator_;
  const GeometryMap* geometry_map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
};

}  // namespace esv_planner
