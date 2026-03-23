#pragma once

#include <isweep_planner/core/common.h>
#include <isweep_planner/env/grid_map.h>
#include <isweep_planner/env/footprint_model.h>
#include <isweep_planner/core/geometry_map.h>
#include <isweep_planner/env/collision_checker.h>
#include <Eigen/Dense>

namespace isweep_planner {

class SvsdfEvaluator {
public:
  enum class Backend { GridEsdf, GeometryBodyFrame };

  SvsdfEvaluator() = default;

  void initGridEsdf(const GridMap& map, const FootprintModel& footprint);
  void initGeometryBodyFrame(const GeometryMap& geometry_map,
                             const FootprintModel& footprint);

  // Core SVSDF query: minimum signed distance (negative = collision)
  double evaluate(const SE2State& state) const;

  // Continuous trajectory evaluation with adaptive interval subdivision
  double evaluateTrajectory(const Trajectory& traj) const;

  // Gradient of SVSDF w.r.t. position and yaw.
  // Uses backend-specific semi-analytic gradients with finite-difference
  // fallback near degenerate cases.
  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos, double& grad_yaw) const;

  // Transition clearance between two SE2 states
  double transitionClearance(const SE2State& from, const SE2State& to,
                             double sample_step = 0.15) const;

  // Minimum clearance along a waypoint chain
  double segmentClearance(const std::vector<SE2State>& waypoints,
                          double sample_step = 0.15) const;

  // SafeYaw: find collision-free yaw closest to desired_yaw
  bool safeYaw(SE2State& state, double desired_yaw,
               const CollisionChecker* checker) const;

  Backend backend() const { return mode_; }
  const FootprintModel& footprint() const { return *footprint_; }

private:
  Backend mode_ = Backend::GridEsdf;
  const GridMap* map_ = nullptr;
  const GeometryMap* geometry_map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;

  // GridEsdf backend
  double evaluateGridEsdf(const SE2State& state) const;

  // GeometryBodyFrame backend
  double evaluateGeometry(const SE2State& state) const;

  // Adaptive interval subdivision for trajectory evaluation
  double evaluateInterval(const Trajectory& traj,
                          double t0, double t1, int depth) const;
};

}  // namespace isweep_planner
