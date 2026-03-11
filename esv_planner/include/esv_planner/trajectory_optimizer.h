#pragma once

#include <esv_planner/common.h>
#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/grid_map.h>
#include <vector>

namespace esv_planner {

struct OptimizerParams {
  int max_iterations = 30;
  double lambda_smooth = 1.0;
  double lambda_time = 1.0;
  double lambda_safety = 50.0;
  double lambda_dynamics = 1.0;
  double lambda_pos_residual = 5.0;   // R² position residual weight
  double lambda_yaw_residual = 2.0;   // R² rotation residual weight
  double max_vel = 1.0;
  double max_acc = 2.0;
  double max_yaw_rate = 1.5;
  double safety_margin = 0.1;        // SVSDF safety threshold (meters)
  double step_size = 0.005;           // gradient descent step size
};

enum class OptimizerSourceMode {
  UNKNOWN,
  CONTINUOUS,
  POLYLINE_GUARD,
  ROTATE_TRANSLATE_GUARD,
};

struct OptimizerSourceInfo {
  OptimizerSourceMode source_mode = OptimizerSourceMode::UNKNOWN;
  bool used_guard = true;
  bool continuous_source_ok = false;
};

struct OptimizerResult {
  bool success = false;
  Trajectory traj;
  OptimizerSourceInfo source_info;
  double min_svsdf = -kInf;
  bool dynamics_ok = false;
  double cost = kInf;
};

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer();

  void init(const GridMap& map, const ContinuousCollisionEvaluator& svsdf,
            const OptimizerParams& params);

  // Optimize SE(2) sub-problem (high-risk segments) — Eq. (3)
  // Jointly optimizes position and yaw with SVSDF collision penalty
  OptimizerResult optimizeSE2Detailed(const std::vector<SE2State>& waypoints,
                                      double total_time);
  Trajectory optimizeSE2(const std::vector<SE2State>& waypoints, double total_time);

  // Optimize R² sub-problem (low-risk segments) — Eq. (4)
  // Position-only optimization with position/rotation residual penalties
  OptimizerResult optimizeR2Detailed(const std::vector<SE2State>& waypoints,
                                     double total_time);
  Trajectory optimizeR2(const std::vector<SE2State>& waypoints, double total_time);

  // Stitch SE(2) and R² trajectory segments into a full trajectory
  Trajectory stitch(const std::vector<MotionSegment>& segments,
                    const std::vector<Trajectory>& optimized);

  // Select the best collision-free candidate, prioritizing clearance first and
  // smoothness second.
  Trajectory selectBest(const std::vector<Trajectory>& candidates);

  // Uniformly retime a trajectory to satisfy the configured dynamics limits.
  Trajectory retimeToDynamicLimits(const Trajectory& traj) const;

  // Check velocity / acceleration / yaw-rate feasibility.
  bool dynamicsFeasible(const Trajectory& traj,
                        double* max_vel = nullptr,
                        double* max_acc = nullptr,
                        double* max_yaw_rate = nullptr) const;

  // Temporary source inspection API used by regression tests while the
  // optimizer is still being refactored away from guard-based outputs.
  OptimizerSourceInfo inspectSource(const Trajectory&) const { return {}; }

private:
  const GridMap* map_ = nullptr;
  const ContinuousCollisionEvaluator* svsdf_ = nullptr;
  OptimizerParams params_;

  // MINCO: fit quintic polynomial pieces through waypoints with C² continuity
  void fitMincoQuintic(const std::vector<Eigen::Vector2d>& positions,
                       const std::vector<double>& durations,
                       const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
                       const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
                       std::vector<PolyPiece>& pieces,
                       double tangent_scale = 1.0) const;

  // Fit yaw quintic pieces
  void fitYawQuintic(const std::vector<double>& yaws,
                     const std::vector<double>& durations,
                     double omega0, double omegaf,
                     std::vector<YawPolyPiece>& pieces,
                     double tangent_scale = 1.0) const;

  // Allocate time per segment based on distance
  std::vector<double> allocateTime(const std::vector<SE2State>& wps, double total_time) const;

  // Compute total cost for a trajectory
  double computeCost(const Trajectory& traj, const std::vector<SE2State>& ref_wps,
                     bool include_svsdf, bool include_residual);

  // Uniformly scale the time parameterization of all trajectory pieces.
  Trajectory scaleTrajectoryTime(const Trajectory& traj, double scale) const;

  // Dynamics feasibility check
  bool checkDynamics(const Trajectory& traj,
                     double* max_vel = nullptr,
                     double* max_acc = nullptr,
                     double* max_yaw_rate = nullptr) const;

  // Conservative fallback that stays exactly on the discrete waypoint polyline.
  Trajectory buildConservativePolylineTrajectory(
      const std::vector<SE2State>& waypoints,
      double total_time) const;

  // Primary continuous builder for support-state optimization.
  // Each piece follows the support-state chord exactly to preserve corridor shape.
  Trajectory buildSupportStateContinuousTrajectory(
      const std::vector<SE2State>& support_states,
      double total_time) const;

  // More conservative SE(2) fallback: decouple in-place rotation from
  // translation to avoid sweeping unsafe intermediate yaws through obstacles.
  Trajectory buildRotateTranslateTrajectory(
      const std::vector<SE2State>& waypoints,
      double total_time) const;
};

}  // namespace esv_planner
