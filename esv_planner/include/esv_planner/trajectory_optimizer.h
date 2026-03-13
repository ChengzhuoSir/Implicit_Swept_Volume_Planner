#pragma once

#include <esv_planner/common.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/grid_map.h>
#include <vector>

namespace esv_planner {

struct OptimizerParams {
  int max_iterations = 100;
  double lambda_smooth = 1.0;
  double lambda_time = 1.0;
  double lambda_safety = 50.0;
  double lambda_dynamics = 1.0;
  double lambda_pos_residual = 5.0;
  double lambda_yaw_residual = 2.0;
  double max_vel = 1.0;
  double max_acc = 2.0;
  double max_yaw_rate = 1.5;
  double safety_margin = 0.1;
  // LBFGSpp parameters
  int lbfgs_m = 6;
  double lbfgs_epsilon = 1e-5;
};

struct OptimizerResult {
  bool success = false;
  Trajectory traj;
  double min_svsdf = -kInf;
  bool dynamics_ok = false;
  double cost = kInf;
};

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer() = default;

  void init(const GridMap& map, const SvsdfEvaluator& svsdf,
            const OptimizerParams& params);

  // Optimize SE(2) sub-problem (high-risk segments) — Eq. (3)
  OptimizerResult optimizeSE2(const std::vector<SE2State>& waypoints,
                              double total_time);

  // Optimize R² sub-problem (low-risk segments) — Eq. (4)
  OptimizerResult optimizeR2(const std::vector<SE2State>& waypoints,
                             double total_time);

  // Stitch SE(2) and R² trajectory segments into a full trajectory
  Trajectory stitch(const std::vector<MotionSegment>& segments,
                    const std::vector<Trajectory>& optimized);

  // Select the best collision-free candidate
  Trajectory selectBest(const std::vector<Trajectory>& candidates);

  // Uniformly retime a trajectory to satisfy dynamics limits
  Trajectory retimeToDynamicLimits(const Trajectory& traj) const;

  // Check velocity / acceleration / yaw-rate feasibility
  bool dynamicsFeasible(const Trajectory& traj,
                        double* max_vel = nullptr,
                        double* max_acc = nullptr,
                        double* max_yaw_rate = nullptr) const;

private:
  const GridMap* map_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  OptimizerParams params_;

  std::vector<double> allocateTime(const std::vector<SE2State>& wps,
                                   double total_time) const;
  double computeCost(const Trajectory& traj,
                     const std::vector<SE2State>& ref_wps,
                     bool include_svsdf, bool include_residual);
  Trajectory scaleTrajectoryTime(const Trajectory& traj, double scale) const;
  bool checkDynamics(const Trajectory& traj,
                     double* max_vel = nullptr,
                     double* max_acc = nullptr,
                     double* max_yaw_rate = nullptr) const;
};

}  // namespace esv_planner
