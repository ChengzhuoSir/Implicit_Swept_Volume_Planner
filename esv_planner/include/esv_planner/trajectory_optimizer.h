#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/svsdf_evaluator.h>
#include <vector>

namespace esv_planner {

struct OptimizerParams {
  int max_iterations = 100;
  double lambda_smooth = 1.0;
  double lambda_time = 1.0;
  double lambda_safety = 10.0;
  double lambda_dynamics = 1.0;
  double lambda_pos_residual = 5.0;   // R² position residual weight
  double lambda_yaw_residual = 2.0;   // R² rotation residual weight
  double max_vel = 1.0;
  double max_acc = 2.0;
  double max_yaw_rate = 1.5;
  double safety_margin = 0.05;        // SVSDF safety threshold (meters)
  double step_size = 0.005;           // gradient descent step size
};

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer();

  void init(const GridMap& map, const SvsdfEvaluator& svsdf,
            const OptimizerParams& params);

  // Optimize SE(2) sub-problem (high-risk segments) — Eq. (3)
  // Jointly optimizes position and yaw with SVSDF collision penalty
  Trajectory optimizeSE2(const std::vector<SE2State>& waypoints, double total_time);

  // Optimize R² sub-problem (low-risk segments) — Eq. (4)
  // Position-only optimization with position/rotation residual penalties
  Trajectory optimizeR2(const std::vector<SE2State>& waypoints, double total_time);

  // Stitch SE(2) and R² trajectory segments into a full trajectory
  Trajectory stitch(const std::vector<MotionSegment>& segments,
                    const std::vector<Trajectory>& optimized);

  // Select best candidate trajectory (minimum control cost)
  Trajectory selectBest(const std::vector<Trajectory>& candidates);

private:
  const GridMap* map_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  OptimizerParams params_;

  // MINCO: fit quintic polynomial pieces through waypoints with C² continuity
  void fitMincoQuintic(const std::vector<Eigen::Vector2d>& positions,
                       const std::vector<double>& durations,
                       const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
                       const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
                       std::vector<PolyPiece>& pieces);

  // Fit yaw quintic pieces
  void fitYawQuintic(const std::vector<double>& yaws,
                     const std::vector<double>& durations,
                     double omega0, double omegaf,
                     std::vector<YawPolyPiece>& pieces);

  // Allocate time per segment based on distance
  std::vector<double> allocateTime(const std::vector<SE2State>& wps, double total_time);

  // Compute total cost for a trajectory
  double computeCost(const Trajectory& traj, const std::vector<SE2State>& ref_wps,
                     bool include_svsdf, bool include_residual);

  // Dynamics feasibility check
  bool checkDynamics(const Trajectory& traj);
};

}  // namespace esv_planner
