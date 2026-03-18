#pragma once

#include <vector>

#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/svsdf_evaluator.h"

namespace esv_planner {

struct OptimizerParams {
  int max_iterations = 30;
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
  double step_size = 0.005;
};

class TrajectoryOptimizer {
 public:
  TrajectoryOptimizer();

  void init(const GridMap& map, const SvsdfEvaluator& svsdf,
            const OptimizerParams& params);

  Trajectory optimizeSE2(const std::vector<SE2State>& waypoints, double total_time);
  Trajectory optimizeR2(const std::vector<SE2State>& waypoints, double total_time);
  Trajectory stitch(const std::vector<MotionSegment>& segments,
                    const std::vector<Trajectory>& optimized);
  Trajectory selectBest(const std::vector<Trajectory>& candidates);

 private:
  void fitMincoQuintic(const std::vector<Eigen::Vector2d>& positions,
                       const std::vector<double>& durations,
                       const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
                       const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
                       std::vector<PolyPiece>& pieces);
  void fitYawQuintic(const std::vector<double>& yaws,
                     const std::vector<double>& durations,
                     double omega0, double omegaf,
                     std::vector<YawPolyPiece>& pieces);
  std::vector<double> allocateTime(const std::vector<SE2State>& waypoints,
                                   double total_time);
  double computeCost(const Trajectory& traj, const std::vector<SE2State>& ref_waypoints,
                     bool include_svsdf, bool include_residual);
  bool checkDynamics(const Trajectory& traj);

  const GridMap* map_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  OptimizerParams params_;
};

}  // namespace esv_planner
