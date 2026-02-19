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
  double max_vel = 1.0;
  double max_acc = 2.0;
};

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer();

  void init(const GridMap& map, const SvsdfEvaluator& svsdf,
            const OptimizerParams& params);

  // Optimize SE(2) sub-problem (high-risk segments) — Eq. (3)
  Trajectory optimizeSE2(const std::vector<SE2State>& waypoints, double total_time);

  // Optimize R² sub-problem (low-risk segments) — Eq. (4)
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

  // MINCO helpers
  void initMincoFromWaypoints(const std::vector<SE2State>& wps,
                               double total_time,
                               std::vector<PolyPiece>& pieces);
};

}  // namespace esv_planner
