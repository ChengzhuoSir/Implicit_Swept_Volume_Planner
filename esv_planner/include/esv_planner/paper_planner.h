#pragma once

#include <esv_planner/common.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/grid_obstacle_source.h>
#include <esv_planner/local_obstacle_cache.h>
#include <esv_planner/paper_backend.h>
#include <esv_planner/path_sparsifier.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/swept_astar.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct PaperPlannerParams {
  double front_end_clearance = 0.35;
  double line_of_sight_clearance = 0.20;
  double local_aabb_half_extent = 0.9;
  double max_segment_length = 2.0;
  int max_support_points = 12;
  PaperBackendParams backend;
};

struct PaperPlannerResult {
  bool success = false;
  Trajectory trajectory;
  std::vector<Eigen::Vector2d> coarse_path;
  std::vector<SE2State> support_states;
  std::vector<Eigen::Vector2d> local_obstacles;
  PlanningStats stats;
};

class PaperPlanner {
public:
  PaperPlanner() = default;

  void init(const GridMap& map,
            const FootprintModel& footprint,
            const SvsdfEvaluator& svsdf,
            const PaperPlannerParams& params);

  PaperPlannerResult plan(const SE2State& start,
                          const SE2State& goal) const;

private:
  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  PaperPlannerParams params_;
  GridObstacleSource obstacle_source_;
  SweptAstar astar_;
  PathSparsifier sparsifier_;
  PaperBackend backend_;
};

}  // namespace esv_planner
