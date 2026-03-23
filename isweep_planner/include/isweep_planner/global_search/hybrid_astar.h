#pragma once

#include <cstdint>
#include <vector>

#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/core/common.h"
#include "isweep_planner/env/grid_map.h"

namespace isweep_planner {

struct HybridAStarParams {
  double step_size = 0.35;
  double wheel_base = 0.8;
  double max_steer = 0.6;
  int steer_samples = 5;

  double goal_tolerance_pos = 0.35;
  double goal_tolerance_yaw = 0.4;

  double reverse_penalty = 2.0;
  double steer_penalty = 0.2;
  double steer_change_penalty = 0.2;
  double switch_penalty = 2.0;

  int max_expansions = 50000;
  int fast_pass_max_expansions = 8000;
  int fast_pass_steer_samples = 3;

  double pose_recovery_max_radius = 0.6;
  double pose_recovery_radial_step = 0.05;
  int pose_recovery_angular_samples = 24;
};

class HybridAStarPlanner {
 public:
  HybridAStarPlanner();

  void init(const GridMap& map, const CollisionChecker& checker,
            const HybridAStarParams& params);

  bool plan(const SE2State& start, const SE2State& goal, std::vector<SE2State>& path);

 private:
  bool inBoundsAndFree(const SE2State& state) const;
  double heuristic(const SE2State& lhs, const SE2State& rhs) const;
  uint64_t keyOf(const SE2State& state) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  HybridAStarParams params_;
};

}  // namespace isweep_planner
