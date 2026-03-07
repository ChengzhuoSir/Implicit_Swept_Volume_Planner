#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <cstdint>
#include <vector>

namespace esv_planner {

struct HybridAStarParams {
  double step_size = 0.35;
  double wheel_base = 0.8;
  double max_steer = 0.6;
  int steer_samples = 5;  // odd number, includes zero

  double goal_tolerance_pos = 0.35;
  double goal_tolerance_yaw = 0.4;

  double reverse_penalty = 2.0;
  double steer_penalty = 0.2;
  double steer_change_penalty = 0.2;
  double switch_penalty = 2.0;

  int max_expansions = 50000;

  // Fast-first search: improves average latency, fallback still uses full budget.
  int fast_pass_max_expansions = 8000;
  int fast_pass_steer_samples = 3;

  // Start/goal local recovery when exact pose has no safe yaw.
  double pose_recovery_max_radius = 0.6;
  double pose_recovery_radial_step = 0.05;
  int pose_recovery_angular_samples = 24;
};

class HybridAStarPlanner {
public:
  HybridAStarPlanner();

  void init(const GridMap& map, const CollisionChecker& checker,
            const HybridAStarParams& params);

  // Traditional hybrid A*: SE(2) node search with kinematic motion primitives.
  bool plan(const SE2State& start, const SE2State& goal, std::vector<SE2State>& path);

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  HybridAStarParams params_;

  bool inBoundsAndFree(const SE2State& s) const;
  double heuristic(const SE2State& a, const SE2State& b) const;
  uint64_t keyOf(const SE2State& s) const;
};

}  // namespace esv_planner
