#pragma once

#include <vector>

#include "esv_planner/env/collision_checker.h"
#include "esv_planner/core/common.h"
#include "esv_planner/env/grid_map.h"

namespace esv_planner {

class SE2SequenceGenerator {
 public:
  SE2SequenceGenerator();

  void init(const GridMap& map, const CollisionChecker& checker, double discretization_step,
            int max_push_attempts);

  std::vector<MotionSegment> generate(const TopoPath& path, const SE2State& start,
                                      const SE2State& goal);

 private:
  std::vector<SE2State> discretizePath(const TopoPath& path, const SE2State& start,
                                       const SE2State& goal);
  bool safeYaw(SE2State& state, double desired_yaw);
  bool pushStateFromObstacle(SE2State& state, double desired_yaw);
  bool segAdjustRecursive(const std::vector<SE2State>& seed, int depth,
                          std::vector<SE2State>& repaired);
  std::vector<SE2State> buildLinearSegment(const SE2State& start,
                                           const SE2State& goal) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  double disc_step_ = 0.15;
  int max_push_ = 5;
};

}  // namespace esv_planner
