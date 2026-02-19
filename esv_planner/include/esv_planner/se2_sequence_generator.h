#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <vector>

namespace esv_planner {

class SE2SequenceGenerator {
public:
  SE2SequenceGenerator();

  void init(const GridMap& map, const CollisionChecker& checker,
            double discretization_step, int max_push_attempts);

  // Generate SE(2) motion sequence for a given topological path
  // Returns segments with risk levels marked
  std::vector<MotionSegment> generate(const TopoPath& path,
                                       const SE2State& start,
                                       const SE2State& goal);

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  double disc_step_ = 0.15;
  int max_push_ = 5;

  // SegAdjust: push colliding segments away from obstacles
  bool segAdjust(std::vector<SE2State>& segment);

  // Assign yaw using SafeYaw
  bool assignYaw(SE2State& state);
};

}  // namespace esv_planner
