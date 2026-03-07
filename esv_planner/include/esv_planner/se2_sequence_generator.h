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

  // Generate SE(2) motion sequence for a given topological path (Algorithm 2)
  // Returns segments with risk levels marked
  std::vector<MotionSegment> generate(const TopoPath& path,
                                       const SE2State& start,
                                       const SE2State& goal);

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  double disc_step_ = 0.15;
  int max_push_ = 5;

  // Discretize path into position waypoints with tangent yaw
  std::vector<SE2State> discretizePath(const TopoPath& path,
                                        const SE2State& start,
                                        const SE2State& goal);

  // SafeYaw: assign collision-free yaw closest to desired
  bool safeYaw(SE2State& state, double desired_yaw);

  // Push a colliding state toward larger ESDF, then re-run SafeYaw.
  bool pushStateFromObstacle(SE2State& state, double desired_yaw);

  // SegAdjust: recursively split a failing segment around a pushed pivot.
  bool segAdjustRecursive(const std::vector<SE2State>& seed,
                          int depth,
                          std::vector<SE2State>& repaired);

  // Fallback for short local failures that do not justify a full HIGH segment.
  bool repairShortWindow(const std::vector<SE2State>& seed,
                         std::vector<SE2State>& repaired);

  // Resample a straight sub-segment for recursive SegAdjust.
  std::vector<SE2State> buildLinearSegment(const SE2State& start,
                                           const SE2State& goal) const;
};

}  // namespace esv_planner
