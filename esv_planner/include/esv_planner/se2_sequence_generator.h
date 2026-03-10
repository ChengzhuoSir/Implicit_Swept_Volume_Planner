#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/svsdf_evaluator.h>
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
  SvsdfEvaluator svsdf_;
  double disc_step_ = 0.15;
  int max_push_ = 5;

  // Discretize path into position waypoints with tangent yaw
  std::vector<SE2State> discretizePath(const TopoPath& path,
                                        const SE2State& start,
                                        const SE2State& goal);

  // SafeYaw: assign collision-free yaw closest to desired
  bool safeYaw(SE2State& state, double desired_yaw);

  // Conservative footprint clearance used to keep generator semantics aligned
  // with the final continuous validation stage.
  double footprintClearance(const SE2State& state) const;

  // Required clearance used by SafeYaw and continuous edge validation.
  double requiredClearance() const;

  // Conservative clearance of the continuous interpolated motion between two
  // assigned states.
  double transitionClearance(const SE2State& from, const SE2State& to) const;

  // Validate the continuous interpolated motion between two assigned states.
  bool transitionSafe(const SE2State& from, const SE2State& to) const;

  // Try to repair a locally unsafe edge by inserting one pushed midpoint.
  bool bridgeUnsafeTransition(const SE2State& from,
                              const SE2State& to,
                              std::vector<SE2State>& bridge);

  // Push a colliding state toward larger ESDF, then re-run SafeYaw.
  bool pushStateFromObstacle(SE2State& state, double desired_yaw);

  // Local repair for a state inside a HIGH window that must remain compatible
  // with both neighbouring states under the required clearance target.
  bool repairStateBetweenNeighbors(const SE2State& prev,
                                   const SE2State& current,
                                   const SE2State& next,
                                   double desired_yaw,
                                   SE2State& repaired);

  // SegAdjust: recursively split a failing segment around a pushed pivot.
  bool segAdjustRecursive(const std::vector<SE2State>& seed,
                          int depth,
                          std::vector<SE2State>& repaired);

  // Fallback for short local failures that do not justify a full HIGH segment.
  bool repairShortWindow(const std::vector<SE2State>& seed,
                         std::vector<SE2State>& repaired);

  // Re-run local repair on fragmented HIGH/LOW windows to keep HIGH coverage
  // as local as possible and avoid tiny oscillating segments.
  bool tryRepairCombinedWindow(const std::vector<SE2State>& seed,
                               std::vector<SE2State>& repaired);

  // Deduplicate shared boundary states while concatenating segment waypoint
  // chains.
  std::vector<SE2State> appendStates(const std::vector<SE2State>& lhs,
                                     const std::vector<SE2State>& rhs) const;

  // Merge adjacent same-risk segments and compact short HIGH/LOW/HIGH
  // oscillations produced by local edge failures.
  std::vector<MotionSegment> compactSegments(
      const std::vector<MotionSegment>& segments);

  // Conservative acceptance check using the same piecewise linear state chain
  // semantics as the downstream fallback validator.
  double conservativeTrajectoryClearance(
      const std::vector<SE2State>& waypoints) const;

  // Find the smallest local sub-window inside a LOW segment whose conservative
  // chain clearance drops below the required margin.
  bool findMarginViolationWindow(const std::vector<SE2State>& waypoints,
                                 size_t& begin_idx,
                                 size_t& end_idx) const;

  // Reclassify margin-violating LOW chains into smaller local HIGH windows.
  std::vector<MotionSegment> enforceLowSegmentMargin(
      const std::vector<MotionSegment>& segments);

  // Resample a straight sub-segment for recursive SegAdjust.
  std::vector<SE2State> buildLinearSegment(const SE2State& start,
                                           const SE2State& goal) const;
};

}  // namespace esv_planner
