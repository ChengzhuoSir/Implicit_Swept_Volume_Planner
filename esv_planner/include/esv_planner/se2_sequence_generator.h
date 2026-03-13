#pragma once
#include <esv_planner/common.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>
#include <vector>

namespace esv_planner {
class SE2SequenceGenerator {
public:
  SE2SequenceGenerator();
  void init(const GridMap& map, const CollisionChecker& checker,
            const SvsdfEvaluator& evaluator,
            double discretization_step, int max_push_attempts);
  std::vector<MotionSegment> generate(const TopoPath& path, const SE2State& start, const SE2State& goal);
private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  const SvsdfEvaluator* evaluator_ = nullptr;
  double disc_step_ = 0.15;
  int max_push_ = 5;
  std::vector<SE2State> discretizePath(const TopoPath& path, const SE2State& start, const SE2State& goal);
  bool safeYaw(SE2State& state, double desired_yaw);
  bool pushStateFromObstacle(SE2State& state, double desired_yaw);
  bool segAdjustRecursive(const std::vector<SE2State>& seed, int depth, std::vector<SE2State>& repaired);
  bool bridgeUnsafeTransition(const SE2State& from, const SE2State& to, std::vector<SE2State>& bridge);
  std::vector<SE2State> buildLinearSegment(const SE2State& start, const SE2State& goal) const;
  std::vector<MotionSegment> generateCoreSegments(const std::vector<SE2State>& states);
};
}
