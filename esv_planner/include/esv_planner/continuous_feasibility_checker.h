#pragma once

#include <esv_planner/common.h>

#include <algorithm>
#include <vector>

namespace esv_planner {

class ContinuousFeasibilityChecker {
public:
  virtual ~ContinuousFeasibilityChecker() = default;

  virtual double requiredClearance() const = 0;
  virtual double stateClearance(const SE2State& state) const = 0;
  virtual double transitionClearance(const SE2State& from,
                                     const SE2State& to) const = 0;
  virtual bool safeYaw(SE2State& state, double desired_yaw) const = 0;

  virtual bool transitionFeasible(const SE2State& from,
                                  const SE2State& to) const {
    return transitionClearance(from, to) >= requiredClearance();
  }

  virtual double segmentClearance(
      const std::vector<SE2State>& waypoints) const {
    if (waypoints.size() < 2) return -kInf;
    double min_clearance = kInf;
    for (const auto& state : waypoints) {
      min_clearance = std::min(min_clearance, stateClearance(state));
    }
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      min_clearance = std::min(
          min_clearance, transitionClearance(waypoints[i], waypoints[i + 1]));
    }
    return min_clearance;
  }

  virtual bool segmentFeasible(
      const std::vector<SE2State>& waypoints) const {
    return segmentClearance(waypoints) >= requiredClearance();
  }
};

}  // namespace esv_planner
