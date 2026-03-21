#pragma once

#include "esv_planner/framework/svsdf_runtime.h"
#include <ros/ros.h>

namespace esv_planner {

class RecoveryManager {
 public:
  RecoveryManager(SvsdfRuntime* runtime) : runtime_(runtime) {}

  void MaybeImproveTailTrajectory(const std::vector<MotionSegment>& segments,
                                  const std::vector<Trajectory>& segment_trajectories,
                                  const ros::WallTime& deadline,
                                  SvsdfRuntime::CandidateResult* result);

  bool TryRecoverBottleneck(const std::vector<MotionSegment>& segments,
                            const std::vector<Trajectory>& segment_trajectories,
                            const Trajectory& current_traj,
                            const ros::WallTime& deadline,
                            SvsdfRuntime::CandidateResult* result);

 private:
  SvsdfRuntime* runtime_;
};

}  // namespace esv_planner
