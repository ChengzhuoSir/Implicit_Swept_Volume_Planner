#include "isweep_planner/framework/svsdf_runtime.h"
#include "isweep_planner/ros_interface/ros_visualizer.h"
#include "isweep_planner/framework/recovery_manager.h"
#include "isweep_planner/global_search/swept_astar.h"
#include "isweep_planner/core/trajectory_utils.h"
#include "isweep_planner/core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <geometry_msgs/PoseStamped.h>

namespace isweep_planner {
SvsdfRuntime::CandidateResult SvsdfRuntime::EvaluateCandidate(
    const std::vector<MotionSegment>& segments, const ros::WallTime& deadline) {
  CandidateResult result;
  result.support_points = CountSupportPoints(segments);
  result.segments = segments;

  SegmentCache cache;
  std::vector<Trajectory> segment_trajectories(segments.size());
  std::vector<bool> escalated(segments.size(), false);
  int high_risk_segments = 0;
  const ros::WallTime candidate_start = ros::WallTime::now();

  auto mark_budget_exhausted = [&](const char* stage) {
    result.budget_exhausted = true;
    ROS_WARN(
        "Candidate evaluation hit optimization deadline during %s after %.3fs (segments=%zu support=%d)",
        stage, (ros::WallTime::now() - candidate_start).toSec(), segments.size(),
        result.support_points);
  };

  auto evaluate_full_trajectory = [&](const Trajectory& traj, double* min_clearance,
                                      double* max_vel, double* max_acc) {
    const ros::WallTime feasibility_start = ros::WallTime::now();
    const bool feasible = IsTrajectoryFeasible(traj, min_clearance, max_vel, max_acc);
    result.full_feasibility_time +=
        (ros::WallTime::now() - feasibility_start).toSec();
    return feasible;
  };

  std::vector<size_t> high_risk_indices;
  std::vector<size_t> low_risk_indices;
  high_risk_indices.reserve(segments.size());
  low_risk_indices.reserve(segments.size());
  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].risk == RiskLevel::HIGH) {
      high_risk_indices.push_back(i);
    } else {
      low_risk_indices.push_back(i);
    }
  }
  high_risk_segments = static_cast<int>(high_risk_indices.size());

  auto collect_local_upgrade_indices =
      [&](const Trajectory& traj, ClearanceProbe* worst_probe) {
        std::vector<size_t> indices;
        if (traj.empty() || segments.empty()) {
          return indices;
        }

        const ClearanceProbe worst = FindWorstClearanceSample(traj, svsdf_evaluator_);
        if (worst_probe != nullptr) {
          *worst_probe = worst;
        }
        if (!worst.valid) {
          return indices;
        }

        const size_t culprit_index =
            SegmentIndexFromTime(segment_trajectories, worst.time);
        const size_t last_index = segments.size() - 1;
        for (size_t radius = 0; radius <= 2 && indices.empty(); ++radius) {
          const size_t begin = culprit_index > radius ? culprit_index - radius : 0;
          const size_t end = std::min(last_index, culprit_index + radius);
          for (size_t i = begin; i <= end; ++i) {
            if (segments[i].risk == RiskLevel::LOW && !escalated[i]) {
              indices.push_back(i);
            }
          }
        }
        return indices;
      };

  const ros::WallTime segment_solve_start = ros::WallTime::now();
  const auto solve_high_risk_segment = [&](size_t i) -> bool {
    if (segments[i].waypoints.size() < 2) {
      return false;
    }
    ROS_INFO("Runtime stage: solving high-risk segment %zu with strict SE(2), points=%zu",
             i, segments[i].waypoints.size());
    Trajectory segment_traj;
    if (!SolveStrictCached(i, segments[i].waypoints, &segment_traj, cache)) {
      ROS_WARN("Strict SE(2) solve failed on high-risk segment %zu", i);
      return false;
    }
    if (segment_traj.empty()) {
      ROS_WARN("Segment %zu optimization failed (risk=HIGH, points=%zu)", i,
               segments[i].waypoints.size());
      return false;
    }
    ROS_INFO("Segment %zu optimized, risk=HIGH, duration=%f", i,
             segment_traj.totalDuration());
    segment_trajectories[i] = segment_traj;
    return true;
  };
  const auto solve_low_risk_segment = [&](size_t i) -> bool {
    if (segments[i].waypoints.size() < 2) {
      return false;
    }
    const double seg_time = EstimateSegmentTime(segments[i].waypoints);
    ROS_INFO("Runtime stage: solving low-risk segment %zu in R^2, points=%zu",
             i, segments[i].waypoints.size());
    Trajectory segment_traj;
    if (!OptimizeLowRiskCached(i, segments[i].waypoints, seg_time, &segment_traj,
                               cache)) {
      ROS_WARN("Low-risk segment %zu failed in both R^2 and strict SE(2)", i);
      return false;
    }
    if (segment_traj.empty()) {
      ROS_WARN("Segment %zu optimization failed (risk=LOW, points=%zu)", i,
               segments[i].waypoints.size());
      return false;
    }
    ROS_INFO("Segment %zu optimized, risk=LOW, duration=%f", i,
             segment_traj.totalDuration());
    segment_trajectories[i] = segment_traj;
    return true;
  };

  for (size_t i : high_risk_indices) {
    if (DeadlineExpired(deadline)) {
      result.segment_solve_time =
          (ros::WallTime::now() - segment_solve_start).toSec();
      mark_budget_exhausted("high-risk segment solve");
      return result;
    }
    if (!solve_high_risk_segment(i)) {
      result.segment_solve_time =
          (ros::WallTime::now() - segment_solve_start).toSec();
      return CandidateResult();
    }
  }

  for (size_t i : low_risk_indices) {
    if (DeadlineExpired(deadline)) {
      result.segment_solve_time =
          (ros::WallTime::now() - segment_solve_start).toSec();
      mark_budget_exhausted("low-risk segment solve");
      return result;
    }
    if (!solve_low_risk_segment(i)) {
      result.segment_solve_time =
          (ros::WallTime::now() - segment_solve_start).toSec();
      return CandidateResult();
    }
  }
  result.segment_solve_time =
      (ros::WallTime::now() - segment_solve_start).toSec();
  result.segment_trajectories = segment_trajectories;

  if (DeadlineExpired(deadline)) {
    mark_budget_exhausted("pre-feasibility");
    return result;
  }

  const Trajectory full_traj = optimizer_.stitch(segments, segment_trajectories);
  if (full_traj.empty()) {
    return CandidateResult();
  }

  ROS_INFO("Stitched trajectory: pieces=%zu, duration=%.3f",
           full_traj.pos_pieces.size(), full_traj.totalDuration());

  double min_clearance = -kInf;
  double max_vel = 0.0;
  double max_acc = 0.0;
  if (!evaluate_full_trajectory(full_traj, &min_clearance, &max_vel, &max_acc)) {
    Trajectory upgraded_full_traj = full_traj;
    double upgraded_min_clearance = min_clearance;
    double upgraded_max_vel = max_vel;
    double upgraded_max_acc = max_acc;
    ClearanceProbe bottleneck;
    const std::vector<size_t> local_upgrade_indices =
        collect_local_upgrade_indices(full_traj, &bottleneck);
    const size_t bottleneck_index =
        bottleneck.valid ? SegmentIndexFromTime(segment_trajectories, bottleneck.time) : 0;

    if (!local_upgrade_indices.empty()) {
      const ros::WallTime escalation_start = ros::WallTime::now();
      for (size_t i : local_upgrade_indices) {
        if (DeadlineExpired(deadline)) {
          mark_budget_exhausted("localized low-risk escalation");
          result.trajectory = upgraded_full_traj;
          result.min_clearance = upgraded_min_clearance;
          result.max_vel = upgraded_max_vel;
          result.max_acc = upgraded_max_acc;
          result.high_risk_segments = high_risk_segments;
          result.segment_trajectories = segment_trajectories;
          result.degraded = true;
          return result;
        }
        if (!SolveStrictCached(i, segments[i].waypoints, &segment_trajectories[i], cache)) {
          ROS_WARN("Localized low-risk strict upgrade failed on segment %zu", i);
          return CandidateResult();
        }
        escalated[i] = true;
        ++result.escalated_segments;
      }
      result.segment_solve_time +=
          (ros::WallTime::now() - escalation_start).toSec();

      ROS_INFO(
          "Localized strict upgrade around bottleneck: culprit_seg=%zu clearance=%.3f upgraded_low=%zu window=[%zu,%zu]",
          bottleneck_index, bottleneck.clearance, local_upgrade_indices.size(),
          local_upgrade_indices.front(), local_upgrade_indices.back());

      result.segment_trajectories = segment_trajectories;
      upgraded_full_traj = optimizer_.stitch(segments, segment_trajectories);
      upgraded_min_clearance = -kInf;
      upgraded_max_vel = 0.0;
      upgraded_max_acc = 0.0;
      if (!upgraded_full_traj.empty() &&
          evaluate_full_trajectory(upgraded_full_traj, &upgraded_min_clearance,
                                   &upgraded_max_vel, &upgraded_max_acc)) {
        result.trajectory = upgraded_full_traj;
        result.min_clearance = upgraded_min_clearance;
        result.max_vel = upgraded_max_vel;
        result.max_acc = upgraded_max_acc;
        result.high_risk_segments = high_risk_segments;
        result.segment_trajectories = segment_trajectories;
        if (DeadlineExpired(deadline)) {
          result.budget_exhausted = true;
        }
        return result;
      }
    } else {
      ROS_WARN(
          "No local low-risk segments found near bottleneck: culprit_seg=%zu clearance=%.3f; skipping blanket strict escalation",
          bottleneck_index, bottleneck.clearance);
    }

    if (DeadlineExpired(deadline)) {
      mark_budget_exhausted("recovery setup");
      result.trajectory = upgraded_full_traj;
      result.min_clearance = upgraded_min_clearance;
      result.max_vel = upgraded_max_vel;
      result.max_acc = upgraded_max_acc;
      result.high_risk_segments = high_risk_segments;
      result.segment_trajectories = segment_trajectories;
      result.degraded = true;
      return result;
    }

    const ros::WallTime recovery_start = ros::WallTime::now();
    RecoveryManager recovery(this);
    if (recovery.TryRecoverBottleneck(segments, segment_trajectories, upgraded_full_traj,
                                      deadline, &result)) {
      result.recovery_time += (ros::WallTime::now() - recovery_start).toSec();
      result.segment_trajectories = segment_trajectories;
      return result;
    }
    result.recovery_time += (ros::WallTime::now() - recovery_start).toSec();

    ROS_WARN("Path degraded after local recovery failure (clearance=%.3f)",
             upgraded_min_clearance);
    result.trajectory = upgraded_full_traj;
    result.min_clearance = upgraded_min_clearance;
    result.max_vel = upgraded_max_vel;
    result.max_acc = upgraded_max_acc;
    result.high_risk_segments = high_risk_segments;
    result.segment_trajectories = segment_trajectories;
    result.degraded = true;
    return result;
  }

  result.trajectory = full_traj;
  result.min_clearance = min_clearance;
  result.max_vel = max_vel;
  result.max_acc = max_acc;
  result.high_risk_segments = high_risk_segments;
  result.segment_trajectories = segment_trajectories;
  if (DeadlineExpired(deadline)) {
    result.budget_exhausted = true;
  }
  return result;
}
} // namespace isweep_planner
