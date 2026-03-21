#include "esv_planner/framework/recovery_manager.h"
#include "esv_planner/ros_interface/ros_visualizer.h"
#include "esv_planner/core/trajectory_utils.h"
#include "esv_planner/global_search/swept_astar.h"
#include <ros/ros.h>
namespace esv_planner {

void RecoveryManager::MaybeImproveTailTrajectory(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories,
    const ros::WallTime& deadline, SvsdfRuntime::CandidateResult* result) {  if (result == nullptr || result->trajectory.empty()) {
    return;
  }
  if (DeadlineExpired(deadline)) {
    result->budget_exhausted = true;
    return;
  }

  const size_t segment_count = std::min(segments.size(), segment_trajectories.size());
  if (segment_count < 2) {
    return;
  }

  SvsdfRuntime::CandidateResult best = *result;
  double best_length = ApproximateTrajectoryLength(best.trajectory);
  const double clearance_target = std::max(0.08, runtime_->optimizer_params_.safety_margin);
  const size_t piece_trigger = std::max<size_t>(180, 9 * segment_count);
  if (best.min_clearance >= clearance_target &&
      best.trajectory.pos_pieces.size() <= piece_trigger) {
    ROS_INFO(
        "Tail refinement skipped: clearance %.3f >= %.3f and pieces %zu <= %zu",
        best.min_clearance, clearance_target, best.trajectory.pos_pieces.size(),
        piece_trigger);
    return;
  }

  const size_t max_suffix_segments = std::min<size_t>(4, segment_count);
  const size_t first_suffix_index =
      segment_count > max_suffix_segments ? segment_count - max_suffix_segments : 0;

  std::vector<size_t> prefix_piece_counts(segment_count + 1, 0);
  std::vector<double> prefix_lengths(segment_count + 1, 0.0);
  for (size_t i = 0; i < segment_count; ++i) {
    prefix_piece_counts[i + 1] =
        prefix_piece_counts[i] + segment_trajectories[i].pos_pieces.size();
    prefix_lengths[i + 1] =
        prefix_lengths[i] + ApproximateTrajectoryLength(segment_trajectories[i]);
  }

  const ros::WallTime tail_start = ros::WallTime::now();
  const double remaining_budget_sec = RemainingBudgetSec(deadline);
  if (remaining_budget_sec <= 1e-3) {
    result->budget_exhausted = true;
    return;
  }
  double tail_time_budget_sec = 0.35;
  if (std::isfinite(remaining_budget_sec)) {
    tail_time_budget_sec =
        std::min(0.50, std::min(remaining_budget_sec,
                                std::max(0.10, 0.12 * remaining_budget_sec)));
  }
  const ros::WallTime local_tail_deadline =
      tail_start + ros::WallDuration(tail_time_budget_sec);
  const int max_tail_attempts =
      std::min<int>(4, std::max<int>(2, static_cast<int>(max_suffix_segments)));
  int tail_attempts = 0;
  int tail_pruned = 0;
  int consecutive_no_improvement = 0;
  const int max_consecutive_no_improvement = 3;
  bool improved = false;
  bool stop_after_accept = false;
  bool local_budget_exhausted = false;

  auto tail_timed_out = [&]() {
    if (DeadlineExpired(deadline)) {
      result->budget_exhausted = true;
      return true;
    }
    if (ros::WallTime::now() >= local_tail_deadline) {
      local_budget_exhausted = true;
      return true;
    }
    return false;
  };

  auto record_no_improvement = [&]() {
    ++consecutive_no_improvement;
  };

  auto maybe_accept_suffix =
      [&](size_t start_index, const std::vector<SE2State>& suffix_waypoints,
          bool use_strict, const char* label) {
        if (suffix_waypoints.size() < 2 || stop_after_accept || tail_timed_out() ||
            tail_attempts >= max_tail_attempts ||
            consecutive_no_improvement >= max_consecutive_no_improvement) {
          return;
        }

        ++tail_attempts;
        Trajectory suffix_traj;
        const double seg_time = runtime_->EstimateSegmentTime(suffix_waypoints);
        const bool solved =
            use_strict ? runtime_->SolveStrictSegment(suffix_waypoints, &suffix_traj)
                       : runtime_->OptimizeLowRiskSegment(suffix_waypoints, seg_time, &suffix_traj);
        if (!solved || suffix_traj.empty()) {
          record_no_improvement();
          return;
        }

        const size_t candidate_piece_estimate =
            prefix_piece_counts[start_index] + suffix_traj.pos_pieces.size();
        const double candidate_length_estimate =
            prefix_lengths[start_index] + WaypointPathLength(suffix_waypoints);
        const double raw_suffix_clearance =
            runtime_->svsdf_evaluator_.segmentClearance(suffix_waypoints, 0.10);
        const bool can_improve_clearance =
            std::isfinite(raw_suffix_clearance) &&
            raw_suffix_clearance > best.min_clearance + 5e-3;
        const bool can_improve_geometry =
            candidate_piece_estimate + 3 < best.trajectory.pos_pieces.size() ||
            candidate_length_estimate + 0.75 < best_length;
        if (!can_improve_clearance && !can_improve_geometry) {
          ++tail_pruned;
          record_no_improvement();
          return;
        }
        if (tail_timed_out()) {
          return;
        }

        const Trajectory candidate_traj =
            ConcatenateTrajectories(segment_trajectories, start_index, suffix_traj);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!runtime_->IsTrajectoryFeasible(candidate_traj, &min_clearance, &max_vel, &max_acc)) {
          record_no_improvement();
          return;
        }

        const double candidate_length = ApproximateTrajectoryLength(candidate_traj);
        const size_t candidate_pieces = candidate_traj.pos_pieces.size();
        const bool better_clearance = min_clearance > best.min_clearance + 5e-3;
        const bool clearance_not_worse = min_clearance + 2e-3 >= best.min_clearance;
        const bool better_geometry =
            candidate_pieces + 3 < best.trajectory.pos_pieces.size() ||
            candidate_length + 0.75 < best_length;

        if (!better_clearance && !(clearance_not_worse && better_geometry)) {
          record_no_improvement();
          return;
        }

        const double previous_best_clearance = best.min_clearance;
        const size_t previous_best_pieces = best.trajectory.pos_pieces.size();
        const double previous_best_length = best_length;
        int prefix_support_points = 0;
        for (size_t i = 0; i < start_index; ++i) {
          prefix_support_points += static_cast<int>(segments[i].waypoints.size());
        }

        ROS_INFO(
            "Tail refinement accepted from seg %zu via %s: pieces %zu -> %zu, length %.3f -> %.3f, clearance %.3f -> %.3f",
            start_index, label, previous_best_pieces, candidate_pieces,
            previous_best_length, candidate_length, previous_best_clearance,
            min_clearance);

        best.trajectory = candidate_traj;
        best.min_clearance = min_clearance;
        best.max_vel = max_vel;
        best.max_acc = max_acc;
        best.support_points =
            prefix_support_points + static_cast<int>(suffix_waypoints.size());
        best.high_risk_segments =
            CountHighRiskSegments(segments, start_index) + (use_strict ? 1 : 0);
        best_length = candidate_length;
        improved = true;
        consecutive_no_improvement = 0;

        const bool strong_improvement =
            min_clearance > previous_best_clearance + 1e-2 ||
            candidate_pieces + 5 < previous_best_pieces ||
            candidate_length + 1.5 < previous_best_length;
        if (strong_improvement) {
          stop_after_accept = true;
        }
      };

  for (size_t start_index = segment_count - 2;; --start_index) {
    if (start_index < first_suffix_index) {
      break;
    }
    if (stop_after_accept || tail_timed_out() || tail_attempts >= max_tail_attempts ||
        consecutive_no_improvement >= max_consecutive_no_improvement) {
      break;
    }

    std::vector<SE2State> merged_suffix;
    merged_suffix.reserve(segments[start_index].waypoints.size());
    bool suffix_has_high = false;
    for (size_t i = start_index; i < segment_count; ++i) {
      suffix_has_high = suffix_has_high || segments[i].risk == RiskLevel::HIGH;
      AppendUniqueWaypoints(segments[i].waypoints, &merged_suffix);
    }

    if (merged_suffix.size() >= 2) {
      maybe_accept_suffix(start_index, merged_suffix, false, "merged_suffix_r2");
      if (suffix_has_high && !stop_after_accept &&
          consecutive_no_improvement < max_consecutive_no_improvement) {
        maybe_accept_suffix(start_index, merged_suffix, true, "merged_suffix_strict");
      }
    }

    if (start_index == first_suffix_index) {
      break;
    }
  }

  if (tail_attempts > 0 || tail_pruned > 0 || improved || local_budget_exhausted ||
      result->budget_exhausted) {
    ROS_INFO(
        "Tail refinement summary: suffix_start=%zu budget=%.2fs attempts=%d/%d pruned=%d improved=%d exhausted=%d",
        first_suffix_index, tail_time_budget_sec, tail_attempts, max_tail_attempts,
        tail_pruned, improved ? 1 : 0,
        (local_budget_exhausted || result->budget_exhausted) ? 1 : 0);
  }

  best.tail_attempts = tail_attempts;
  best.tail_pruned = tail_pruned;
  best.budget_exhausted = best.budget_exhausted || result->budget_exhausted;
  *result = best;
}

}  // namespace esv_planner
