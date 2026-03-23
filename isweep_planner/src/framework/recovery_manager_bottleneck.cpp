#include "isweep_planner/framework/recovery_manager.h"
#include "isweep_planner/ros_interface/ros_visualizer.h"
#include "isweep_planner/core/trajectory_utils.h"
#include "isweep_planner/global_search/swept_astar.h"
#include <ros/ros.h>
namespace isweep_planner {

bool RecoveryManager::TryRecoverBottleneck(

    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories, const Trajectory& current_traj,
    const ros::WallTime& deadline, SvsdfRuntime::CandidateResult* result) {  if (result == nullptr || current_traj.empty()) {
    return false;
  }
  if (DeadlineExpired(deadline)) {
    result->budget_exhausted = true;
    return false;
  }

  const double remaining_budget_sec = RemainingBudgetSec(deadline);
  if (remaining_budget_sec <= 1e-3) {
    result->budget_exhausted = true;
    return false;
  }

  const ros::WallTime recover_start = ros::WallTime::now();
  const size_t segment_count = std::min(segments.size(), segment_trajectories.size());
  if (segment_count == 0) {
    return false;
  }

  const ClearanceProbe worst = FindWorstClearanceSample(current_traj, runtime_->svsdf_evaluator_);
  if (!worst.valid) {
    return false;
  }

  std::vector<ClearanceProbe> hotspots(1, worst);
  const double recover_timeout_sec = std::min(8.0, remaining_budget_sec);

  const size_t culprit_index = SegmentIndexFromTime(segment_trajectories, worst.time);
  const double anchor_clearance_threshold = std::max(0.05, runtime_->optimizer_params_.safety_margin);
  const double endpoint_pos_tolerance =
      std::max(0.15, runtime_->hybrid_astar_params_.goal_tolerance_pos + 0.05);
  const double endpoint_yaw_tolerance =
      std::max(0.20, runtime_->hybrid_astar_params_.goal_tolerance_yaw + 0.10);

  struct RecoveryWindow {
    size_t begin = 0;
    size_t end = 0;
    size_t culprit_index = 0;
    ClearanceProbe hotspot;
  };

  std::vector<RecoveryWindow> candidate_windows;
  auto add_window = [&](size_t begin, size_t end, size_t window_culprit_index,
                        const ClearanceProbe& hotspot) {
    if (begin > end || end >= segment_count) {
      return;
    }
    for (const RecoveryWindow& window : candidate_windows) {
      if (window.begin == begin && window.end == end) {
        return;
      }
    }
    RecoveryWindow window;
    window.begin = begin;
    window.end = end;
    window.culprit_index = window_culprit_index;
    window.hotspot = hotspot;
    candidate_windows.push_back(window);
  };

  auto add_windows_for_hotspot = [&](const ClearanceProbe& hotspot) {
    const size_t hotspot_culprit_index =
        SegmentIndexFromTime(segment_trajectories, hotspot.time);

    size_t risky_begin = hotspot_culprit_index;
    size_t risky_end = hotspot_culprit_index;
    if (segments[hotspot_culprit_index].risk == RiskLevel::LOW) {
      if (hotspot_culprit_index > 0 &&
          segments[hotspot_culprit_index - 1].risk == RiskLevel::HIGH) {
        risky_begin = hotspot_culprit_index - 1;
      }
      if (hotspot_culprit_index + 1 < segment_count &&
          segments[hotspot_culprit_index + 1].risk == RiskLevel::HIGH) {
        risky_end = hotspot_culprit_index + 1;
      }
    }
    while (risky_begin > 0 && segments[risky_begin - 1].risk == RiskLevel::HIGH) {
      --risky_begin;
    }
    while (risky_end + 1 < segment_count &&
           segments[risky_end + 1].risk == RiskLevel::HIGH) {
      ++risky_end;
    }

    const size_t local_begin = risky_begin > 0 ? risky_begin - 1 : risky_begin;
    const size_t local_end = std::min(segment_count - 1, risky_end + 1);
    const size_t expanded_begin = local_begin > 0 ? local_begin - 1 : local_begin;
    const size_t expanded_end = std::min(segment_count - 1, local_end + 1);

    add_window(expanded_begin, expanded_end, hotspot_culprit_index, hotspot);
    add_window(local_begin, local_end, hotspot_culprit_index, hotspot);
    add_window(hotspot_culprit_index, hotspot_culprit_index, hotspot_culprit_index,
               hotspot);
  };

  for (const ClearanceProbe& hotspot : hotspots) {
    add_windows_for_hotspot(hotspot);
  }

  std::stable_sort(candidate_windows.begin(), candidate_windows.end(),
                   [](const RecoveryWindow& lhs, const RecoveryWindow& rhs) {
                     const size_t lhs_span = lhs.end - lhs.begin;
                     const size_t rhs_span = rhs.end - rhs.begin;
                     if (lhs_span != rhs_span) {
                       return lhs_span < rhs_span;
                     }
                     return lhs.hotspot.clearance < rhs.hotspot.clearance;
                   });
  if (candidate_windows.size() > 3) {
    candidate_windows.resize(3);
  }

  ROS_INFO(
      "Local bottleneck rebuild: culprit_seg=%zu state=(%.3f, %.3f, %.3f) clearance=%.3f hotspots=%zu windows=%zu timeout=%.1f per_window=%.1f",
      culprit_index, worst.state.x, worst.state.y, worst.state.yaw, worst.clearance,
      hotspots.size(), candidate_windows.size(), recover_timeout_sec,
      std::max(3.0, recover_timeout_sec /
                        static_cast<double>(
                            std::max<size_t>(1, std::min(candidate_windows.size(),
                                                          hotspots.size() + 1)))));

  SvsdfRuntime::CandidateResult best;
  double best_length = kInf;
  bool found = false;
  struct PartialRecoveryPatch {
    size_t begin = 0;
    size_t end = 0;
    Trajectory replacement;
    double local_clearance = -kInf;
    int support_points = 0;
    int high_risk_segments = 0;
    int escalated_segments = 0;
  };
  std::vector<PartialRecoveryPatch> partial_patches;
  std::set<std::pair<size_t, size_t>> processed_windows;
  size_t active_culprit_index = culprit_index;
  ClearanceProbe active_hotspot = worst;
  struct RecoveryDiagnostics {
    int corridor_search_attempts = 0;
    int corridor_search_successes = 0;
    int corridor_topo_successes = 0;
    int corridor_segment_successes = 0;
    int direct_plan_attempts = 0;
    int direct_plan_successes = 0;
    int detour_plan_attempts = 0;
    int detour_plan_successes = 0;
    int single_patch_attempts = 0;
    int single_patch_solver_failures = 0;
    int single_patch_raw_infeasible = 0;
    int single_patch_raw_feasible = 0;
    int single_patch_local_infeasible = 0;
    int single_patch_rebuilt_infeasible = 0;
    int single_patch_successes = 0;
    int segment_patch_attempts = 0;
    int segment_patch_solver_failures = 0;
    int segment_patch_middle_infeasible = 0;
    int segment_patch_rebuilt_infeasible = 0;
    int segment_patch_successes = 0;
    double best_raw_clearance = -kInf;
    double best_local_clearance = -kInf;
    double best_rebuilt_clearance = -kInf;
    std::string best_raw_label;
    std::string best_local_label;
    std::string best_rebuilt_label;
  };
  RecoveryDiagnostics current_diag;
  auto note_best_clearance =
      [&](double clearance, const char* label, const char* stage) {
    if (!std::isfinite(clearance)) {
      return;
    }
    if (std::strcmp(stage, "rebuilt") == 0) {
      if (clearance > current_diag.best_rebuilt_clearance) {
        current_diag.best_rebuilt_clearance = clearance;
        current_diag.best_rebuilt_label = label;
      }
      return;
    }
    if (std::strcmp(stage, "raw") == 0) {
      if (clearance > current_diag.best_raw_clearance) {
        current_diag.best_raw_clearance = clearance;
        current_diag.best_raw_label = label;
      }
      return;
    }
    if (clearance > current_diag.best_local_clearance) {
      current_diag.best_local_clearance = clearance;
      current_diag.best_local_label = label;
    }
  };
  const double per_window_timeout_sec =
      std::max(2.0, recover_timeout_sec /
                        static_cast<double>(
                            std::max<size_t>(1, candidate_windows.size())));
  ros::WallTime current_window_start = recover_start;
  auto recover_timed_out = [&]() {
    if (DeadlineExpired(deadline)) {
      if (result != nullptr) {
        result->budget_exhausted = true;
      }
      return true;
    }
    const ros::WallDuration total_elapsed = ros::WallTime::now() - recover_start;
    if (total_elapsed.toSec() > recover_timeout_sec) {
      return true;
    }
    const ros::WallDuration window_elapsed = ros::WallTime::now() - current_window_start;
    return window_elapsed.toSec() > per_window_timeout_sec;
  };

  auto maybe_accept_patch =
      [&](size_t begin, size_t end, const std::vector<SE2State>& patch_waypoints,
          const char* label) {
        if (recover_timed_out()) {
          return;
        }
        if (patch_waypoints.size() < 2) {
          return;
        }
        ++current_diag.single_patch_attempts;
        const double raw_patch_clearance =
            runtime_->svsdf_evaluator_.segmentClearance(patch_waypoints, 0.10);
        note_best_clearance(raw_patch_clearance, label, "raw");
        if (std::isfinite(raw_patch_clearance) && raw_patch_clearance > -0.051) {
          ++current_diag.single_patch_raw_feasible;
        } else {
          ++current_diag.single_patch_raw_infeasible;
        }

        Trajectory patch_traj;
        if (!runtime_->SolveStrictSegment(patch_waypoints, &patch_traj) || patch_traj.empty()) {
          patch_traj = runtime_->optimizer_.optimizeSE2(
              patch_waypoints, 1.5 * runtime_->EstimateSegmentTime(patch_waypoints));
        }
        if (patch_traj.empty()) {
          ++current_diag.single_patch_solver_failures;
          return;
        }

        double patch_clearance = -kInf;
        if (!runtime_->IsTrajectoryFeasible(patch_traj, &patch_clearance, nullptr, nullptr)) {
          ++current_diag.single_patch_local_infeasible;
          note_best_clearance(patch_clearance, label, "local");
          return;
        }
        note_best_clearance(patch_clearance, label, "local");

        const Trajectory rebuilt =
            ConcatenateTrajectories(segment_trajectories, begin, end + 1, patch_traj);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!runtime_->IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
          ++current_diag.single_patch_rebuilt_infeasible;
          note_best_clearance(min_clearance, label, "rebuilt");
          PartialRecoveryPatch partial;
          partial.begin = begin;
          partial.end = end;
          partial.replacement = patch_traj;
          partial.local_clearance = patch_clearance;
          partial.support_points = static_cast<int>(patch_waypoints.size());
          partial.high_risk_segments = 1;
          partial.escalated_segments = CountHighRiskSegments(segments, begin, end + 1);
          bool replaced = false;
          for (PartialRecoveryPatch& existing : partial_patches) {
            if (existing.begin == partial.begin && existing.end == partial.end) {
              if (partial.local_clearance > existing.local_clearance + 1e-3 ||
                  (partial.local_clearance + 1e-3 >= existing.local_clearance &&
                   partial.replacement.pos_pieces.size() <
                       existing.replacement.pos_pieces.size())) {
                existing = partial;
              }
              replaced = true;
              break;
            }
          }
          if (!replaced) {
            partial_patches.push_back(partial);
          }
          return;
        }
        ++current_diag.single_patch_successes;
        note_best_clearance(min_clearance, label, "rebuilt");

        const double rebuilt_length = ApproximateTrajectoryLength(rebuilt);
        const bool better_candidate =
            !found || min_clearance > best.min_clearance + 1e-3 ||
            (min_clearance + 1e-3 >= best.min_clearance &&
             (rebuilt.pos_pieces.size() < best.trajectory.pos_pieces.size() ||
              rebuilt_length + 0.25 < best_length));
        if (!better_candidate) {
          return;
        }

        SvsdfRuntime::CandidateResult candidate = *result;
        candidate.trajectory = rebuilt;
        candidate.min_clearance = min_clearance;
        candidate.max_vel = max_vel;
        candidate.max_acc = max_acc;
        candidate.support_points =
            CountSupportPoints(segments, 0, begin) +
            static_cast<int>(patch_waypoints.size()) +
            CountSupportPoints(segments, end + 1, segment_count);
        candidate.high_risk_segments =
            CountHighRiskSegments(segments, 0, begin) + 1 +
            CountHighRiskSegments(segments, end + 1, segment_count);
        candidate.escalated_segments = std::max(
            candidate.escalated_segments,
            CountHighRiskSegments(segments, begin, end + 1));

        ROS_INFO(
            "Local bottleneck rebuild accepted via %s: culprit_seg=%zu window=[%zu,%zu] clearance %.3f -> %.3f pieces %zu -> %zu",
            label, active_culprit_index, begin, end, active_hotspot.clearance,
            min_clearance,
            current_traj.pos_pieces.size(), rebuilt.pos_pieces.size());

        best = candidate;
        best_length = rebuilt_length;
        found = true;
      };

  auto maybe_accept_segment_patch =
      [&](size_t begin, size_t end, const std::vector<MotionSegment>& patch_segments,
          const char* label) {
        if (recover_timed_out()) {
          return;
        }
        if (patch_segments.empty()) {
          return;
        }
        ++current_diag.segment_patch_attempts;

        std::vector<Trajectory> patch_segment_trajectories(patch_segments.size());
        std::vector<bool> escalated(patch_segments.size(), false);
        int escalated_segments = 0;
        for (size_t patch_index = 0; patch_index < patch_segments.size(); ++patch_index) {
          if (recover_timed_out()) {
            return;
          }
          if (patch_segments[patch_index].waypoints.size() < 2) {
            return;
          }

          const double seg_time =
              runtime_->EstimateSegmentTime(patch_segments[patch_index].waypoints);
          Trajectory patch_segment_traj;
          if (patch_segments[patch_index].risk == RiskLevel::HIGH) {
            if (!runtime_->SolveStrictSegment(patch_segments[patch_index].waypoints,
                                    &patch_segment_traj)) {
              ++current_diag.segment_patch_solver_failures;
              return;
            }
          } else {
            if (!runtime_->OptimizeLowRiskSegment(patch_segments[patch_index].waypoints, seg_time,
                                        &patch_segment_traj)) {
              ++current_diag.segment_patch_solver_failures;
              return;
            }
            double segment_clearance = -kInf;
            if (!runtime_->IsTrajectoryFeasible(patch_segment_traj, &segment_clearance, nullptr,
                                      nullptr)) {
              if (!runtime_->SolveStrictSegment(patch_segments[patch_index].waypoints,
                                      &patch_segment_traj)) {
                ++current_diag.segment_patch_solver_failures;
                return;
              }
              escalated[patch_index] = true;
              ++escalated_segments;
            }
          }

          if (patch_segment_traj.empty()) {
            return;
          }
          patch_segment_trajectories[patch_index] = patch_segment_traj;
        }

        Trajectory middle = runtime_->optimizer_.stitch(patch_segments, patch_segment_trajectories);
        double middle_clearance = -kInf;
        if (middle.empty() ||
            !runtime_->IsTrajectoryFeasible(middle, &middle_clearance, nullptr, nullptr)) {
          ++current_diag.segment_patch_middle_infeasible;
          note_best_clearance(middle_clearance, label, "local");
          bool upgraded = false;
          for (size_t patch_index = 0; patch_index < patch_segments.size(); ++patch_index) {
            if (patch_segments[patch_index].risk == RiskLevel::LOW &&
                !escalated[patch_index]) {
              if (!runtime_->SolveStrictSegment(patch_segments[patch_index].waypoints,
                                      &patch_segment_trajectories[patch_index])) {
                return;
              }
              escalated[patch_index] = true;
              ++escalated_segments;
              upgraded = true;
            }
          }
          if (!upgraded) {
            return;
          }
          middle = runtime_->optimizer_.stitch(patch_segments, patch_segment_trajectories);
          if (middle.empty() ||
              !runtime_->IsTrajectoryFeasible(middle, &middle_clearance, nullptr, nullptr)) {
            ++current_diag.segment_patch_middle_infeasible;
            note_best_clearance(middle_clearance, label, "local");
            return;
          }
        }
        note_best_clearance(middle_clearance, label, "local");

        const Trajectory rebuilt =
            ConcatenateTrajectories(segment_trajectories, begin, end + 1, middle);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!runtime_->IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
          ++current_diag.segment_patch_rebuilt_infeasible;
          note_best_clearance(min_clearance, label, "rebuilt");
          PartialRecoveryPatch partial;
          partial.begin = begin;
          partial.end = end;
          partial.replacement = middle;
          partial.local_clearance = middle_clearance;
          partial.support_points =
              CountSupportPoints(patch_segments, 0, patch_segments.size());
          partial.high_risk_segments =
              CountHighRiskSegments(patch_segments, 0, patch_segments.size());
          partial.escalated_segments = escalated_segments;
          bool replaced = false;
          for (PartialRecoveryPatch& existing : partial_patches) {
            if (existing.begin == partial.begin && existing.end == partial.end) {
              if (partial.local_clearance > existing.local_clearance + 1e-3 ||
                  (partial.local_clearance + 1e-3 >= existing.local_clearance &&
                   partial.replacement.pos_pieces.size() <
                       existing.replacement.pos_pieces.size())) {
                existing = partial;
              }
              replaced = true;
              break;
            }
          }
          if (!replaced) {
            partial_patches.push_back(partial);
          }
          return;
        }
        ++current_diag.segment_patch_successes;
        note_best_clearance(min_clearance, label, "rebuilt");

        const double rebuilt_length = ApproximateTrajectoryLength(rebuilt);
        const bool better_candidate =
            !found || min_clearance > best.min_clearance + 1e-3 ||
            (min_clearance + 1e-3 >= best.min_clearance &&
             (rebuilt.pos_pieces.size() < best.trajectory.pos_pieces.size() ||
              rebuilt_length + 0.25 < best_length));
        if (!better_candidate) {
          return;
        }

        SvsdfRuntime::CandidateResult candidate = *result;
        candidate.trajectory = rebuilt;
        candidate.min_clearance = min_clearance;
        candidate.max_vel = max_vel;
        candidate.max_acc = max_acc;
        candidate.support_points =
            CountSupportPoints(segments, 0, begin) +
            CountSupportPoints(patch_segments, 0, patch_segments.size()) +
            CountSupportPoints(segments, end + 1, segment_count);
        candidate.high_risk_segments =
            CountHighRiskSegments(segments, 0, begin) +
            CountHighRiskSegments(patch_segments, 0, patch_segments.size()) +
            CountHighRiskSegments(segments, end + 1, segment_count);
        candidate.escalated_segments =
            std::max(candidate.escalated_segments, escalated_segments);

        ROS_INFO(
            "Local bottleneck rebuild accepted via %s: culprit_seg=%zu window=[%zu,%zu] clearance %.3f -> %.3f pieces %zu -> %zu",
            label, active_culprit_index, begin, end, active_hotspot.clearance,
            min_clearance,
            current_traj.pos_pieces.size(), rebuilt.pos_pieces.size());

        best = candidate;
        best_length = rebuilt_length;
        found = true;
      };

  auto log_window_diag = [&](size_t begin, size_t end) {
    ROS_INFO(
        "Local bottleneck diag window=[%zu,%zu] culprit_seg=%zu corridor=%d/%d topo=%d seg=%d direct=%d/%d detour=%d/%d single_patch=%d raw_ok=%d raw_fail=%d solve_fail=%d local_fail=%d rebuilt_fail=%d ok=%d seg_patch=%d solve_fail=%d middle_fail=%d rebuilt_fail=%d ok=%d best_raw=%.3f(%s) best_local=%.3f(%s) best_rebuilt=%.3f(%s)",
        begin, end, active_culprit_index, current_diag.corridor_search_successes,
        current_diag.corridor_search_attempts, current_diag.corridor_topo_successes,
        current_diag.corridor_segment_successes, current_diag.direct_plan_successes,
        current_diag.direct_plan_attempts, current_diag.detour_plan_successes,
        current_diag.detour_plan_attempts, current_diag.single_patch_attempts,
        current_diag.single_patch_raw_feasible,
        current_diag.single_patch_raw_infeasible,
        current_diag.single_patch_solver_failures,
        current_diag.single_patch_local_infeasible,
        current_diag.single_patch_rebuilt_infeasible,
        current_diag.single_patch_successes, current_diag.segment_patch_attempts,
        current_diag.segment_patch_solver_failures,
        current_diag.segment_patch_middle_infeasible,
        current_diag.segment_patch_rebuilt_infeasible,
        current_diag.segment_patch_successes, current_diag.best_raw_clearance,
        current_diag.best_raw_label.empty() ? "-" :
                                             current_diag.best_raw_label.c_str(),
        current_diag.best_local_clearance,
        current_diag.best_local_label.empty() ? "-" :
                                               current_diag.best_local_label.c_str(),
        current_diag.best_rebuilt_clearance,
        current_diag.best_rebuilt_label.empty() ? "-" :
                                                 current_diag.best_rebuilt_label.c_str());
  };

  for (const RecoveryWindow& window : candidate_windows) {
      if (recover_timed_out()) {
        ROS_WARN("TryRecoverBottleneck timeout after %.1fs", recover_timeout_sec);
        break;
      }
      current_window_start = ros::WallTime::now();
      current_diag = RecoveryDiagnostics();
      active_culprit_index = window.culprit_index;
      active_hotspot = window.hotspot;
      size_t begin = window.begin;
      size_t end = window.end;

      while (begin > 0 &&
             (!std::isfinite(runtime_->svsdf_evaluator_.evaluate(segments[begin].waypoints.front())) ||
              runtime_->svsdf_evaluator_.evaluate(segments[begin].waypoints.front()) <
                  anchor_clearance_threshold)) {
        --begin;
      }
      while (end + 1 < segment_count &&
             (!std::isfinite(runtime_->svsdf_evaluator_.evaluate(segments[end].waypoints.back())) ||
              runtime_->svsdf_evaluator_.evaluate(segments[end].waypoints.back()) <
                  anchor_clearance_threshold)) {
        ++end;
      }

      if (!processed_windows.insert(std::make_pair(begin, end)).second) {
        continue;
      }

      const SE2State start_anchor = segments[begin].waypoints.front();
      const SE2State goal_anchor = segments[end].waypoints.back();
      const double start_clearance = runtime_->svsdf_evaluator_.evaluate(start_anchor);
      const double goal_clearance = runtime_->svsdf_evaluator_.evaluate(goal_anchor);
      if (!std::isfinite(start_clearance) || !std::isfinite(goal_clearance) ||
          start_clearance < anchor_clearance_threshold ||
          goal_clearance < anchor_clearance_threshold) {
        continue;
      }

      const std::pair<double, double> corridor_window =
          SegmentTimeWindow(segment_trajectories, begin, end);
      const double corridor_block_threshold =
          std::max(0.10, anchor_clearance_threshold + runtime_->grid_map_.resolution());
      const double corridor_sample_spacing =
          std::max(0.20, 0.8 * runtime_->footprint_.inscribedRadius());
      std::vector<Eigen::Vector2d> blocked_centers = SampleLowClearanceCorridor(
          current_traj, runtime_->svsdf_evaluator_, corridor_window.first - 0.20,
          corridor_window.second + 0.20, corridor_block_threshold,
          corridor_sample_spacing);
      if (blocked_centers.empty()) {
        blocked_centers.push_back(active_hotspot.state.position());
      }

      ROS_INFO(
          "Local bottleneck rebuild window=[%zu,%zu] culprit_seg=%zu anchors=(%.3f, %.3f)->(%.3f, %.3f) blocked_centers=%zu",
          begin, end, active_culprit_index, start_anchor.x, start_anchor.y,
          goal_anchor.x, goal_anchor.y, blocked_centers.size());

      const bool precise_window = (begin == end && end == active_culprit_index);

      const double hotspot_radius_base =
          std::max(1.00, 3.0 * runtime_->footprint_.circumscribedRadius());
      std::vector<double> hotspot_radius_scales;
      hotspot_radius_scales.push_back(1.0);
      if (precise_window) {
        hotspot_radius_scales.insert(hotspot_radius_scales.begin(), 0.45);
        hotspot_radius_scales.insert(hotspot_radius_scales.begin() + 1, 0.70);
        hotspot_radius_scales.push_back(1.5);
        hotspot_radius_scales.push_back(2.0);
      }
      for (double radius_scale : hotspot_radius_scales) {
        if (recover_timed_out()) {
          break;
        }
        const double hotspot_radius = hotspot_radius_base * radius_scale;
        const double clearance_base =
            std::max(anchor_clearance_threshold,
                     runtime_->footprint_.inscribedRadius() + runtime_->grid_map_.resolution());
        std::vector<double> clearance_candidates;
        clearance_candidates.push_back(
            std::max(clearance_base, 0.5 * runtime_->footprint_.circumscribedRadius()));
        if (precise_window) {
          clearance_candidates.push_back(clearance_base);
          clearance_candidates.push_back(std::max(0.08, runtime_->optimizer_params_.safety_margin));
        }
        std::vector<int> min_safe_yaw_candidates;
        min_safe_yaw_candidates.push_back(4);
        if (precise_window) {
          min_safe_yaw_candidates.push_back(2);
          min_safe_yaw_candidates.push_back(1);
        }
        for (double corridor_clearance : clearance_candidates) {
          if (recover_timed_out()) {
            break;
          }
          SweptAstar corridor_astar;
          corridor_astar.init(runtime_->grid_map_, runtime_->collision_checker_, corridor_clearance);

          Eigen::Vector2d min_corner = start_anchor.position().cwiseMin(goal_anchor.position());
          Eigen::Vector2d max_corner = start_anchor.position().cwiseMax(goal_anchor.position());
          min_corner = min_corner.cwiseMin(active_hotspot.state.position());
          max_corner = max_corner.cwiseMax(active_hotspot.state.position());
          for (const Eigen::Vector2d& center : blocked_centers) {
            min_corner = min_corner.cwiseMin(center);
            max_corner = max_corner.cwiseMax(center);
          }
          const double search_padding =
              precise_window ? std::max(3.0, 2.0 * hotspot_radius)
                             : std::max(2.0, 1.5 * hotspot_radius);
          min_corner.array() -= search_padding;
          max_corner.array() += search_padding;
          GridIndex min_index = runtime_->grid_map_.worldToGrid(min_corner.x(), min_corner.y());
          GridIndex max_index = runtime_->grid_map_.worldToGrid(max_corner.x(), max_corner.y());

          SweptAstarSearchOptions options;
          options.blocked_centers = blocked_centers;
          options.blocked_radius = hotspot_radius;
          options.min_x = std::max(0, min_index.x);
          options.min_y = std::max(0, min_index.y);
          options.max_x = std::min(runtime_->grid_map_.width() - 1, max_index.x);
          options.max_y = std::min(runtime_->grid_map_.height() - 1, max_index.y);
          options.preferred_clearance = corridor_clearance + 0.10;
          options.clearance_penalty = 1.5;
          options.blocked_center_penalty = 1.0;

          for (int min_safe_yaw_count : min_safe_yaw_candidates) {
            if (recover_timed_out()) {
              break;
            }
            options.min_safe_yaw_count = min_safe_yaw_count;
            ++current_diag.corridor_search_attempts;
            const AstarSearchResult corridor_result =
                corridor_astar.search(start_anchor.position(), goal_anchor.position(),
                                      options);
            if (!corridor_result.success || corridor_result.path.size() < 2) {
              continue;
            }
            ++current_diag.corridor_search_successes;

            const TopoPath corridor_topo = BuildTopoPathFromCorridor(
                corridor_result.path, runtime_->grid_map_, runtime_->collision_checker_, corridor_clearance,
                blocked_centers, hotspot_radius, min_safe_yaw_count);
            if (corridor_topo.waypoints.size() < 2) {
              continue;
            }
            ++current_diag.corridor_topo_successes;

            const std::vector<MotionSegment> corridor_segments =
                runtime_->se2_generator_.generate(corridor_topo, start_anchor, goal_anchor);
            if (corridor_segments.empty()) {
              continue;
            }
            ++current_diag.corridor_segment_successes;
            const std::string label =
                "local_corridor_astar_r" + std::to_string(hotspot_radius) +
                "_c" + std::to_string(corridor_clearance) +
                "_y" + std::to_string(min_safe_yaw_count);
            maybe_accept_segment_patch(begin, end, corridor_segments, label.c_str());
          }
        }
      }

      std::vector<SE2State> patch_waypoints;
      if (recover_timed_out()) {
        continue;
      }
      ++current_diag.direct_plan_attempts;
      if (!runtime_->hybrid_astar_.plan(start_anchor, goal_anchor, patch_waypoints) ||
          patch_waypoints.size() < 2) {
        log_window_diag(begin, end);
        continue;
      }
      ++current_diag.direct_plan_successes;

      const SE2State patch_start = patch_waypoints.front();
      const SE2State patch_goal = patch_waypoints.back();
      if ((patch_start.position() - start_anchor.position()).norm() >
              endpoint_pos_tolerance ||
          std::abs(normalizeAngle(patch_start.yaw - start_anchor.yaw)) >
              endpoint_yaw_tolerance ||
          (patch_goal.position() - goal_anchor.position()).norm() >
              endpoint_pos_tolerance ||
          std::abs(normalizeAngle(patch_goal.yaw - goal_anchor.yaw)) >
              endpoint_yaw_tolerance) {
        log_window_diag(begin, end);
        continue;
      }

      patch_waypoints.front() = start_anchor;
      patch_waypoints.back() = goal_anchor;
      maybe_accept_patch(begin, end, patch_waypoints, "direct_hastar");

      const Eigen::Vector2d lateral(-std::sin(active_hotspot.state.yaw),
                                    std::cos(active_hotspot.state.yaw));
      std::vector<double> detour_offsets;
      detour_offsets.push_back(1.00);
      if (precise_window) {
        detour_offsets.push_back(0.60);
        detour_offsets.push_back(1.40);
        detour_offsets.push_back(1.80);
      }
      for (int side : {-1, 1}) {
        if (recover_timed_out()) {
          break;
        }
        for (double offset : detour_offsets) {
          if (recover_timed_out()) {
            break;
          }
          ++current_diag.detour_plan_attempts;
          const Eigen::Vector2d via_pos =
              active_hotspot.state.position() +
              static_cast<double>(side) * offset * lateral;
          if (runtime_->grid_map_.getEsdf(via_pos.x(), via_pos.y()) < 0.10) {
            continue;
          }

          const SE2State via_state(via_pos.x(), via_pos.y(), active_hotspot.state.yaw);
          std::vector<SE2State> first_leg;
          if (!runtime_->hybrid_astar_.plan(start_anchor, via_state, first_leg) ||
              first_leg.size() < 2) {
            continue;
          }

          const SE2State recovered_via = first_leg.back();
          std::vector<SE2State> second_leg;
          if (!runtime_->hybrid_astar_.plan(recovered_via, goal_anchor, second_leg) ||
              second_leg.size() < 2) {
            continue;
          }
          ++current_diag.detour_plan_successes;

          if ((first_leg.front().position() - start_anchor.position()).norm() >
                  endpoint_pos_tolerance ||
              std::abs(normalizeAngle(first_leg.front().yaw - start_anchor.yaw)) >
                  endpoint_yaw_tolerance ||
              (second_leg.back().position() - goal_anchor.position()).norm() >
                  endpoint_pos_tolerance ||
              std::abs(normalizeAngle(second_leg.back().yaw - goal_anchor.yaw)) >
                  endpoint_yaw_tolerance) {
            continue;
          }

          first_leg.front() = start_anchor;
          second_leg.back() = goal_anchor;

          std::vector<SE2State> detour_patch = first_leg;
          AppendUniqueWaypoints(second_leg, &detour_patch);
          maybe_accept_patch(begin, end, detour_patch,
                             side < 0 ? "detour_right" : "detour_left");
        }
      }
      log_window_diag(begin, end);
      if (found) break;
  }

  if (!found && !partial_patches.empty()) {
    std::stable_sort(
        partial_patches.begin(), partial_patches.end(),
        [](const PartialRecoveryPatch& lhs, const PartialRecoveryPatch& rhs) {
          if (lhs.local_clearance != rhs.local_clearance) {
            return lhs.local_clearance > rhs.local_clearance;
          }
          const size_t lhs_span = lhs.end - lhs.begin;
          const size_t rhs_span = rhs.end - rhs.begin;
          if (lhs_span != rhs_span) {
            return lhs_span > rhs_span;
          }
          return lhs.replacement.pos_pieces.size() < rhs.replacement.pos_pieces.size();
        });

    std::vector<PartialRecoveryPatch> selected_patches;
    for (const PartialRecoveryPatch& patch : partial_patches) {
      bool overlaps = false;
      for (const PartialRecoveryPatch& selected : selected_patches) {
        if (!(patch.end < selected.begin || selected.end < patch.begin)) {
          overlaps = true;
          break;
        }
      }
      if (!overlaps) {
        selected_patches.push_back(patch);
      }
    }

    std::sort(selected_patches.begin(), selected_patches.end(),
              [](const PartialRecoveryPatch& lhs, const PartialRecoveryPatch& rhs) {
                return lhs.begin < rhs.begin;
              });

    Trajectory rebuilt;
    size_t segment_cursor = 0;
    int rebuilt_support_points = 0;
    int rebuilt_high_risk_segments = 0;
    int rebuilt_escalated_segments = 0;
    for (const PartialRecoveryPatch& patch : selected_patches) {
      for (size_t i = segment_cursor; i < patch.begin && i < segment_trajectories.size(); ++i) {
        AppendTrajectoryPieces(segment_trajectories[i], &rebuilt);
      }
      AppendTrajectoryPieces(patch.replacement, &rebuilt);
      rebuilt_support_points += CountSupportPoints(segments, segment_cursor, patch.begin);
      rebuilt_high_risk_segments +=
          CountHighRiskSegments(segments, segment_cursor, patch.begin);
      rebuilt_support_points += patch.support_points;
      rebuilt_high_risk_segments += patch.high_risk_segments;
      rebuilt_escalated_segments =
          std::max(rebuilt_escalated_segments, patch.escalated_segments);
      segment_cursor = patch.end + 1;
    }
    for (size_t i = segment_cursor; i < segment_trajectories.size(); ++i) {
      AppendTrajectoryPieces(segment_trajectories[i], &rebuilt);
    }
    rebuilt_support_points += CountSupportPoints(segments, segment_cursor, segment_count);
    rebuilt_high_risk_segments +=
        CountHighRiskSegments(segments, segment_cursor, segment_count);

    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    if (!rebuilt.empty() &&
        runtime_->IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
      ROS_INFO(
          "Local bottleneck rebuild accepted via multi_window_combine: patches=%zu clearance %.3f -> %.3f pieces %zu -> %zu",
          selected_patches.size(), worst.clearance, min_clearance,
          current_traj.pos_pieces.size(), rebuilt.pos_pieces.size());
      best = *result;
      best.trajectory = rebuilt;
      best.min_clearance = min_clearance;
      best.max_vel = max_vel;
      best.max_acc = max_acc;
      best.support_points = rebuilt_support_points;
      best.high_risk_segments = rebuilt_high_risk_segments;
      best.escalated_segments =
          std::max(best.escalated_segments, rebuilt_escalated_segments);
      found = true;
    }
  }

  if (!found) {
    ROS_WARN(
        "Local bottleneck rebuild failed near seg %zu at t=%.3f state=(%.3f, %.3f, %.3f) clearance=%.3f",
        culprit_index, worst.time, worst.state.x, worst.state.y, worst.state.yaw,
        worst.clearance);
    return false;
  }

  *result = best;
  return true;
}

}  // namespace isweep_planner
