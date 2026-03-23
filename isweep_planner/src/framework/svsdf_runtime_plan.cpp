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
SvsdfPlanResult SvsdfRuntime::Plan(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& goal) {
  SvsdfPlanResult result;
  if (!map_ready_) {
    return result;
  }

  const SE2State start_state = ToSe2State(start);
  const SE2State goal_state = ToSe2State(goal);

  const ros::WallTime total_start = ros::WallTime::now();
  const ros::WallTime search_start = ros::WallTime::now();
  const Eigen::Vector2d start_2d(start_state.x, start_state.y);
  const Eigen::Vector2d goal_2d(goal_state.x, goal_state.y);

  const double topo_safe_dist = 0.25; // Ensure connectivity for topological roadmap
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_, topo_safe_dist);
  topology_planner_.buildRoadmap(start_2d, goal_2d);
  std::vector<TopoPath> topo_paths = topology_planner_.searchPaths();
  
  if (!topo_paths.empty()) {
    const size_t raw_topology_count = topo_paths.size();
    const size_t max_eval_paths =
        raw_topology_count > 3 ? 2 : std::min<size_t>(raw_topology_count, 3);
    if (topo_paths.size() > max_eval_paths) {
      topo_paths.resize(max_eval_paths);
    }
    ROS_INFO(
        "Runtime stage 1: Found %zu topological paths, shortening %zu selected candidates (Algorithm 1)",
        raw_topology_count, topo_paths.size());
    topology_planner_.shortenPaths(topo_paths);
    ROS_INFO("Runtime stage 1: evaluating %zu shortened paths (from %zu raw)",
             topo_paths.size(), raw_topology_count);
  }

  std::vector<SE2State> hybrid_path;
  if (topo_paths.empty()) {
    ROS_WARN("Topology search failed, trying Hybrid A* fallback");
    if (hybrid_astar_.plan(start_state, goal_state, hybrid_path)) {
      result.coarse_path = RosVisualizer::MakePath(hybrid_path);
      result.stats.coarse_path_points = static_cast<int>(hybrid_path.size());
    }
  }

  if (result.coarse_path.poses.empty() && !topo_paths.empty()) {
    result.coarse_path = RosVisualizer::MakePathFromTopo(topo_paths.front());
    result.stats.coarse_path_points =
        static_cast<int>(topo_paths.front().waypoints.size());
  }
  result.stats.search_time = (ros::WallTime::now() - search_start).toSec();

  const ros::WallTime optimize_start = ros::WallTime::now();
  const double optimize_timeout_sec =
      topo_paths.size() > 1 ? 20.0 : 15.0;
  const ros::WallTime optimize_deadline =
      optimize_start + ros::WallDuration(optimize_timeout_sec);
  double total_segment_solve_time = 0.0;
  double total_full_feasibility_time = 0.0;
  double total_tail_refine_time = 0.0;
  double total_recovery_time = 0.0;
  std::vector<CandidateResult> candidate_results;
  std::vector<CandidateResult> degraded_results;
  candidate_results.reserve(topo_paths.size());

  // Stage A: parallel SE2 sequence generation + waypoint push
  struct PreprocessedCandidate {
    std::vector<MotionSegment> segments;
    bool valid = false;
  };
  std::vector<PreprocessedCandidate> preprocessed(topo_paths.size());
  const ros::WallTime preprocess_start = ros::WallTime::now();

  #pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < topo_paths.size(); ++i) {
    std::vector<MotionSegment> raw_segments =
        se2_generator_.generate(topo_paths[i], start_state, goal_state);
    if (raw_segments.empty()) {
      continue;
    }
    PushSegmentWaypointsFromObstacles(&raw_segments);
    preprocessed[i].segments = std::move(raw_segments);
    preprocessed[i].valid = true;
  }
  const double preprocess_time =
      (ros::WallTime::now() - preprocess_start).toSec();
  result.stats.preprocess_time = preprocess_time;
  size_t preprocessed_valid = 0;
  for (const PreprocessedCandidate& candidate : preprocessed) {
    if (candidate.valid) {
      ++preprocessed_valid;
    }
  }
  ROS_INFO(
      "Plan() stage 2: preprocessed %zu/%zu candidates into motion segments in %.3fs",
      preprocessed_valid, preprocessed.size(), preprocess_time);

  // Stage B: serial optimization + early pruning
  for (size_t i = 0; i < preprocessed.size(); ++i) {
    if (!preprocessed[i].valid) {
      ROS_WARN("Path %zu discarded: no motion segments", i);
      continue;
    }
    const double elapsed = (ros::WallTime::now() - optimize_start).toSec();
    if (elapsed > optimize_timeout_sec) {
      ROS_WARN("Plan() optimization timeout after %.1fs, evaluated %zu/%zu paths",
               elapsed, i, topo_paths.size());
      break;
    }
    // Early pruning: skip candidates with many more segments than best so far
    if (!candidate_results.empty() &&
        preprocessed[i].segments.size() >
            2 * candidate_results[0].trajectory.pos_pieces.size()) {
      ROS_INFO("Path %zu pruned: too many segments (%zu)", i, preprocessed[i].segments.size());
      continue;
    }

    const CandidateResult candidate =
        EvaluateCandidate(preprocessed[i].segments, optimize_deadline);
    total_segment_solve_time += candidate.segment_solve_time;
    total_full_feasibility_time += candidate.full_feasibility_time;
    total_tail_refine_time += candidate.tail_refine_time;
    total_recovery_time += candidate.recovery_time;
    if (candidate.budget_exhausted) {
      ROS_WARN(
          "Path %zu reached the optimization deadline during evaluation: stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
          i, candidate.segment_solve_time, candidate.full_feasibility_time,
          candidate.tail_refine_time, candidate.recovery_time,
          candidate.tail_attempts, candidate.tail_pruned);
    }
    if (candidate.trajectory.empty()) {
      if (candidate.budget_exhausted && DeadlineExpired(optimize_deadline)) {
        ROS_WARN(
            "Path %zu stopped by the optimization deadline before producing a usable trajectory",
            i);
        break;
      }
      ROS_WARN("Path %zu discarded: infeasible segment or stitched trajectory", i);
      continue;
    }

    if (candidate.degraded) {
      ROS_WARN(
          "Path %zu degraded: clearance=%.3f (kept as fallback), stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
          i, candidate.min_clearance, candidate.segment_solve_time,
          candidate.full_feasibility_time, candidate.tail_refine_time,
          candidate.recovery_time, candidate.tail_attempts, candidate.tail_pruned);
      degraded_results.push_back(candidate);
      if (DeadlineExpired(optimize_deadline)) {
        break;
      }
      continue;
    }

    ROS_INFO(
        "Path %zu accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f, escalated=%d, stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
        i, candidate.min_clearance, candidate.max_vel, candidate.max_acc,
        candidate.escalated_segments, candidate.segment_solve_time,
        candidate.full_feasibility_time, candidate.tail_refine_time,
        candidate.recovery_time, candidate.tail_attempts, candidate.tail_pruned);
    candidate_results.push_back(candidate);
    if (DeadlineExpired(optimize_deadline)) {
      break;
    }
  }

  if (candidate_results.empty() && hybrid_path.size() >= 2) {
    ROS_WARN("All topological candidates failed, retrying with Hybrid A* corridor.");
    TopoPath hybrid_topo;
    hybrid_topo.waypoints.reserve(hybrid_path.size());
    for (size_t i = 0; i < hybrid_path.size(); ++i) {
      hybrid_topo.waypoints.push_back(
          TopoWaypoint(Eigen::Vector2d(hybrid_path[i].x, hybrid_path[i].y),
                       hybrid_path[i].yaw));
    }
    hybrid_topo.computeLength();

    const std::vector<MotionSegment> segments =
        se2_generator_.generate(hybrid_topo, start_state, goal_state);
    if (!segments.empty()) {
      const CandidateResult candidate = EvaluateCandidate(segments, optimize_deadline);
      total_segment_solve_time += candidate.segment_solve_time;
      total_full_feasibility_time += candidate.full_feasibility_time;
      total_tail_refine_time += candidate.tail_refine_time;
      total_recovery_time += candidate.recovery_time;
      if (!candidate.trajectory.empty()) {
        ROS_INFO("Hybrid fallback accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f",
                 candidate.min_clearance, candidate.max_vel, candidate.max_acc);
        candidate_results.push_back(candidate);
      }
    }
  }

  if (candidate_results.empty() && hybrid_path.size() >= 2) {
    const double total_time = 1.5 * EstimateSegmentTime(hybrid_path);
    const Trajectory hybrid_traj = optimizer_.optimizeSE2(hybrid_path, total_time);
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    if (!hybrid_traj.empty() &&
        IsTrajectoryFeasible(hybrid_traj, &min_clearance, &max_vel, &max_acc)) {
      ROS_INFO("Monolithic Hybrid fallback accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f",
               min_clearance, max_vel, max_acc);
      CandidateResult candidate;
      candidate.trajectory = hybrid_traj;
      candidate.min_clearance = min_clearance;
      candidate.max_vel = max_vel;
      candidate.max_acc = max_acc;
      candidate.support_points = static_cast<int>(hybrid_path.size());
      candidate_results.push_back(candidate);
    }
  }

  result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
  result.stats.segment_solve_time = total_segment_solve_time;
  result.stats.full_feasibility_time = total_full_feasibility_time;
  result.stats.tail_refine_time = total_tail_refine_time;
  result.stats.recovery_time = total_recovery_time;

  const auto log_final_stage_totals = [&](const char* outcome) {
    ROS_INFO(
        "Plan() final stage totals [%s]: topology=%.3fs preprocess=%.3fs segment_solve=%.3fs full_feasibility=%.3fs tail_refinement=%.3fs recovery=%.3fs optimize_total=%.3fs total=%.3fs",
        outcome, result.stats.search_time, result.stats.preprocess_time,
        result.stats.segment_solve_time, result.stats.full_feasibility_time,
        result.stats.tail_refine_time, result.stats.recovery_time,
        result.stats.optimization_time, result.stats.total_solve_time);
  };

  // If no strictly feasible candidate, use the best degraded one
  if (candidate_results.empty() && !degraded_results.empty()) {
    size_t best_idx = 0;
    double best_clearance = -kInf;
    for (size_t i = 0; i < degraded_results.size(); ++i) {
      if (degraded_results[i].min_clearance > best_clearance) {
        best_clearance = degraded_results[i].min_clearance;
        best_idx = i;
      }
    }
    ROS_WARN("Using best degraded candidate: clearance=%.3f", best_clearance);
    candidate_results.push_back(degraded_results[best_idx]);
  }

  if (candidate_results.empty()) {
    if (hybrid_path.size() >= 2) {
      double fallback_clearance = kInf;
      for (size_t i = 0; i < hybrid_path.size(); ++i) {
        fallback_clearance = std::min(
            fallback_clearance,
            grid_map_.getEsdf(hybrid_path[i].x, hybrid_path[i].y));
      }
      if (std::isfinite(fallback_clearance) && fallback_clearance > 0.0) {
        ROS_WARN("Falling back to centerline-safe Hybrid A* path: clearance=%.3f",
                 fallback_clearance);
        result.success = true;
        result.trajectory = RosVisualizer::MakePath(hybrid_path);
        result.stats.min_clearance = fallback_clearance;
        result.stats.support_points = static_cast<int>(hybrid_path.size());
        result.stats.local_obstacle_points = 0;
        result.stats.optimizer_iterations = 0;
        result.stats.segment_solve_time = total_segment_solve_time;
        result.stats.full_feasibility_time = total_full_feasibility_time;
        result.stats.tail_refine_time = total_tail_refine_time;
        result.stats.recovery_time = total_recovery_time;
        result.stats.optimization_time =
            (ros::WallTime::now() - optimize_start).toSec();
        result.stats.total_solve_time =
            (ros::WallTime::now() - total_start).toSec();
        log_final_stage_totals("centerline_fallback");
        return result;
      }
    }
    result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    log_final_stage_totals("failed");
    return result;
  }

  const ros::WallTime selection_start = ros::WallTime::now();
  size_t best_index = 0;
  bool found_index = false;
  double best_clearance = -kInf;
  for (const CandidateResult& candidate : candidate_results) {
    if (!candidate.trajectory.empty()) {
      best_clearance = std::max(best_clearance, candidate.min_clearance);
    }
  }
  const double clearance_band =
      std::isfinite(best_clearance)
          ? std::max(0.005, 0.10 * std::max(0.05, best_clearance))
          : 0.005;

  bool best_clearance_close = false;
  double best_smoothness = kInf;
  size_t best_pieces = std::numeric_limits<size_t>::max();
  double best_duration = kInf;
  for (size_t i = 0; i < candidate_results.size(); ++i) {
    const CandidateResult& candidate = candidate_results[i];
    if (candidate.trajectory.empty()) {
      continue;
    }

    const bool clearance_close =
        std::isfinite(best_clearance) &&
        candidate.min_clearance + 1e-6 >= best_clearance - clearance_band;
    const double smoothness = TrajectorySmoothnessCost(candidate.trajectory);
    const size_t pieces = candidate.trajectory.pos_pieces.size();
    const double duration = candidate.trajectory.totalDuration();

    bool better = false;
    if (!found_index) {
      better = true;
    } else if (clearance_close != best_clearance_close) {
      better = clearance_close;
    } else if (smoothness + 1e-6 < best_smoothness) {
      better = true;
    } else if (std::abs(smoothness - best_smoothness) <= 1e-6 && pieces < best_pieces) {
      better = true;
    } else if (std::abs(smoothness - best_smoothness) <= 1e-6 &&
               pieces == best_pieces && duration + 1e-6 < best_duration) {
      better = true;
    } else if (std::abs(smoothness - best_smoothness) <= 1e-6 &&
               pieces == best_pieces && std::abs(duration - best_duration) <= 1e-6 &&
               candidate.min_clearance > candidate_results[best_index].min_clearance) {
      better = true;
    }

    if (!better) {
      continue;
    }
    best_index = i;
    found_index = true;
    best_clearance_close = clearance_close;
    best_smoothness = smoothness;
    best_pieces = pieces;
    best_duration = duration;
  }

  if (!found_index) {
    result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    log_final_stage_totals("no_best_candidate");
    return result;
  }

  const double selection_time = (ros::WallTime::now() - selection_start).toSec();
  CandidateResult final_candidate = candidate_results[best_index];
  Trajectory best = final_candidate.trajectory;
  ROS_INFO(
      "Plan() candidate selection: considered=%zu best=%zu clearance=%.3f smooth=%.3f pieces=%zu duration=%.3f selection=%.3fs",
      candidate_results.size(), best_index, final_candidate.min_clearance,
      best_smoothness, best.pos_pieces.size(), best.totalDuration(), selection_time);
  if (!final_candidate.segments.empty() && !final_candidate.segment_trajectories.empty()) {
    const ros::WallTime tail_start = ros::WallTime::now();
    final_candidate.trajectory = best;
    RecoveryManager recovery(this);
    recovery.MaybeImproveTailTrajectory(final_candidate.segments, final_candidate.segment_trajectories,
                                        optimize_deadline, &final_candidate);
    const double tail_elapsed = (ros::WallTime::now() - tail_start).toSec();
    final_candidate.tail_refine_time += tail_elapsed;
    total_tail_refine_time += tail_elapsed;
    best = final_candidate.trajectory;
  }

  result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
  result.success = true;
  
  ROS_INFO("Plan(): best traj pieces=%zu duration=%.3f, sampling first piece c0=(%.4f,%.4f) c1=(%.4f,%.4f)",
           best.pos_pieces.size(), best.totalDuration(),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,1),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,1));
  result.trajectory = RosVisualizer::MakeTrajectoryPath(best);
  result.raw_trajectory = best;
  ROS_INFO("Plan(): published trajectory poses=%zu", result.trajectory.poses.size());
  result.stats.min_clearance = final_candidate.min_clearance;
  result.stats.support_points = final_candidate.support_points;
  result.stats.local_obstacle_points = final_candidate.high_risk_segments;
  result.stats.optimizer_iterations = 0;
  result.stats.segment_solve_time = total_segment_solve_time;
  result.stats.full_feasibility_time = total_full_feasibility_time;
  result.stats.tail_refine_time = total_tail_refine_time;
  result.stats.recovery_time = total_recovery_time;
  result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
  const double optimization_other_time = std::max(
      0.0, result.stats.optimization_time - preprocess_time -
               total_segment_solve_time - total_full_feasibility_time -
               total_tail_refine_time - total_recovery_time - selection_time);
  ROS_INFO(
      "Plan() timing breakdown: topology=%.3fs preprocess=%.3fs segment_solve=%.3fs full_feasibility=%.3fs tail_refinement=%.3fs recovery=%.3fs selection=%.3fs optimize_other=%.3fs optimize_total=%.3fs",
      result.stats.search_time, preprocess_time, total_segment_solve_time,
      total_full_feasibility_time, total_tail_refine_time,
      total_recovery_time, selection_time, optimization_other_time,
      result.stats.optimization_time);
  log_final_stage_totals("success");
  return result;
}
} // namespace isweep_planner
