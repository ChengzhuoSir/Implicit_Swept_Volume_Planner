#include "esv_planner/paper_planner.h"

#include <esv_planner/minco_parameterization.h>
#include <esv_planner/trajectory_sampling.h>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>

namespace esv_planner {

namespace {

std::vector<SE2State> buildDenseSupport(const std::vector<Eigen::Vector2d>& path,
                                        const SE2State& start,
                                        const SE2State& goal,
                                        int target_points) {
  std::vector<SE2State> support;
  if (path.size() < 2) {
    return support;
  }

  const int clamped_target = std::max(2, std::min(target_points, static_cast<int>(path.size())));
  support.reserve(static_cast<size_t>(clamped_target));
  const double last_index = static_cast<double>(path.size() - 1);
  const double denom = static_cast<double>(clamped_target - 1);
  for (int i = 0; i < clamped_target; ++i) {
    const size_t idx = static_cast<size_t>(
        std::lround(last_index * static_cast<double>(i) / denom));
    const Eigen::Vector2d& point = path[std::min(idx, path.size() - 1)];
    support.push_back(SE2State(point.x(), point.y(), 0.0));
  }

  support.front() = start;
  support.back() = goal;
  for (size_t i = 1; i + 1 < support.size(); ++i) {
    const Eigen::Vector2d delta = support[i + 1].position() - support[i - 1].position();
    support[i].yaw =
        delta.norm() > 1e-6 ? std::atan2(delta.y(), delta.x()) : support[i - 1].yaw;
  }
  return support;
}

Trajectory buildTrajectoryFromSupport(const std::vector<SE2State>& support,
                                      double nominal_speed,
                                      double min_piece_duration) {
  if (support.size() < 2) {
    return Trajectory();
  }

  std::vector<double> durations;
  durations.reserve(support.size() - 1);
  for (size_t i = 1; i < support.size(); ++i) {
    const double distance = (support[i].position() - support[i - 1].position()).norm();
    durations.push_back(std::max(min_piece_duration, distance / std::max(0.1, nominal_speed)));
  }
  const Eigen::VectorXd vars = MincoParameterization::packSe2StateVariables(support);
  return MincoParameterization::buildSe2TrajectoryFixedTime(
      vars, durations, support.front(), support.back());
}

Trajectory buildPiecewiseLinearTrajectory(const std::vector<SE2State>& support,
                                          double nominal_speed,
                                          double min_piece_duration) {
  Trajectory trajectory;
  if (support.size() < 2) {
    return trajectory;
  }

  trajectory.pos_pieces.reserve(support.size() - 1);
  trajectory.yaw_pieces.reserve(support.size() - 1);
  for (size_t i = 1; i < support.size(); ++i) {
    const Eigen::Vector2d delta = support[i].position() - support[i - 1].position();
    const double duration = std::max(
        min_piece_duration, delta.norm() / std::max(0.1, nominal_speed));

    PolyPiece pos_piece;
    pos_piece.duration = duration;
    pos_piece.coeffs.col(0) = support[i - 1].position();
    pos_piece.coeffs.col(1) = delta / duration;
    trajectory.pos_pieces.push_back(pos_piece);

    YawPolyPiece yaw_piece;
    yaw_piece.duration = duration;
    yaw_piece.coeffs(0, 0) = support[i - 1].yaw;
    yaw_piece.coeffs(0, 1) =
        normalizeAngle(support[i].yaw - support[i - 1].yaw) / duration;
    trajectory.yaw_pieces.push_back(yaw_piece);
  }

  return trajectory;
}

void refreshSupportYaw(std::vector<SE2State>* support) {
  if (!support || support->size() < 2) {
    return;
  }
  for (size_t i = 1; i + 1 < support->size(); ++i) {
    const Eigen::Vector2d delta =
        (*support)[i + 1].position() - (*support)[i - 1].position();
    if (delta.norm() > 1e-6) {
      (*support)[i].yaw = std::atan2(delta.y(), delta.x());
    } else {
      (*support)[i].yaw = (*support)[i - 1].yaw;
    }
  }
}

struct ClearanceRepairResult {
  std::vector<SE2State> support;
  Trajectory trajectory;
  double clearance = -kInf;
};

ClearanceRepairResult repairTrajectoryClearance(
    const std::vector<SE2State>& initial_support,
    const Trajectory& initial_traj,
    const SvsdfEvaluator& svsdf,
    double nominal_speed,
    double min_piece_duration) {
  ClearanceRepairResult result;
  result.support = initial_support;
  result.trajectory = initial_traj;
  result.clearance = initial_traj.empty() ? -kInf : svsdf.evaluateTrajectory(initial_traj);
  if (result.support.size() < 3 || result.clearance >= 0.0) {
    return result;
  }

  for (int round = 0; round < 4; ++round) {
    const std::vector<SE2State> samples =
        sampleTrajectoryByArcLength(result.trajectory, 0.05, 0.02);
    if (samples.empty()) {
      break;
    }

    double worst_clearance = kInf;
    size_t worst_index = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
      const double clearance = svsdf.evaluate(samples[i]);
      if (clearance < worst_clearance) {
        worst_clearance = clearance;
        worst_index = i;
      }
    }
    result.clearance = worst_clearance;
    if (worst_clearance >= 0.0) {
      break;
    }

    Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
    double grad_yaw = 0.0;
    svsdf.gradient(samples[worst_index], grad_pos, grad_yaw);
    if (grad_pos.norm() < 1e-6) {
      break;
    }

    size_t best_support_index = 1;
    double best_distance = kInf;
    for (size_t i = 1; i + 1 < result.support.size(); ++i) {
      const double distance =
          (result.support[i].position() - samples[worst_index].position()).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_support_index = i;
      }
    }

    const double step = std::min(0.10, -worst_clearance + 0.02);
    const Eigen::Vector2d direction = grad_pos.normalized();
    result.support[best_support_index].x += step * direction.x();
    result.support[best_support_index].y += step * direction.y();
    if (best_support_index > 1) {
      result.support[best_support_index - 1].x += 0.35 * step * direction.x();
      result.support[best_support_index - 1].y += 0.35 * step * direction.y();
    }
    if (best_support_index + 2 < result.support.size()) {
      result.support[best_support_index + 1].x += 0.35 * step * direction.x();
      result.support[best_support_index + 1].y += 0.35 * step * direction.y();
    }
    refreshSupportYaw(&result.support);
    result.trajectory = buildTrajectoryFromSupport(
        result.support, nominal_speed, min_piece_duration);
    if (result.trajectory.empty()) {
      break;
    }
    result.clearance = svsdf.evaluateTrajectory(result.trajectory);
  }

  return result;
}

}  // namespace

void PaperPlanner::init(const GridMap& map,
                        const FootprintModel& footprint,
                        const SvsdfEvaluator& svsdf,
                        const PaperPlannerParams& params) {
  map_ = &map;
  footprint_ = &footprint;
  svsdf_ = &svsdf;
  params_ = params;
  obstacle_source_.init(map);
  astar_.init(map, params.front_end_clearance);
  sparsifier_.init(map, params.line_of_sight_clearance, params.max_segment_length,
                   params.max_support_points);
  backend_.init(map, footprint, svsdf, params.backend);
}

PaperPlannerResult PaperPlanner::plan(const SE2State& start,
                                      const SE2State& goal) const {
  PaperPlannerResult result;
  if (!map_ || !footprint_ || !svsdf_) {
    return result;
  }

  const ros::Time t0 = ros::Time::now();
  const AstarSearchResult search = astar_.search(start.position(), goal.position());
  result.stats.search_time = (ros::Time::now() - t0).toSec();
  result.stats.coarse_path_points = static_cast<int>(search.path.size());
  result.coarse_path = search.path;
  if (!search.success || search.path.size() < 2) {
    result.stats.total_solve_time = (ros::Time::now() - t0).toSec();
    return result;
  }

  result.support_states = sparsifier_.sparsify(search.path, start, goal);
  result.stats.support_points = static_cast<int>(result.support_states.size());
  if (result.support_states.size() < 2) {
    result.stats.total_solve_time = (ros::Time::now() - t0).toSec();
    return result;
  }

  LocalObstacleCache cache;
  cache.build(obstacle_source_, result.support_states, params_.local_aabb_half_extent);
  result.local_obstacles = cache.points();
  result.stats.local_obstacle_points =
      static_cast<int>(result.local_obstacles.size());

  const ros::Time t_opt = ros::Time::now();
  PaperBackendResult backend_result =
      backend_.optimize(result.support_states, result.local_obstacles);
  result.stats.optimization_time = (ros::Time::now() - t_opt).toSec();
  result.stats.optimizer_iterations = backend_result.iterations;
  result.support_states = backend_result.support_states;
  result.trajectory = backend_result.trajectory;
  result.stats.min_clearance = backend_result.min_clearance;

  if (result.stats.min_clearance < 0.0) {
    const Trajectory polyline_traj = buildPiecewiseLinearTrajectory(
        result.support_states,
        params_.backend.nominal_speed,
        params_.backend.min_piece_duration);
    const double polyline_clearance =
        polyline_traj.empty() ? -kInf : svsdf_->evaluateTrajectory(polyline_traj);
    if (polyline_clearance > result.stats.min_clearance) {
      result.trajectory = polyline_traj;
      result.stats.min_clearance = polyline_clearance;
    }

    const ClearanceRepairResult repaired = repairTrajectoryClearance(
        result.support_states,
        result.trajectory,
        *svsdf_,
        params_.backend.nominal_speed,
        params_.backend.min_piece_duration);
    if (repaired.clearance > result.stats.min_clearance) {
      result.support_states = repaired.support;
      result.trajectory = repaired.trajectory;
      result.stats.support_points = static_cast<int>(repaired.support.size());
      result.stats.min_clearance = repaired.clearance;
    }

    const std::vector<SE2State> repaired_support = buildDenseSupport(
        search.path, start, goal,
        std::max(48, result.stats.support_points + 8));
    const Trajectory repaired_traj = buildTrajectoryFromSupport(
        repaired_support,
        params_.backend.nominal_speed,
        params_.backend.min_piece_duration);
    const double repaired_clearance =
        repaired_traj.empty() ? -kInf : svsdf_->evaluateTrajectory(repaired_traj);
    if (repaired_clearance > result.stats.min_clearance) {
      result.support_states = repaired_support;
      result.trajectory = repaired_traj;
      result.stats.support_points = static_cast<int>(repaired_support.size());
      result.stats.min_clearance = repaired_clearance;
    }

    const Trajectory dense_polyline_traj = buildPiecewiseLinearTrajectory(
        repaired_support,
        params_.backend.nominal_speed,
        params_.backend.min_piece_duration);
    const double dense_polyline_clearance =
        dense_polyline_traj.empty() ? -kInf : svsdf_->evaluateTrajectory(dense_polyline_traj);
    if (dense_polyline_clearance > result.stats.min_clearance) {
      result.support_states = repaired_support;
      result.trajectory = dense_polyline_traj;
      result.stats.support_points = static_cast<int>(repaired_support.size());
      result.stats.min_clearance = dense_polyline_clearance;
    }
  }

  if (result.trajectory.empty()) {
    const std::vector<double> fallback_durations(
        std::max<size_t>(1, result.support_states.size() - 1),
        params_.backend.min_piece_duration);
    if (!result.support_states.empty()) {
      const Eigen::VectorXd se2_vars =
          MincoParameterization::packSe2StateVariables(result.support_states);
      result.trajectory = MincoParameterization::buildSe2TrajectoryFixedTime(
          se2_vars, fallback_durations, result.support_states.front(),
          result.support_states.back());
    }
    result.stats.min_clearance =
        result.trajectory.empty() ? -kInf : svsdf_->evaluateTrajectory(result.trajectory);
  }

  result.stats.total_solve_time = (ros::Time::now() - t0).toSec();
  result.success = !result.trajectory.empty() && result.stats.min_clearance >= 0.0;
  return result;
}

}  // namespace esv_planner
