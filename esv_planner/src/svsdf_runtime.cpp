#include "esv_planner/svsdf_runtime.h"
#include "esv_planner/swept_astar.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include <geometry_msgs/PoseStamped.h>

namespace esv_planner {

namespace {

struct ClearanceProbe {
  bool valid = false;
  double time = 0.0;
  double clearance = kInf;
  SE2State state;
};

double WaypointPathLength(const std::vector<SE2State>& waypoints) {
  double length = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    length += (waypoints[i].position() - waypoints[i - 1].position()).norm();
  }
  return length;
}

void AppendUniqueWaypoints(const std::vector<SE2State>& source,
                           std::vector<SE2State>* target) {
  if (target == nullptr || source.empty()) {
    return;
  }
  if (target->empty()) {
    *target = source;
    return;
  }

  size_t begin = 0;
  if ((target->back().position() - source.front().position()).norm() < 1e-6 &&
      std::abs(normalizeAngle(target->back().yaw - source.front().yaw)) < 1e-6) {
    begin = 1;
  }
  target->insert(target->end(), source.begin() + static_cast<long>(begin), source.end());
}

std::vector<SE2State> BuildLinearWaypoints(const SE2State& start, const SE2State& goal,
                                           double step) {
  std::vector<SE2State> waypoints;
  waypoints.push_back(start);

  const Eigen::Vector2d delta = goal.position() - start.position();
  const double length = delta.norm();
  if (length < 1e-9) {
    waypoints.push_back(goal);
    return waypoints;
  }

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(1e-3, step))));
  for (int s = 1; s < samples; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(samples);
    const Eigen::Vector2d position = start.position() + t * delta;
    waypoints.push_back(SE2State(position.x(), position.y(), tangent_yaw));
  }
  waypoints.push_back(goal);
  return waypoints;
}

Trajectory ConcatenateTrajectories(const std::vector<Trajectory>& segment_trajectories,
                                   size_t prefix_count, const Trajectory& suffix) {
  Trajectory result;
  for (size_t i = 0; i < prefix_count && i < segment_trajectories.size(); ++i) {
    const Trajectory& prefix = segment_trajectories[i];
    result.pos_pieces.insert(result.pos_pieces.end(), prefix.pos_pieces.begin(),
                             prefix.pos_pieces.end());
    result.yaw_pieces.insert(result.yaw_pieces.end(), prefix.yaw_pieces.begin(),
                             prefix.yaw_pieces.end());
  }
  result.pos_pieces.insert(result.pos_pieces.end(), suffix.pos_pieces.begin(),
                           suffix.pos_pieces.end());
  result.yaw_pieces.insert(result.yaw_pieces.end(), suffix.yaw_pieces.begin(),
                           suffix.yaw_pieces.end());
  return result;
}

Trajectory ConcatenateTrajectories(const std::vector<Trajectory>& segment_trajectories,
                                   size_t prefix_count, size_t suffix_begin,
                                   const Trajectory& middle) {
  Trajectory result;
  for (size_t i = 0; i < prefix_count && i < segment_trajectories.size(); ++i) {
    const Trajectory& prefix = segment_trajectories[i];
    result.pos_pieces.insert(result.pos_pieces.end(), prefix.pos_pieces.begin(),
                             prefix.pos_pieces.end());
    result.yaw_pieces.insert(result.yaw_pieces.end(), prefix.yaw_pieces.begin(),
                             prefix.yaw_pieces.end());
  }
  result.pos_pieces.insert(result.pos_pieces.end(), middle.pos_pieces.begin(),
                           middle.pos_pieces.end());
  result.yaw_pieces.insert(result.yaw_pieces.end(), middle.yaw_pieces.begin(),
                           middle.yaw_pieces.end());
  for (size_t i = suffix_begin; i < segment_trajectories.size(); ++i) {
    const Trajectory& suffix = segment_trajectories[i];
    result.pos_pieces.insert(result.pos_pieces.end(), suffix.pos_pieces.begin(),
                             suffix.pos_pieces.end());
    result.yaw_pieces.insert(result.yaw_pieces.end(), suffix.yaw_pieces.begin(),
                             suffix.yaw_pieces.end());
  }
  return result;
}

double ApproximateTrajectoryLength(const Trajectory& traj) {
  if (traj.empty()) {
    return kInf;
  }

  const double total = traj.totalDuration();
  double length = 0.0;
  SE2State prev = traj.sample(0.0);
  for (double t = 0.05; t <= total; t += 0.05) {
    const SE2State state = traj.sample(t);
    length += (state.position() - prev.position()).norm();
    prev = state;
  }
  const SE2State tail = traj.sample(total);
  length += (tail.position() - prev.position()).norm();
  return length;
}

int CountHighRiskSegments(const std::vector<MotionSegment>& segments, size_t end_index) {
  int count = 0;
  for (size_t i = 0; i < end_index && i < segments.size(); ++i) {
    if (segments[i].risk == RiskLevel::HIGH) {
      ++count;
    }
  }
  return count;
}

int CountHighRiskSegments(const std::vector<MotionSegment>& segments, size_t begin_index,
                          size_t end_index) {
  int count = 0;
  for (size_t i = begin_index; i < end_index && i < segments.size(); ++i) {
    if (segments[i].risk == RiskLevel::HIGH) {
      ++count;
    }
  }
  return count;
}

int CountSupportPoints(const std::vector<MotionSegment>& segments, size_t begin_index,
                       size_t end_index) {
  int count = 0;
  for (size_t i = begin_index; i < end_index && i < segments.size(); ++i) {
    count += static_cast<int>(segments[i].waypoints.size());
  }
  return count;
}

ClearanceProbe FindWorstClearanceSample(const Trajectory& traj,
                                        const SvsdfEvaluator& evaluator) {
  ClearanceProbe probe;
  if (traj.empty()) {
    return probe;
  }

  const double total = traj.totalDuration();
  const double dt = 0.02;
  for (double t = 0.0; t <= total + 1e-9; t += dt) {
    const double sample_time = std::min(t, total);
    const SE2State state = traj.sample(sample_time);
    const double clearance = evaluator.evaluate(state);
    if (!probe.valid || clearance < probe.clearance) {
      probe.valid = true;
      probe.time = sample_time;
      probe.clearance = clearance;
      probe.state = state;
    }
  }
  return probe;
}

size_t SegmentIndexFromTime(const std::vector<Trajectory>& segment_trajectories, double time) {
  if (segment_trajectories.empty()) {
    return 0;
  }

  double accumulated = 0.0;
  for (size_t i = 0; i < segment_trajectories.size(); ++i) {
    accumulated += segment_trajectories[i].totalDuration();
    if (time <= accumulated + 1e-9 || i + 1 == segment_trajectories.size()) {
      return i;
    }
  }
  return segment_trajectories.size() - 1;
}

std::pair<double, double> SegmentTimeWindow(
    const std::vector<Trajectory>& segment_trajectories, size_t begin_index,
    size_t end_index) {
  double begin_time = 0.0;
  for (size_t i = 0; i < begin_index && i < segment_trajectories.size(); ++i) {
    begin_time += segment_trajectories[i].totalDuration();
  }

  double end_time = begin_time;
  for (size_t i = begin_index; i <= end_index && i < segment_trajectories.size(); ++i) {
    end_time += segment_trajectories[i].totalDuration();
  }
  return std::make_pair(begin_time, end_time);
}

std::vector<Eigen::Vector2d> SampleLowClearanceCorridor(
    const Trajectory& traj, const SvsdfEvaluator& evaluator, double begin_time,
    double end_time, double clearance_threshold, double min_spacing) {
  std::vector<Eigen::Vector2d> centers;
  if (traj.empty()) {
    return centers;
  }

  const double total = traj.totalDuration();
  const double dt = 0.05;
  const double spacing = std::max(0.05, min_spacing);
  const double sample_begin = std::max(0.0, begin_time);
  const double sample_end = std::min(total, end_time);
  for (double t = sample_begin; t <= sample_end + 1e-9; t += dt) {
    const SE2State state = traj.sample(std::min(t, total));
    const double clearance = evaluator.evaluate(state);
    if (!std::isfinite(clearance) || clearance > clearance_threshold) {
      continue;
    }

    if (!centers.empty() && (state.position() - centers.back()).norm() < spacing) {
      continue;
    }
    centers.push_back(state.position());
  }
  return centers;
}

double MinDistanceToCenters(const Eigen::Vector2d& point,
                            const std::vector<Eigen::Vector2d>& centers) {
  double min_distance = kInf;
  for (const Eigen::Vector2d& center : centers) {
    min_distance = std::min(min_distance, (point - center).norm());
  }
  return min_distance;
}

bool CorridorPositionAllowed(const GridMap& map, const CollisionChecker& checker,
                             const Eigen::Vector2d& point, double required_clearance,
                             const std::vector<Eigen::Vector2d>& blocked_centers,
                             double blocked_radius, int min_safe_yaw_count) {
  if (map.getEsdf(point.x(), point.y()) <= required_clearance) {
    return false;
  }
  if (min_safe_yaw_count > 0 &&
      static_cast<int>(checker.safeYawIndices(point.x(), point.y()).size()) <
          min_safe_yaw_count) {
    return false;
  }
  if (blocked_radius > 0.0 && !blocked_centers.empty() &&
      MinDistanceToCenters(point, blocked_centers) < blocked_radius) {
    return false;
  }
  return true;
}

bool CorridorLineFree(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                      const GridMap& map, const CollisionChecker& checker,
                      double required_clearance,
                      const std::vector<Eigen::Vector2d>& blocked_centers,
                      double blocked_radius, int min_safe_yaw_count) {
  const double distance = (goal - start).norm();
  const double step = std::max(0.02, 0.5 * map.resolution());
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d point = start + t * (goal - start);
    if (!CorridorPositionAllowed(map, checker, point, required_clearance,
                                 blocked_centers, blocked_radius,
                                 min_safe_yaw_count)) {
      return false;
    }
  }
  return true;
}

TopoPath BuildTopoPathFromCorridor(
    const std::vector<Eigen::Vector2d>& dense_path, const GridMap& map,
    const CollisionChecker& checker, double required_clearance,
    const std::vector<Eigen::Vector2d>& blocked_centers, double blocked_radius,
    int min_safe_yaw_count) {
  TopoPath topo_path;
  if (dense_path.size() < 2) {
    return topo_path;
  }

  topo_path.waypoints.emplace_back(dense_path.front());
  size_t anchor_index = 0;
  while (anchor_index + 1 < dense_path.size()) {
    size_t next_index = anchor_index + 1;
    for (size_t candidate = dense_path.size() - 1; candidate > anchor_index + 1; --candidate) {
      if (CorridorLineFree(dense_path[anchor_index], dense_path[candidate], map, checker,
                           required_clearance, blocked_centers, blocked_radius,
                           min_safe_yaw_count)) {
        next_index = candidate;
        break;
      }
    }
    topo_path.waypoints.emplace_back(dense_path[next_index]);
    anchor_index = next_index;
  }

  topo_path.computeLength();
  return topo_path;
}

std::vector<Eigen::Vector2d> LoadFootprint(ros::NodeHandle& pnh) {
  std::vector<double> flat;
  pnh.param("footprint", flat, std::vector<double>());
  std::vector<Eigen::Vector2d> polygon;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    polygon.push_back(Eigen::Vector2d(flat[i], flat[i + 1]));
  }
  if (polygon.empty()) {
    polygon.push_back(Eigen::Vector2d(0.20, 0.25));
    polygon.push_back(Eigen::Vector2d(0.20, -0.25));
    polygon.push_back(Eigen::Vector2d(-0.10, -0.25));
    polygon.push_back(Eigen::Vector2d(-0.10, -0.09));
    polygon.push_back(Eigen::Vector2d(-0.30, -0.09));
    polygon.push_back(Eigen::Vector2d(-0.30, 0.09));
    polygon.push_back(Eigen::Vector2d(-0.10, 0.09));
    polygon.push_back(Eigen::Vector2d(-0.10, 0.25));
  }
  return polygon;
}

SE2State ToSe2State(const Eigen::Vector3d& state) {
  return SE2State(state.x(), state.y(), state.z());
}

int CountSupportPoints(const std::vector<MotionSegment>& segments) {
  int count = 0;
  for (size_t i = 0; i < segments.size(); ++i) {
    count += static_cast<int>(segments[i].waypoints.size());
  }
  return count;
}

std::vector<SE2State> SparsifyStrictWaypoints(const std::vector<SE2State>& dense,
                                             const GridMap& map) {
  if (dense.size() <= 2) {
    return dense;
  }

  const double kOpenSpacing = 2.00;
  const double kTightSpacing = 0.60;
  const double kOpenYawDelta = 0.60;
  const double kTightYawDelta = 0.25;
  const double kOpenTurnDelta = 0.45;
  const double kTightTurnDelta = 0.20;
  const double kTightClearance = std::max(0.35, 6.0 * map.resolution());
  const double kCriticalClearance = std::max(0.20, 4.0 * map.resolution());
  const size_t kMaxPoints = 160;

  std::vector<SE2State> sparse;
  sparse.reserve(std::min(dense.size(), kMaxPoints));
  sparse.push_back(dense.front());

  double accumulated_distance = 0.0;
  bool tight_since_last = false;
  for (size_t i = 1; i + 1 < dense.size(); ++i) {
    const Eigen::Vector2d step = dense[i].position() - dense[i - 1].position();
    accumulated_distance += step.norm();
    const double clearance = map.getEsdf(dense[i].x, dense[i].y);
    tight_since_last = tight_since_last ||
                       (!std::isfinite(clearance) || clearance < kTightClearance);
    const double yaw_delta =
        std::abs(normalizeAngle(dense[i].yaw - sparse.back().yaw));

    double turn_delta = 0.0;
    const Eigen::Vector2d incoming = dense[i].position() - dense[i - 1].position();
    const Eigen::Vector2d outgoing = dense[i + 1].position() - dense[i].position();
    if (incoming.norm() > 1e-9 && outgoing.norm() > 1e-9) {
      turn_delta = std::abs(normalizeAngle(
          std::atan2(outgoing.y(), outgoing.x()) -
          std::atan2(incoming.y(), incoming.x())));
    }

    const double max_spacing = tight_since_last ? kTightSpacing : kOpenSpacing;
    const double max_yaw_delta = tight_since_last ? kTightYawDelta : kOpenYawDelta;
    const double max_turn_delta = tight_since_last ? kTightTurnDelta : kOpenTurnDelta;
    const bool critical_clearance =
        !std::isfinite(clearance) || clearance < kCriticalClearance;
    if (accumulated_distance >= max_spacing || yaw_delta >= max_yaw_delta ||
        turn_delta >= max_turn_delta ||
        (critical_clearance && accumulated_distance >= 0.30)) {
      sparse.push_back(dense[i]);
      accumulated_distance = 0.0;
      tight_since_last = critical_clearance;
    }
  }

  if ((sparse.back().position() - dense.back().position()).norm() > 1e-6) {
    sparse.push_back(dense.back());
  } else {
    sparse.back() = dense.back();
  }

  // Ensure at least 3 points (2 pieces) for the optimizer to have degrees of freedom
  if (sparse.size() < 3 && dense.size() >= 3) {
    sparse.clear();
    sparse.push_back(dense.front());
    // Pick the midpoint
    sparse.push_back(dense[dense.size() / 2]);
    sparse.push_back(dense.back());
  }

  if (sparse.size() <= kMaxPoints) {
    return sparse;
  }

  std::vector<SE2State> capped;
  capped.reserve(kMaxPoints);
  capped.push_back(sparse.front());
  const size_t interior_points = kMaxPoints - 2;
  const size_t available = sparse.size() - 2;
  for (size_t i = 0; i < interior_points; ++i) {
    const double alpha = static_cast<double>(i + 1) /
                         static_cast<double>(interior_points + 1);
    const size_t index = 1 + static_cast<size_t>(std::round(alpha * available));
    capped.push_back(sparse[std::min(index, sparse.size() - 2)]);
  }
  capped.push_back(sparse.back());
  return capped;
}

}  // namespace

void SvsdfRuntime::PushSegmentWaypointsFromObstacles(
    std::vector<MotionSegment>* segments) const {
  if (segments == nullptr) return;
  const double push_margin = 0.40;
  const double push_step = 0.12;
  const int max_iters = 20;
  const double h = grid_map_.resolution();

  for (auto& seg : *segments) {
    for (auto& wp : seg.waypoints) {
      for (int iter = 0; iter < max_iters; ++iter) {
        const double esdf = grid_map_.getEsdf(wp.x, wp.y);
        if (std::isfinite(esdf) && esdf >= push_margin) break;

        const double dEdx = (grid_map_.getEsdf(wp.x + h, wp.y)
                           - grid_map_.getEsdf(wp.x - h, wp.y)) / (2.0 * h);
        const double dEdy = (grid_map_.getEsdf(wp.x, wp.y + h)
                           - grid_map_.getEsdf(wp.x, wp.y - h)) / (2.0 * h);
        const double grad_norm = std::sqrt(dEdx * dEdx + dEdy * dEdy);

        if (grad_norm > 1e-6) {
          wp.x += push_step * dEdx / grad_norm;
          wp.y += push_step * dEdy / grad_norm;
        } else {
          bool found = false;
          for (double radius = h; radius <= 3.0; radius += h) {
            double best_esdf = -1.0;
            double best_dx = 0, best_dy = 0;
            for (int a = 0; a < 16; ++a) {
              const double angle = a * M_PI / 8.0;
              const double dx = radius * std::cos(angle);
              const double dy = radius * std::sin(angle);
              const double e = grid_map_.getEsdf(wp.x + dx, wp.y + dy);
              if (std::isfinite(e) && e > best_esdf) {
                best_esdf = e;
                best_dx = dx;
                best_dy = dy;
              }
            }
            if (best_esdf >= push_margin) {
              wp.x += best_dx;
              wp.y += best_dy;
              found = true;
              break;
            }
          }
          if (!found) break;
        }
      }
    }
  }

  for (size_t i = 0; i + 1 < segments->size(); ++i) {
    if (!(*segments)[i].waypoints.empty() && !(*segments)[i + 1].waypoints.empty()) {
      (*segments)[i + 1].waypoints.front() = (*segments)[i].waypoints.back();
    }
  }
}

void SvsdfRuntime::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  LoadParameters(pnh);
  strict_frontend_.Initialize(nh, pnh, footprint_);
  strict_solver_.Initialize(nh, pnh, footprint_);
  initialized_ = true;
}

void SvsdfRuntime::LoadParameters(ros::NodeHandle& pnh) {
  footprint_.setPolygon(LoadFootprint(pnh));

  double inscribed_radius = 0.08;
  pnh.param("inscribed_radius", inscribed_radius, 0.08);
  footprint_.setInscribedRadius(inscribed_radius);

  pnh.param("topology/num_samples", topology_num_samples_, 520);
  pnh.param("topology/knn", topology_knn_, 18);
  pnh.param("topology/max_paths", topology_max_paths_, 14);

  pnh.param("se2/discretization_step", se2_disc_step_, 0.15);
  pnh.param("kernel_yaw_num", yaw_bins_, 18);
  pnh.param("se2/max_push_attempts", max_push_attempts_, 5);

  pnh.param("optimizer/max_iterations", optimizer_params_.max_iterations, 100);
  pnh.param("optimizer/lambda_smooth", optimizer_params_.lambda_smooth, 1.0);
  pnh.param("optimizer/lambda_time", optimizer_params_.lambda_time, 1.0);
  pnh.param("optimizer/lambda_safety", optimizer_params_.lambda_safety, 10.0);
  pnh.param("optimizer/lambda_dynamics", optimizer_params_.lambda_dynamics, 1.0);
  pnh.param("optimizer/lambda_pos_residual",
            optimizer_params_.lambda_pos_residual, 5.0);
  pnh.param("optimizer/lambda_yaw_residual",
            optimizer_params_.lambda_yaw_residual, 2.0);
  pnh.param("optimizer/max_vel", optimizer_params_.max_vel, 1.0);
  pnh.param("optimizer/max_acc", optimizer_params_.max_acc, 2.0);
  pnh.param("optimizer/max_yaw_rate", optimizer_params_.max_yaw_rate, 1.5);
  pnh.param("optimizer/safety_margin", optimizer_params_.safety_margin, 0.05);
  pnh.param("optimizer/step_size", optimizer_params_.step_size, 0.005);

  pnh.param("hybrid_astar/step_size", hybrid_astar_params_.step_size, 0.35);
  pnh.param("hybrid_astar/wheel_base", hybrid_astar_params_.wheel_base, 0.8);
  pnh.param("hybrid_astar/max_steer", hybrid_astar_params_.max_steer, 0.6);
  pnh.param("hybrid_astar/steer_samples", hybrid_astar_params_.steer_samples, 5);
  pnh.param("hybrid_astar/goal_tolerance_pos",
            hybrid_astar_params_.goal_tolerance_pos, 0.35);
  pnh.param("hybrid_astar/goal_tolerance_yaw",
            hybrid_astar_params_.goal_tolerance_yaw, 0.4);
  pnh.param("hybrid_astar/reverse_penalty",
            hybrid_astar_params_.reverse_penalty, 2.0);
  pnh.param("hybrid_astar/steer_penalty",
            hybrid_astar_params_.steer_penalty, 0.2);
  pnh.param("hybrid_astar/steer_change_penalty",
            hybrid_astar_params_.steer_change_penalty, 0.2);
  pnh.param("hybrid_astar/switch_penalty",
            hybrid_astar_params_.switch_penalty, 2.0);
  pnh.param("hybrid_astar/max_expansions",
            hybrid_astar_params_.max_expansions, 50000);
  pnh.param("hybrid_astar/fast_pass_max_expansions",
            hybrid_astar_params_.fast_pass_max_expansions, 8000);
  pnh.param("hybrid_astar/fast_pass_steer_samples",
            hybrid_astar_params_.fast_pass_steer_samples, 3);
  pnh.param("hybrid_astar/pose_recovery_max_radius",
            hybrid_astar_params_.pose_recovery_max_radius, 0.6);
  pnh.param("hybrid_astar/pose_recovery_radial_step",
            hybrid_astar_params_.pose_recovery_radial_step, 0.05);
  pnh.param("hybrid_astar/pose_recovery_angular_samples",
            hybrid_astar_params_.pose_recovery_angular_samples, 24);
}

bool SvsdfRuntime::UpdateMap(const nav_msgs::OccupancyGrid& map) {
  if (!initialized_) {
    return false;
  }

  latest_map_ = map;
  nav_msgs::OccupancyGrid::Ptr map_ptr(new nav_msgs::OccupancyGrid(map));
  grid_map_.fromOccupancyGrid(map_ptr);

  collision_checker_.init(grid_map_, footprint_, yaw_bins_);
  se2_generator_.init(grid_map_, collision_checker_, se2_disc_step_,
                      max_push_attempts_);
  hybrid_astar_.init(grid_map_, collision_checker_, hybrid_astar_params_);
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_,
                         footprint_.inscribedRadius());
  svsdf_evaluator_.initGridEsdf(grid_map_, footprint_);
  optimizer_.init(grid_map_, svsdf_evaluator_, optimizer_params_);
  if (!strict_frontend_.UpdateMap(map) || !strict_solver_.UpdateMap(map)) {
    map_ready_ = false;
    return false;
  }
  strict_solver_.setGridMapForMidend(&grid_map_);

  map_ready_ = true;
  return true;
}

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

  const double topo_safe_dist = 0.25; // Ensure connectivity for topological roadmap
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_, topo_safe_dist);

  const Eigen::Vector2d start_2d(start_state.x, start_state.y);
  const Eigen::Vector2d goal_2d(goal_state.x, goal_state.y);
  topology_planner_.buildRoadmap(start_2d, goal_2d);
  std::vector<TopoPath> topo_paths = topology_planner_.searchPaths();
  
  if (!topo_paths.empty()) {
    ROS_INFO("Runtime stage 1: Found %zu topological paths, starting shortening (Algorithm 1)",
             topo_paths.size());
    topology_planner_.shortenPaths(topo_paths);
  }

  std::vector<SE2State> hybrid_path;
  if (topo_paths.empty()) {
    ROS_WARN("Topology search failed, trying Hybrid A* fallback");
    if (hybrid_astar_.plan(start_state, goal_state, hybrid_path)) {
      result.coarse_path = MakePath(hybrid_path);
      result.stats.coarse_path_points = static_cast<int>(hybrid_path.size());
    }
  }

  if (result.coarse_path.poses.empty() && !topo_paths.empty()) {
    result.coarse_path = MakePathFromTopo(topo_paths.front());
    result.stats.coarse_path_points =
        static_cast<int>(topo_paths.front().waypoints.size());
  }
  result.stats.search_time = (ros::WallTime::now() - search_start).toSec();

  const ros::WallTime optimize_start = ros::WallTime::now();
  std::vector<CandidateResult> candidate_results;
  std::vector<CandidateResult> degraded_results;
  candidate_results.reserve(topo_paths.size());
  for (size_t i = 0; i < topo_paths.size(); ++i) {
    const double elapsed = (ros::WallTime::now() - optimize_start).toSec();
    if (elapsed > 60.0) {
      ROS_WARN("Plan() optimization timeout after %.1fs, evaluated %zu/%zu paths",
               elapsed, i, topo_paths.size());
      break;
    }
    const std::vector<MotionSegment> raw_segments =
        se2_generator_.generate(topo_paths[i], start_state, goal_state);
    if (raw_segments.empty()) {
      ROS_WARN("Path %zu discarded: no motion segments", i);
      continue;
    }

    // Push all waypoints (including segment boundaries) away from obstacles
    std::vector<MotionSegment> segments = raw_segments;
    PushSegmentWaypointsFromObstacles(&segments);

    const CandidateResult candidate = EvaluateCandidate(segments);
    if (candidate.trajectory.empty()) {
      ROS_WARN("Path %zu discarded: infeasible segment or stitched trajectory", i);
      continue;
    }

    if (candidate.degraded) {
      ROS_WARN("Path %zu degraded: clearance=%.3f (kept as fallback)", i, candidate.min_clearance);
      degraded_results.push_back(candidate);
      continue;
    }

    ROS_INFO("Path %zu accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f, escalated=%d", i,
             candidate.min_clearance, candidate.max_vel, candidate.max_acc,
             candidate.escalated_segments);
    candidate_results.push_back(candidate);
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
      const CandidateResult candidate = EvaluateCandidate(segments);
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
        result.trajectory = MakePath(hybrid_path);
        result.stats.min_clearance = fallback_clearance;
        result.stats.support_points = static_cast<int>(hybrid_path.size());
        result.stats.local_obstacle_points = 0;
        result.stats.optimizer_iterations = 0;
        result.stats.total_solve_time =
            (ros::WallTime::now() - total_start).toSec();
        return result;
      }
    }
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    return result;
  }

  std::vector<Trajectory> candidates;
  candidates.reserve(candidate_results.size());
  for (size_t i = 0; i < candidate_results.size(); ++i) {
    candidates.push_back(candidate_results[i].trajectory);
  }

  // If only one candidate passed EvaluateCandidate, use it directly
  // (selectBest may re-evaluate with stricter criteria and reject it)
  Trajectory best;
  if (candidates.size() == 1) {
    best = candidates[0];
  } else {
    best = optimizer_.selectBest(candidates);
    if (best.empty() && !candidates.empty()) {
      // selectBest rejected all candidates; fall back to the one with best clearance
      size_t best_clearance_idx = 0;
      double best_clearance = -kInf;
      for (size_t i = 0; i < candidate_results.size(); ++i) {
        if (candidate_results[i].min_clearance > best_clearance) {
          best_clearance = candidate_results[i].min_clearance;
          best_clearance_idx = i;
        }
      }
      best = candidates[best_clearance_idx];
    }
  }
  if (best.empty()) {
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    return result;
  }

  size_t best_index = 0;
  bool found_index = false;
  for (size_t i = 0; i < candidate_results.size(); ++i) {
    if (candidate_results[i].trajectory.pos_pieces.size() == best.pos_pieces.size() &&
        std::abs(candidate_results[i].trajectory.totalDuration() -
                 best.totalDuration()) < 1e-6) {
      best_index = i;
      found_index = true;
      break;
    }
  }
  if (!found_index) {
    best_index = 0;
  }

  result.success = true;
  
  // Directly write to file for reliable analysis
  FILE* traj_file = fopen("/home/chengzhuo/workspace/plan/traj_data.csv", "w");
  if (traj_file) {
    fprintf(traj_file, "t,x,y,yaw\n");
    const double total_dur = best.totalDuration();
    for (double t = 0.0; t <= total_dur; t += 0.05) {
      SE2State state = best.sample(t);
      fprintf(traj_file, "%.3f,%.4f,%.4f,%.4f\n", t, state.x, state.y, state.yaw);
    }
    fclose(traj_file);
  }

  ROS_INFO("Plan(): best traj pieces=%zu duration=%.3f, sampling first piece c0=(%.4f,%.4f) c1=(%.4f,%.4f)",
           best.pos_pieces.size(), best.totalDuration(),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,1),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,1));
  result.trajectory = MakeTrajectoryPath(best);
  ROS_INFO("Plan(): published trajectory poses=%zu", result.trajectory.poses.size());
  result.stats.min_clearance = candidate_results[best_index].min_clearance;
  result.stats.support_points = candidate_results[best_index].support_points;
  result.stats.local_obstacle_points = candidate_results[best_index].high_risk_segments;
  result.stats.optimizer_iterations = 0;
  result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
  return result;
}

SvsdfRuntime::CandidateResult SvsdfRuntime::EvaluateCandidate(
    const std::vector<MotionSegment>& segments) {
  CandidateResult result;
  result.support_points = CountSupportPoints(segments);

  std::vector<Trajectory> segment_trajectories(segments.size());
  std::vector<bool> escalated(segments.size(), false);
  int high_risk_segments = 0;
  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].waypoints.size() < 2) {
      return CandidateResult();
    }

    const double seg_time = EstimateSegmentTime(segments[i].waypoints);
    Trajectory segment_traj;
    if (segments[i].risk == RiskLevel::HIGH) {
      ++high_risk_segments;
      ROS_INFO("Runtime stage: solving high-risk segment %zu with strict SE(2), points=%zu",
               i, segments[i].waypoints.size());
      if (!SolveStrictSegment(segments[i].waypoints, &segment_traj)) {
        ROS_WARN("Strict SE(2) solve failed on high-risk segment %zu", i);
        return CandidateResult();
      }
    } else {
      ROS_INFO("Runtime stage: solving low-risk segment %zu in R^2, points=%zu",
               i, segments[i].waypoints.size());
      if (!OptimizeLowRiskSegment(segments[i].waypoints, seg_time, &segment_traj)) {
        ROS_WARN("Low-risk segment %zu failed in both R^2 and strict SE(2)", i);
        return CandidateResult();
      }
      double segment_clearance = -kInf;
      if (!IsTrajectoryFeasible(segment_traj, &segment_clearance, nullptr, nullptr)) {
        if (!SolveStrictSegment(segments[i].waypoints, &segment_traj)) {
          ROS_WARN("Low-risk segment %zu escalation to strict SE(2) failed", i);
          return CandidateResult();
        }
        escalated[i] = true;
        ++result.escalated_segments;
      }
    }
    if (segment_traj.empty()) {
      ROS_WARN("Segment %zu optimization failed (risk=%s, points=%zu)", i,
               segments[i].risk == RiskLevel::HIGH ? "HIGH" : "LOW",
               segments[i].waypoints.size());
      return CandidateResult();
    }
    ROS_INFO("Segment %zu optimized, risk=%s, duration=%f", i,
             segments[i].risk == RiskLevel::HIGH ? "HIGH" : "LOW",
             segment_traj.totalDuration());
    segment_trajectories[i] = segment_traj;
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
  if (!IsTrajectoryFeasible(full_traj, &min_clearance, &max_vel, &max_acc)) {
    bool upgraded_low_risk = false;
    for (size_t i = 0; i < segments.size(); ++i) {
      if (segments[i].risk == RiskLevel::LOW && !escalated[i]) {
        if (!SolveStrictSegment(segments[i].waypoints, &segment_trajectories[i])) {
          ROS_WARN("Low-risk segment %zu final strict SE(2) upgrade failed", i);
          return CandidateResult();
        }
        escalated[i] = true;
        ++result.escalated_segments;
        upgraded_low_risk = true;
      }
    }

    const Trajectory full_traj = optimizer_.stitch(segments, segment_trajectories);
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
  if (!full_traj.empty() &&
        IsTrajectoryFeasible(full_traj, &min_clearance, &max_vel, &max_acc)) {
      result.trajectory = full_traj;
      result.min_clearance = min_clearance;
      result.max_vel = max_vel;
      result.max_acc = max_acc;
      result.high_risk_segments = high_risk_segments;
      MaybeImproveTailTrajectory(segments, segment_trajectories, &result);
      return result;
    }

    if (TryRecoverBottleneck(segments, segment_trajectories, full_traj, &result)) {
      return result;
    }

    // --- ENHANCED RECOVERY: Global SE(2) Escalation ---
    ROS_WARN("Candidate infeasible (clearance=%.3f). Escalating ALL segments to strict SE(2)...", min_clearance);
    for (size_t i = 0; i < segments.size(); ++i) {
      if (!escalated[i] && segments[i].risk == RiskLevel::LOW) {
        if (!SolveStrictSegment(segments[i].waypoints, &segment_trajectories[i])) {
          ROS_WARN("Global escalation failed on segment %zu", i);
          return CandidateResult();
        }
      }
    }

    const Trajectory global_se2_traj = optimizer_.stitch(segments, segment_trajectories);
    if (!global_se2_traj.empty() &&
        IsTrajectoryFeasible(global_se2_traj, &min_clearance, &max_vel, &max_acc)) {
      ROS_INFO("Global SE(2) escalation SUCCEEDED: clearance=%.3f", min_clearance);
      result.trajectory = global_se2_traj;
      result.min_clearance = min_clearance;
      result.max_vel = max_vel;
      result.max_acc = max_acc;
      result.high_risk_segments = static_cast<int>(segments.size());
      MaybeImproveTailTrajectory(segments, segment_trajectories, &result);
      return result;
    }

    ROS_WARN("Path degraded: infeasible even after global SE(2) escalation (clearance=%.3f)", min_clearance);
    // Return the trajectory anyway with its clearance info so Plan() can use it as a fallback
    if (!global_se2_traj.empty()) {
      result.trajectory = global_se2_traj;
      result.min_clearance = min_clearance;
      result.max_vel = max_vel;
      result.max_acc = max_acc;
      result.high_risk_segments = static_cast<int>(segments.size());
      result.degraded = true;
      return result;
    }
    return CandidateResult();
  }

  result.trajectory = full_traj;
  result.min_clearance = min_clearance;
  result.max_vel = max_vel;
  result.max_acc = max_acc;
  result.high_risk_segments = high_risk_segments;
  MaybeImproveTailTrajectory(segments, segment_trajectories, &result);
  return result;
}

void SvsdfRuntime::MaybeImproveTailTrajectory(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories,
    CandidateResult* result) {
  if (result == nullptr || result->trajectory.empty()) {
    return;
  }

  const size_t segment_count = std::min(segments.size(), segment_trajectories.size());
  if (segment_count < 2) {
    return;
  }

  CandidateResult best = *result;
  double best_length = ApproximateTrajectoryLength(best.trajectory);
  const size_t max_suffix_segments = std::min<size_t>(6, segment_count);
  const size_t first_suffix_index =
      segment_count > max_suffix_segments ? segment_count - max_suffix_segments : 0;

  auto maybe_accept_suffix =
      [&](size_t start_index, const std::vector<SE2State>& suffix_waypoints,
          bool use_strict, const char* label) {
        if (suffix_waypoints.size() < 2) {
          return;
        }

        Trajectory suffix_traj;
        const double seg_time = EstimateSegmentTime(suffix_waypoints);
        const bool solved =
            use_strict ? SolveStrictSegment(suffix_waypoints, &suffix_traj)
                       : OptimizeLowRiskSegment(suffix_waypoints, seg_time, &suffix_traj);
        if (!solved || suffix_traj.empty()) {
          return;
        }

        const Trajectory candidate_traj =
            ConcatenateTrajectories(segment_trajectories, start_index, suffix_traj);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!IsTrajectoryFeasible(candidate_traj, &min_clearance, &max_vel, &max_acc)) {
          return;
        }

        const double candidate_length = ApproximateTrajectoryLength(candidate_traj);
        const size_t candidate_pieces = candidate_traj.pos_pieces.size();
        const bool better_clearance = min_clearance > best.min_clearance + 1e-3;
        const bool clearance_not_worse = min_clearance + 1e-3 >= best.min_clearance;
        const bool better_geometry =
            candidate_pieces + 1 < best.trajectory.pos_pieces.size() ||
            candidate_length + 0.25 < best_length;

        if (!better_clearance && !(clearance_not_worse && better_geometry)) {
          return;
        }

        int prefix_support_points = 0;
        for (size_t i = 0; i < start_index; ++i) {
          prefix_support_points += static_cast<int>(segments[i].waypoints.size());
        }

        ROS_INFO(
            "Tail refinement accepted from seg %zu via %s: pieces %zu -> %zu, length %.3f -> %.3f, clearance %.3f -> %.3f",
            start_index, label, best.trajectory.pos_pieces.size(), candidate_pieces,
            best_length, candidate_length, best.min_clearance, min_clearance);

        best.trajectory = candidate_traj;
        best.min_clearance = min_clearance;
        best.max_vel = max_vel;
        best.max_acc = max_acc;
        best.support_points =
            prefix_support_points + static_cast<int>(suffix_waypoints.size());
        best.high_risk_segments =
            CountHighRiskSegments(segments, start_index) + (use_strict ? 1 : 0);
        best_length = candidate_length;
      };

  for (size_t start_index = first_suffix_index; start_index + 1 < segment_count;
       ++start_index) {
    std::vector<SE2State> merged_suffix;
    merged_suffix.reserve(segments[start_index].waypoints.size());
    bool suffix_has_high = false;
    for (size_t i = start_index; i < segment_count; ++i) {
      suffix_has_high = suffix_has_high || segments[i].risk == RiskLevel::HIGH;
      AppendUniqueWaypoints(segments[i].waypoints, &merged_suffix);
    }

    if (merged_suffix.size() < 2) {
      continue;
    }

    const size_t suffix_segment_count = segment_count - start_index;
    const double suffix_length = WaypointPathLength(merged_suffix);
    const double suffix_chord =
        (merged_suffix.back().position() - merged_suffix.front().position()).norm();

    if (suffix_segment_count >= 2) {
      maybe_accept_suffix(start_index, merged_suffix, false, "merged_suffix_r2");
      if (suffix_has_high) {
        maybe_accept_suffix(start_index, merged_suffix, true, "merged_suffix_strict");
      }
    }

    if (suffix_chord > 2.0 && suffix_length > 1.10 * suffix_chord) {
      const std::vector<SE2State> direct_suffix =
          BuildLinearWaypoints(merged_suffix.front(), merged_suffix.back(), se2_disc_step_);
      maybe_accept_suffix(start_index, direct_suffix, false, "direct_suffix_r2");
      if (suffix_has_high) {
        maybe_accept_suffix(start_index, direct_suffix, true, "direct_suffix_strict");
      }
    }
  }

  *result = best;
}

bool SvsdfRuntime::TryRecoverBottleneck(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories, const Trajectory& current_traj,
    CandidateResult* result) {
  if (result == nullptr || current_traj.empty()) {
    return false;
  }

  const ros::WallTime recover_start = ros::WallTime::now();
  const double kRecoverTimeoutSec = 10.0;

  const size_t segment_count = std::min(segments.size(), segment_trajectories.size());
  if (segment_count == 0) {
    return false;
  }

  const ClearanceProbe worst = FindWorstClearanceSample(current_traj, svsdf_evaluator_);
  if (!worst.valid) {
    return false;
  }

  const size_t culprit_index = SegmentIndexFromTime(segment_trajectories, worst.time);
  const double anchor_clearance_threshold = std::max(0.05, optimizer_params_.safety_margin);
  const double endpoint_pos_tolerance =
      std::max(0.15, hybrid_astar_params_.goal_tolerance_pos + 0.05);
  const double endpoint_yaw_tolerance =
      std::max(0.20, hybrid_astar_params_.goal_tolerance_yaw + 0.10);

  std::vector<std::pair<size_t, size_t>> candidate_windows;
  auto add_window = [&](size_t begin, size_t end) {
    if (begin > end || end >= segment_count) {
      return;
    }
    for (const auto& window : candidate_windows) {
      if (window.first == begin && window.second == end) {
        return;
      }
    }
    candidate_windows.emplace_back(begin, end);
  };

  size_t risky_begin = culprit_index;
  size_t risky_end = culprit_index;
  if (segments[culprit_index].risk == RiskLevel::LOW) {
    if (culprit_index > 0 && segments[culprit_index - 1].risk == RiskLevel::HIGH) {
      risky_begin = culprit_index - 1;
    }
    if (culprit_index + 1 < segment_count &&
        segments[culprit_index + 1].risk == RiskLevel::HIGH) {
      risky_end = culprit_index + 1;
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

  add_window(expanded_begin, expanded_end);
  add_window(local_begin, local_end);
  add_window(culprit_index, culprit_index);

  ROS_INFO(
      "Local bottleneck rebuild: culprit_seg=%zu state=(%.3f, %.3f, %.3f) clearance=%.3f windows=%zu",
      culprit_index, worst.state.x, worst.state.y, worst.state.yaw, worst.clearance,
      candidate_windows.size());

  CandidateResult best;
  double best_length = kInf;
  bool found = false;
  std::set<std::pair<size_t, size_t>> processed_windows;

  auto maybe_accept_patch =
      [&](size_t begin, size_t end, const std::vector<SE2State>& patch_waypoints,
          const char* label) {
        if (patch_waypoints.size() < 2) {
          return;
        }

        Trajectory patch_traj;
        if (!SolveStrictSegment(patch_waypoints, &patch_traj) || patch_traj.empty()) {
          patch_traj = optimizer_.optimizeSE2(
              patch_waypoints, 1.5 * EstimateSegmentTime(patch_waypoints));
        }
        if (patch_traj.empty()) {
          return;
        }

        const Trajectory rebuilt =
            ConcatenateTrajectories(segment_trajectories, begin, end + 1, patch_traj);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
          return;
        }

        const double rebuilt_length = ApproximateTrajectoryLength(rebuilt);
        const bool better_candidate =
            !found || min_clearance > best.min_clearance + 1e-3 ||
            (min_clearance + 1e-3 >= best.min_clearance &&
             (rebuilt.pos_pieces.size() < best.trajectory.pos_pieces.size() ||
              rebuilt_length + 0.25 < best_length));
        if (!better_candidate) {
          return;
        }

        CandidateResult candidate = *result;
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
            label, culprit_index, begin, end, worst.clearance, min_clearance,
            current_traj.pos_pieces.size(), rebuilt.pos_pieces.size());

        best = candidate;
        best_length = rebuilt_length;
        found = true;
      };

  auto maybe_accept_segment_patch =
      [&](size_t begin, size_t end, const std::vector<MotionSegment>& patch_segments,
          const char* label) {
        if (patch_segments.empty()) {
          return;
        }

        std::vector<Trajectory> patch_segment_trajectories(patch_segments.size());
        std::vector<bool> escalated(patch_segments.size(), false);
        int escalated_segments = 0;
        for (size_t patch_index = 0; patch_index < patch_segments.size(); ++patch_index) {
          if (patch_segments[patch_index].waypoints.size() < 2) {
            return;
          }

          const double seg_time =
              EstimateSegmentTime(patch_segments[patch_index].waypoints);
          Trajectory patch_segment_traj;
          if (patch_segments[patch_index].risk == RiskLevel::HIGH) {
            if (!SolveStrictSegment(patch_segments[patch_index].waypoints,
                                    &patch_segment_traj)) {
              return;
            }
          } else {
            if (!OptimizeLowRiskSegment(patch_segments[patch_index].waypoints, seg_time,
                                        &patch_segment_traj)) {
              return;
            }
            double segment_clearance = -kInf;
            if (!IsTrajectoryFeasible(patch_segment_traj, &segment_clearance, nullptr,
                                      nullptr)) {
              if (!SolveStrictSegment(patch_segments[patch_index].waypoints,
                                      &patch_segment_traj)) {
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

        Trajectory middle = optimizer_.stitch(patch_segments, patch_segment_trajectories);
        double middle_clearance = -kInf;
        if (middle.empty() ||
            !IsTrajectoryFeasible(middle, &middle_clearance, nullptr, nullptr)) {
          bool upgraded = false;
          for (size_t patch_index = 0; patch_index < patch_segments.size(); ++patch_index) {
            if (patch_segments[patch_index].risk == RiskLevel::LOW &&
                !escalated[patch_index]) {
              if (!SolveStrictSegment(patch_segments[patch_index].waypoints,
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
          middle = optimizer_.stitch(patch_segments, patch_segment_trajectories);
          if (middle.empty() ||
              !IsTrajectoryFeasible(middle, &middle_clearance, nullptr, nullptr)) {
            return;
          }
        }

        const Trajectory rebuilt =
            ConcatenateTrajectories(segment_trajectories, begin, end + 1, middle);
        double min_clearance = -kInf;
        double max_vel = 0.0;
        double max_acc = 0.0;
        if (!IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
          return;
        }

        const double rebuilt_length = ApproximateTrajectoryLength(rebuilt);
        const bool better_candidate =
            !found || min_clearance > best.min_clearance + 1e-3 ||
            (min_clearance + 1e-3 >= best.min_clearance &&
             (rebuilt.pos_pieces.size() < best.trajectory.pos_pieces.size() ||
              rebuilt_length + 0.25 < best_length));
        if (!better_candidate) {
          return;
        }

        CandidateResult candidate = *result;
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
            label, culprit_index, begin, end, worst.clearance, min_clearance,
            current_traj.pos_pieces.size(), rebuilt.pos_pieces.size());

        best = candidate;
        best_length = rebuilt_length;
        found = true;
      };

  for (const auto& window : candidate_windows) {
      if ((ros::WallTime::now() - recover_start).toSec() > kRecoverTimeoutSec) {
        ROS_WARN("TryRecoverBottleneck timeout after %.1fs", kRecoverTimeoutSec);
        break;
      }
      size_t begin = window.first;
      size_t end = window.second;

      while (begin > 0 &&
             (!std::isfinite(svsdf_evaluator_.evaluate(segments[begin].waypoints.front())) ||
              svsdf_evaluator_.evaluate(segments[begin].waypoints.front()) <
                  anchor_clearance_threshold)) {
        --begin;
      }
      while (end + 1 < segment_count &&
             (!std::isfinite(svsdf_evaluator_.evaluate(segments[end].waypoints.back())) ||
              svsdf_evaluator_.evaluate(segments[end].waypoints.back()) <
                  anchor_clearance_threshold)) {
        ++end;
      }

      if (!processed_windows.insert(std::make_pair(begin, end)).second) {
        continue;
      }

      const SE2State start_anchor = segments[begin].waypoints.front();
      const SE2State goal_anchor = segments[end].waypoints.back();
      const double start_clearance = svsdf_evaluator_.evaluate(start_anchor);
      const double goal_clearance = svsdf_evaluator_.evaluate(goal_anchor);
      if (!std::isfinite(start_clearance) || !std::isfinite(goal_clearance) ||
          start_clearance < anchor_clearance_threshold ||
          goal_clearance < anchor_clearance_threshold) {
        continue;
      }

      TopologyPlanner local_topology_planner;
      const int local_topology_samples =
          std::max(1200, std::min(4000, topology_num_samples_ / 4));
      const int local_topology_knn =
          std::max(12, std::min(24, topology_knn_));
      const int local_topology_paths =
          std::max(6, std::min(12, topology_max_paths_));
      const double local_inscribed_radius =
          footprint_.inscribedRadius() + grid_map_.resolution();
      local_topology_planner.init(grid_map_, collision_checker_, local_topology_samples,
                                  local_topology_knn, local_topology_paths,
                                  local_inscribed_radius);
      local_topology_planner.buildRoadmap(start_anchor.position(), goal_anchor.position());
      const std::pair<double, double> corridor_window =
          SegmentTimeWindow(segment_trajectories, begin, end);
      const double corridor_block_threshold =
          std::max(0.10, anchor_clearance_threshold + grid_map_.resolution());
      const double corridor_sample_spacing =
          std::max(0.20, 0.8 * footprint_.inscribedRadius());
      std::vector<Eigen::Vector2d> blocked_centers = SampleLowClearanceCorridor(
          current_traj, svsdf_evaluator_, corridor_window.first - 0.20,
          corridor_window.second + 0.20, corridor_block_threshold,
          corridor_sample_spacing);
      if (blocked_centers.empty()) {
        blocked_centers.push_back(worst.state.position());
      }

      ROS_INFO(
          "Local bottleneck rebuild window=[%zu,%zu] anchors=(%.3f, %.3f)->(%.3f, %.3f) blocked_centers=%zu",
          begin, end, start_anchor.x, start_anchor.y, goal_anchor.x, goal_anchor.y,
          blocked_centers.size());

      const double hotspot_radius_base =
          std::max(1.00, 3.0 * footprint_.circumscribedRadius());
      const double hotspot_radius_scales[] = {1.0, 1.5, 2.0};
      for (double radius_scale : hotspot_radius_scales) {
        const double hotspot_radius = hotspot_radius_base * radius_scale;
        std::vector<TopoPath> local_paths =
            local_topology_planner.searchPathsAvoidingCenters(blocked_centers,
                                                              hotspot_radius);
        const size_t max_local_paths = std::min<size_t>(3, local_paths.size());
        for (size_t path_idx = 0; path_idx < max_local_paths; ++path_idx) {
          const std::vector<MotionSegment> local_segments =
              se2_generator_.generate(local_paths[path_idx], start_anchor, goal_anchor);
          if (local_segments.empty()) {
            continue;
          }
          const std::string label =
              "local_topology_r" + std::to_string(hotspot_radius);
          maybe_accept_segment_patch(begin, end, local_segments, label.c_str());
        }

        const double clearance_base =
            std::max(anchor_clearance_threshold,
                     footprint_.inscribedRadius() + grid_map_.resolution());
        const double clearance_candidates[] = {
            std::max(clearance_base, 0.5 * footprint_.circumscribedRadius()),
            clearance_base,
            std::max(0.08, optimizer_params_.safety_margin)};
        const int min_safe_yaw_candidates[] = {4, 2, 1};
        for (double corridor_clearance : clearance_candidates) {
          SweptAstar corridor_astar;
          corridor_astar.init(grid_map_, collision_checker_, corridor_clearance);

          Eigen::Vector2d min_corner = start_anchor.position().cwiseMin(goal_anchor.position());
          Eigen::Vector2d max_corner = start_anchor.position().cwiseMax(goal_anchor.position());
          min_corner = min_corner.cwiseMin(worst.state.position());
          max_corner = max_corner.cwiseMax(worst.state.position());
          for (const Eigen::Vector2d& center : blocked_centers) {
            min_corner = min_corner.cwiseMin(center);
            max_corner = max_corner.cwiseMax(center);
          }
          const double search_padding = std::max(2.0, 1.5 * hotspot_radius);
          min_corner.array() -= search_padding;
          max_corner.array() += search_padding;
          GridIndex min_index = grid_map_.worldToGrid(min_corner.x(), min_corner.y());
          GridIndex max_index = grid_map_.worldToGrid(max_corner.x(), max_corner.y());

          SweptAstarSearchOptions options;
          options.blocked_centers = blocked_centers;
          options.blocked_radius = hotspot_radius;
          options.min_x = std::max(0, min_index.x);
          options.min_y = std::max(0, min_index.y);
          options.max_x = std::min(grid_map_.width() - 1, max_index.x);
          options.max_y = std::min(grid_map_.height() - 1, max_index.y);
          options.preferred_clearance = corridor_clearance + 0.10;
          options.clearance_penalty = 1.5;
          options.blocked_center_penalty = 1.0;

          for (int min_safe_yaw_count : min_safe_yaw_candidates) {
            options.min_safe_yaw_count = min_safe_yaw_count;
            const AstarSearchResult corridor_result =
                corridor_astar.search(start_anchor.position(), goal_anchor.position(),
                                      options);
            if (!corridor_result.success || corridor_result.path.size() < 2) {
              continue;
            }

            const TopoPath corridor_topo = BuildTopoPathFromCorridor(
                corridor_result.path, grid_map_, collision_checker_, corridor_clearance,
                blocked_centers, hotspot_radius, min_safe_yaw_count);
            if (corridor_topo.waypoints.size() < 2) {
              continue;
            }

            const std::vector<MotionSegment> corridor_segments =
                se2_generator_.generate(corridor_topo, start_anchor, goal_anchor);
            if (corridor_segments.empty()) {
              continue;
            }
            const std::string label =
                "local_corridor_astar_r" + std::to_string(hotspot_radius) +
                "_c" + std::to_string(corridor_clearance) +
                "_y" + std::to_string(min_safe_yaw_count);
            maybe_accept_segment_patch(begin, end, corridor_segments, label.c_str());
          }
        }
      }

      std::vector<SE2State> patch_waypoints;
      if (!hybrid_astar_.plan(start_anchor, goal_anchor, patch_waypoints) ||
          patch_waypoints.size() < 2) {
        continue;
      }

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
        continue;
      }

      patch_waypoints.front() = start_anchor;
      patch_waypoints.back() = goal_anchor;
      maybe_accept_patch(begin, end, patch_waypoints, "direct_hastar");

      const Eigen::Vector2d lateral(-std::sin(worst.state.yaw), std::cos(worst.state.yaw));
      const double detour_offsets[] = {0.60, 1.00, 1.40, 1.80};
      for (int side : {-1, 1}) {
        for (double offset : detour_offsets) {
          const Eigen::Vector2d via_pos =
              worst.state.position() + static_cast<double>(side) * offset * lateral;
          if (grid_map_.getEsdf(via_pos.x(), via_pos.y()) < 0.10) {
            continue;
          }

          const SE2State via_state(via_pos.x(), via_pos.y(), worst.state.yaw);
          std::vector<SE2State> first_leg;
          if (!hybrid_astar_.plan(start_anchor, via_state, first_leg) ||
              first_leg.size() < 2) {
            continue;
          }

          const SE2State recovered_via = first_leg.back();
          std::vector<SE2State> second_leg;
          if (!hybrid_astar_.plan(recovered_via, goal_anchor, second_leg) ||
              second_leg.size() < 2) {
            continue;
          }

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
      if (found) break;
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

bool SvsdfRuntime::SolveStrictSegment(const std::vector<SE2State>& waypoints,
                                      Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }
  std::vector<SE2State> sparse_waypoints = SparsifyStrictWaypoints(waypoints, grid_map_);
  ROS_INFO("Runtime stage: strict SE(2) input sparsified from %zu to %zu points",
           waypoints.size(), sparse_waypoints.size());

  // Push interior waypoints away from obstacles along ESDF gradient
  const double push_margin = 0.40;
  const double push_step = 0.10;
  const int max_push_iters = 15;
  const double h = grid_map_.resolution();
  for (size_t i = 1; i + 1 < sparse_waypoints.size(); ++i) {
    for (int iter = 0; iter < max_push_iters; ++iter) {
      const double esdf = grid_map_.getEsdf(sparse_waypoints[i].x, sparse_waypoints[i].y);
      if (std::isfinite(esdf) && esdf >= push_margin) break;

      // Compute ESDF gradient via finite difference
      const double dEdx = (grid_map_.getEsdf(sparse_waypoints[i].x + h, sparse_waypoints[i].y)
                         - grid_map_.getEsdf(sparse_waypoints[i].x - h, sparse_waypoints[i].y)) / (2.0 * h);
      const double dEdy = (grid_map_.getEsdf(sparse_waypoints[i].x, sparse_waypoints[i].y + h)
                         - grid_map_.getEsdf(sparse_waypoints[i].x, sparse_waypoints[i].y - h)) / (2.0 * h);
      const double grad_norm = std::sqrt(dEdx * dEdx + dEdy * dEdy);

      if (grad_norm > 1e-6) {
        // Push along gradient (toward higher ESDF)
        sparse_waypoints[i].x += push_step * dEdx / grad_norm;
        sparse_waypoints[i].y += push_step * dEdy / grad_norm;
      } else {
        // Gradient is zero (deep inside obstacle) — radial search for nearest free cell
        bool found_free = false;
        for (double radius = h; radius <= 2.0; radius += h) {
          double best_esdf = -1.0;
          double best_dx = 0, best_dy = 0;
          for (int a = 0; a < 16; ++a) {
            const double angle = a * M_PI / 8.0;
            const double dx = radius * std::cos(angle);
            const double dy = radius * std::sin(angle);
            const double e = grid_map_.getEsdf(sparse_waypoints[i].x + dx,
                                                sparse_waypoints[i].y + dy);
            if (std::isfinite(e) && e > best_esdf) {
              best_esdf = e;
              best_dx = dx;
              best_dy = dy;
            }
          }
          if (best_esdf > push_margin) {
            sparse_waypoints[i].x += best_dx;
            sparse_waypoints[i].y += best_dy;
            found_free = true;
            break;
          }
        }
        if (!found_free) break;
      }
    }
  }

  return strict_solver_.ready() &&
         strict_solver_.Solve(sparse_waypoints, trajectory);
}

bool SvsdfRuntime::OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                                          double seg_time,
                                          Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }

  // Use a balanced sparsification for low-risk segments to maintain
  // continuity at stitching points while keeping open space smooth.
  std::vector<SE2State> sparse_waypoints;
  if (waypoints.size() > 4) {
    sparse_waypoints.push_back(waypoints.front());
    sparse_waypoints.push_back(waypoints[waypoints.size() / 2]);
    sparse_waypoints.push_back(waypoints.back());
  } else {
    sparse_waypoints = waypoints;
  }

  *trajectory = optimizer_.optimizeR2(sparse_waypoints, seg_time);
  if (!trajectory->empty()) {
    return true;
  }
  return SolveStrictSegment(waypoints, trajectory);
}

bool SvsdfRuntime::IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                                        double* max_vel, double* max_acc) const {
  if (traj.empty()) {
    return false;
  }

  double clearance = svsdf_evaluator_.evaluateTrajectory(traj);
  double vmax = 0.0;
  double amax = 0.0;
  const double total = traj.totalDuration();
  for (double t = 0.0; t <= total; t += 0.02) {
    vmax = std::max(vmax, traj.sampleVelocity(t).norm());
    amax = std::max(amax, traj.sampleAcceleration(t).norm());
  }

  if (min_clearance) {
    *min_clearance = clearance;
  }
  if (max_vel) {
    *max_vel = vmax;
  }
  if (max_acc) {
    *max_acc = amax;
  }

  return std::isfinite(clearance) && clearance > -0.051;
}

double SvsdfRuntime::EstimateSegmentTime(
    const std::vector<SE2State>& waypoints) const {
  if (waypoints.size() < 2) {
    return 0.5;
  }
  double length = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double dx = waypoints[i].x - waypoints[i - 1].x;
    const double dy = waypoints[i].y - waypoints[i - 1].y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return std::max(0.5, length / std::max(0.10, optimizer_params_.max_vel));
}

nav_msgs::Path SvsdfRuntime::MakePath(const std::vector<SE2State>& states) const {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for (size_t i = 0; i < states.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = states[i].x;
    pose.pose.position.y = states[i].y;
    pose.pose.orientation.z = std::sin(states[i].yaw * 0.5);
    pose.pose.orientation.w = std::cos(states[i].yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::Path SvsdfRuntime::MakePathFromTopo(const TopoPath& path_in) const {
  std::vector<SE2State> states;
  states.reserve(path_in.waypoints.size());
  for (size_t i = 0; i < path_in.waypoints.size(); ++i) {
    const TopoWaypoint& wp = path_in.waypoints[i];
    states.push_back(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  }
  return MakePath(states);
}

nav_msgs::Path SvsdfRuntime::MakeTrajectoryPath(const Trajectory& traj) const {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  if (traj.empty()) {
    ROS_WARN("MakeTrajectoryPath: traj is empty!");
    return path;
  }

  const double total = traj.totalDuration();
  ROS_INFO("MakeTrajectoryPath: pieces=%zu total_duration=%.4f", traj.pos_pieces.size(), total);
  // Sample first and last position for sanity
  if (!traj.pos_pieces.empty()) {
    SE2State s0 = traj.sample(0.0);
    SE2State sf = traj.sample(std::max(0.0, total - 0.01));
    ROS_INFO("MakeTrajectoryPath: start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)",
             s0.x, s0.y, s0.yaw, sf.x, sf.y, sf.yaw);
  }
  for (double t = 0.0; t <= total; t += 0.05) {
    const SE2State state = traj.sample(t);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.z = std::sin(state.yaw * 0.5);
    pose.pose.orientation.w = std::cos(state.yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace esv_planner
