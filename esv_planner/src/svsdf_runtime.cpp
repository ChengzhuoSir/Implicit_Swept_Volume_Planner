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

bool DeadlineExpired(const ros::WallTime& deadline) {
  return !deadline.isZero() && ros::WallTime::now() >= deadline;
}

double RemainingBudgetSec(const ros::WallTime& deadline) {
  if (deadline.isZero()) {
    return kInf;
  }
  return std::max(0.0, (deadline - ros::WallTime::now()).toSec());
}

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

void AppendTrajectoryPieces(const Trajectory& source, Trajectory* target) {
  if (target == nullptr || source.empty()) {
    return;
  }
  target->pos_pieces.insert(target->pos_pieces.end(), source.pos_pieces.begin(),
                            source.pos_pieces.end());
  target->yaw_pieces.insert(target->yaw_pieces.end(), source.yaw_pieces.begin(),
                            source.yaw_pieces.end());
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

double TrajectorySmoothnessCost(const Trajectory& traj) {
  if (traj.empty()) {
    return kInf;
  }

  double cost = 0.0;
  for (const PolyPiece& piece : traj.pos_pieces) {
    const double duration = std::max(1e-3, piece.duration);
    const int n_sub = 8;
    const double h = duration / static_cast<double>(n_sub);
    for (int s = 0; s <= n_sub; ++s) {
      const double t = s * h;
      const Eigen::Vector2d acc = piece.acceleration(t);
      const double weight =
          (s == 0 || s == n_sub) ? 1.0 : ((s % 2 == 1) ? 4.0 : 2.0);
      cost += weight * acc.squaredNorm() * h / 3.0;
    }
  }
  return cost;
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
      static_cast<int>(checker.safeYawCount(point.x(), point.y())) <
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
  const double yaw = distance > 1e-9 ? std::atan2((goal - start).y(), (goal - start).x()) : 0.0;
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
    if (!checker.isFree(SE2State(point.x(), point.y(), yaw))) {
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
  const double topo_safe_dist = 0.25;
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_, topo_safe_dist);
  svsdf_evaluator_.initGridEsdf(grid_map_, footprint_);
  optimizer_.init(grid_map_, svsdf_evaluator_, optimizer_params_);
  if (!strict_solver_.UpdateMap(map)) {
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
        result.trajectory = MakePath(hybrid_path);
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
    MaybeImproveTailTrajectory(final_candidate.segments, final_candidate.segment_trajectories,
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
  result.trajectory = MakeTrajectoryPath(best);
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
    if (TryRecoverBottleneck(segments, segment_trajectories, upgraded_full_traj,
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

void SvsdfRuntime::MaybeImproveTailTrajectory(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories,
    const ros::WallTime& deadline, CandidateResult* result) {
  if (result == nullptr || result->trajectory.empty()) {
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

  CandidateResult best = *result;
  double best_length = ApproximateTrajectoryLength(best.trajectory);
  const double clearance_target = std::max(0.08, optimizer_params_.safety_margin);
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
        const double seg_time = EstimateSegmentTime(suffix_waypoints);
        const bool solved =
            use_strict ? SolveStrictSegment(suffix_waypoints, &suffix_traj)
                       : OptimizeLowRiskSegment(suffix_waypoints, seg_time, &suffix_traj);
        if (!solved || suffix_traj.empty()) {
          record_no_improvement();
          return;
        }

        const size_t candidate_piece_estimate =
            prefix_piece_counts[start_index] + suffix_traj.pos_pieces.size();
        const double candidate_length_estimate =
            prefix_lengths[start_index] + WaypointPathLength(suffix_waypoints);
        const double raw_suffix_clearance =
            svsdf_evaluator_.segmentClearance(suffix_waypoints, 0.10);
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
        if (!IsTrajectoryFeasible(candidate_traj, &min_clearance, &max_vel, &max_acc)) {
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

bool SvsdfRuntime::TryRecoverBottleneck(

    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& segment_trajectories, const Trajectory& current_traj,
    const ros::WallTime& deadline, CandidateResult* result) {
  if (result == nullptr || current_traj.empty()) {
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

  const ClearanceProbe worst = FindWorstClearanceSample(current_traj, svsdf_evaluator_);
  if (!worst.valid) {
    return false;
  }

  std::vector<ClearanceProbe> hotspots(1, worst);
  const double recover_timeout_sec = std::min(8.0, remaining_budget_sec);

  const size_t culprit_index = SegmentIndexFromTime(segment_trajectories, worst.time);
  const double anchor_clearance_threshold = std::max(0.05, optimizer_params_.safety_margin);
  const double endpoint_pos_tolerance =
      std::max(0.15, hybrid_astar_params_.goal_tolerance_pos + 0.05);
  const double endpoint_yaw_tolerance =
      std::max(0.20, hybrid_astar_params_.goal_tolerance_yaw + 0.10);

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

  CandidateResult best;
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
            svsdf_evaluator_.segmentClearance(patch_waypoints, 0.10);
        note_best_clearance(raw_patch_clearance, label, "raw");
        if (std::isfinite(raw_patch_clearance) && raw_patch_clearance > -0.051) {
          ++current_diag.single_patch_raw_feasible;
        } else {
          ++current_diag.single_patch_raw_infeasible;
        }

        Trajectory patch_traj;
        if (!SolveStrictSegment(patch_waypoints, &patch_traj) || patch_traj.empty()) {
          patch_traj = optimizer_.optimizeSE2(
              patch_waypoints, 1.5 * EstimateSegmentTime(patch_waypoints));
        }
        if (patch_traj.empty()) {
          ++current_diag.single_patch_solver_failures;
          return;
        }

        double patch_clearance = -kInf;
        if (!IsTrajectoryFeasible(patch_traj, &patch_clearance, nullptr, nullptr)) {
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
        if (!IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
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
              EstimateSegmentTime(patch_segments[patch_index].waypoints);
          Trajectory patch_segment_traj;
          if (patch_segments[patch_index].risk == RiskLevel::HIGH) {
            if (!SolveStrictSegment(patch_segments[patch_index].waypoints,
                                    &patch_segment_traj)) {
              ++current_diag.segment_patch_solver_failures;
              return;
            }
          } else {
            if (!OptimizeLowRiskSegment(patch_segments[patch_index].waypoints, seg_time,
                                        &patch_segment_traj)) {
              ++current_diag.segment_patch_solver_failures;
              return;
            }
            double segment_clearance = -kInf;
            if (!IsTrajectoryFeasible(patch_segment_traj, &segment_clearance, nullptr,
                                      nullptr)) {
              if (!SolveStrictSegment(patch_segments[patch_index].waypoints,
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

        Trajectory middle = optimizer_.stitch(patch_segments, patch_segment_trajectories);
        double middle_clearance = -kInf;
        if (middle.empty() ||
            !IsTrajectoryFeasible(middle, &middle_clearance, nullptr, nullptr)) {
          ++current_diag.segment_patch_middle_infeasible;
          note_best_clearance(middle_clearance, label, "local");
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
        if (!IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
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
        blocked_centers.push_back(active_hotspot.state.position());
      }

      ROS_INFO(
          "Local bottleneck rebuild window=[%zu,%zu] culprit_seg=%zu anchors=(%.3f, %.3f)->(%.3f, %.3f) blocked_centers=%zu",
          begin, end, active_culprit_index, start_anchor.x, start_anchor.y,
          goal_anchor.x, goal_anchor.y, blocked_centers.size());

      const bool precise_window = (begin == end && end == active_culprit_index);

      const double hotspot_radius_base =
          std::max(1.00, 3.0 * footprint_.circumscribedRadius());
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
                     footprint_.inscribedRadius() + grid_map_.resolution());
        std::vector<double> clearance_candidates;
        clearance_candidates.push_back(
            std::max(clearance_base, 0.5 * footprint_.circumscribedRadius()));
        if (precise_window) {
          clearance_candidates.push_back(clearance_base);
          clearance_candidates.push_back(std::max(0.08, optimizer_params_.safety_margin));
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
          corridor_astar.init(grid_map_, collision_checker_, corridor_clearance);

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
                corridor_result.path, grid_map_, collision_checker_, corridor_clearance,
                blocked_centers, hotspot_radius, min_safe_yaw_count);
            if (corridor_topo.waypoints.size() < 2) {
              continue;
            }
            ++current_diag.corridor_topo_successes;

            const std::vector<MotionSegment> corridor_segments =
                se2_generator_.generate(corridor_topo, start_anchor, goal_anchor);
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
      if (!hybrid_astar_.plan(start_anchor, goal_anchor, patch_waypoints) ||
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
          if (grid_map_.getEsdf(via_pos.x(), via_pos.y()) < 0.10) {
            continue;
          }

          const SE2State via_state(via_pos.x(), via_pos.y(), active_hotspot.state.yaw);
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
        IsTrajectoryFeasible(rebuilt, &min_clearance, &max_vel, &max_acc)) {
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

bool SvsdfRuntime::SolveStrictSegment(const std::vector<SE2State>& waypoints,
                                      Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }
  const double raw_clearance = svsdf_evaluator_.segmentClearance(waypoints, 0.10);
  const bool preserve_dense_input =
      std::isfinite(raw_clearance) && raw_clearance > -0.051;
  std::vector<SE2State> sparse_waypoints =
      preserve_dense_input ? waypoints : SparsifyStrictWaypoints(waypoints, grid_map_);
  ROS_INFO(
      "Runtime stage: strict SE(2) input %s from %zu to %zu points (raw_clearance=%.3f)",
      preserve_dense_input ? "preserved" : "sparsified", waypoints.size(),
      sparse_waypoints.size(), raw_clearance);

  // Push interior waypoints away from obstacles along ESDF gradient
  const double push_margin = 0.40;
  const double push_step = 0.10;
  const int max_push_iters = 15;
  const double h = grid_map_.resolution();
  if (!preserve_dense_input) {
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
  }

  if (!strict_solver_.ready() ||
      !strict_solver_.Solve(sparse_waypoints, trajectory, preserve_dense_input) ||
      trajectory->empty()) {
    return false;
  }

  if (preserve_dense_input) {
    double solved_clearance = -kInf;
    if (!IsTrajectoryFeasible(*trajectory, &solved_clearance, nullptr, nullptr)) {
      ROS_WARN(
          "Strict SE(2) solver degraded a feasible waypoint chain: raw_clearance=%.3f solved_clearance=%.3f",
          raw_clearance, solved_clearance);
    }
  }

  return true;
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

bool SvsdfRuntime::SolveStrictCached(size_t idx, const std::vector<SE2State>& wp,
                                     Trajectory* traj, SegmentCache& cache) {
  SegmentCacheKey key(idx, true);
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = SolveStrictSegment(wp, traj);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
}

bool SvsdfRuntime::OptimizeLowRiskCached(size_t idx, const std::vector<SE2State>& wp,
                                         double t, Trajectory* traj, SegmentCache& cache) {
  SegmentCacheKey key(idx, false);
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = OptimizeLowRiskSegment(wp, t, traj);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
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

  const double min_sample_dt = 0.05;
  const double max_sample_dt = 0.40;
  const size_t min_pose_count = 1000;
  const size_t max_pose_count = 2500;
  const size_t target_pose_count =
      std::max(min_pose_count,
               std::min(max_pose_count,
                        traj.pos_pieces.size() * static_cast<size_t>(2)));
  const double sample_dt =
      total > 1e-9
          ? std::min(max_sample_dt,
                     std::max(min_sample_dt,
                              total / static_cast<double>(target_pose_count)))
          : min_sample_dt;
  const size_t estimated_pose_count =
      total > 1e-9 ? static_cast<size_t>(std::ceil(total / sample_dt)) + 1 : 1;
  path.poses.reserve(estimated_pose_count);
  ROS_INFO("MakeTrajectoryPath: sample_dt=%.3f target_poses=%zu estimated_poses=%zu",
           sample_dt, target_pose_count, estimated_pose_count);

  for (double t = 0.0; t < total; t += sample_dt) {
    const SE2State state = traj.sample(t);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.z = std::sin(state.yaw * 0.5);
    pose.pose.orientation.w = std::cos(state.yaw * 0.5);
    path.poses.push_back(pose);
  }

  const SE2State final_state = traj.sample(total);
  geometry_msgs::PoseStamped final_pose;
  final_pose.header = path.header;
  final_pose.pose.position.x = final_state.x;
  final_pose.pose.position.y = final_state.y;
  final_pose.pose.orientation.z = std::sin(final_state.yaw * 0.5);
  final_pose.pose.orientation.w = std::cos(final_state.yaw * 0.5);
  path.poses.push_back(final_pose);
  return path;
}

}  // namespace esv_planner
