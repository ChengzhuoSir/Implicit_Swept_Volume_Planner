#include "isweep_planner/core/trajectory_utils.h"

namespace isweep_planner {


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


}  // namespace isweep_planner
