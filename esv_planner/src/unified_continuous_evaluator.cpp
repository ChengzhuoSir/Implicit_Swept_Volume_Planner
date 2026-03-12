#include "esv_planner/unified_continuous_evaluator.h"

#include <algorithm>
#include <cmath>

namespace esv_planner {

namespace {

constexpr int kMaxGeometryIntervalDepth = 3;
constexpr int kMaxGeometryLeafSamples = 32;

Eigen::Vector2d rotateIntoBody(const Eigen::Vector2d& world_delta, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * world_delta.x() + s * world_delta.y(),
                         -s * world_delta.x() + c * world_delta.y());
}

}  // namespace

void UnifiedContinuousEvaluator::initGridEsdf(const GridMap& map,
                                              const FootprintModel& footprint) {
  mode_ = BackendMode::GridEsdf;
  grid_esdf_evaluator_.init(map, footprint);
  geometry_map_ = &map.geometryMap();
  footprint_ = &footprint;
}

void UnifiedContinuousEvaluator::initGeometryBodyFrame(
    const GeometryMap& geometry_map,
    const FootprintModel& footprint) {
  mode_ = BackendMode::GeometryBodyFrame;
  geometry_map_ = &geometry_map;
  footprint_ = &footprint;
}

double UnifiedContinuousEvaluator::evaluate(const SE2State& state) const {
  if (mode_ == BackendMode::GeometryBodyFrame) {
    return evaluateGeometryState(state);
  }
  return grid_esdf_evaluator_.evaluate(state);
}

double UnifiedContinuousEvaluator::evaluateTrajectory(const Trajectory& traj) const {
  if (mode_ == BackendMode::GeometryBodyFrame) {
    return evaluateGeometryTrajectory(traj);
  }
  return grid_esdf_evaluator_.evaluateTrajectory(traj);
}

void UnifiedContinuousEvaluator::gradient(const SE2State& state,
                                          Eigen::Vector2d& grad_pos,
                                          double& grad_yaw) const {
  if (mode_ == BackendMode::GeometryBodyFrame) {
    const double eps_pos = 0.01;
    const double eps_yaw = 0.01;

    SE2State sx_plus = state;
    SE2State sx_minus = state;
    sx_plus.x += eps_pos;
    sx_minus.x -= eps_pos;
    grad_pos.x() = (evaluateGeometryState(sx_plus) - evaluateGeometryState(sx_minus)) /
                   (2.0 * eps_pos);

    SE2State sy_plus = state;
    SE2State sy_minus = state;
    sy_plus.y += eps_pos;
    sy_minus.y -= eps_pos;
    grad_pos.y() = (evaluateGeometryState(sy_plus) - evaluateGeometryState(sy_minus)) /
                   (2.0 * eps_pos);

    SE2State syaw_plus = state;
    SE2State syaw_minus = state;
    syaw_plus.yaw += eps_yaw;
    syaw_minus.yaw -= eps_yaw;
    grad_yaw = (evaluateGeometryState(syaw_plus) - evaluateGeometryState(syaw_minus)) /
               (2.0 * eps_yaw);
    return;
  }
  grid_esdf_evaluator_.gradient(state, grad_pos, grad_yaw);
}

double UnifiedContinuousEvaluator::evaluateGeometryState(const SE2State& state) const {
  if (geometry_map_ == nullptr || footprint_ == nullptr) {
    return -kInf;
  }

  const double radius = footprint_->circumscribedRadius() + 0.25;
  auto local_segments = geometry_map_->queryLocalSegments(state.position(), radius, 48);
  if (local_segments.empty()) {
    double nearest_dist = kInf;
    const Eigen::Vector2d nearest_world =
        geometry_map_->nearestObstacle(state.position(), &nearest_dist);
    if (!std::isfinite(nearest_dist)) {
      return kInf;
    }
    Eigen::Matrix<double, 1, 2> single;
    single.row(0) =
        rotateIntoBody(nearest_world - state.position(), state.yaw).transpose();
    return footprint_->bodyFrameSdfModel().queryBatch(single).front().signed_distance;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 2> candidates(
      static_cast<Eigen::Index>(local_segments.size() * 3), 2);
  Eigen::Index row = 0;
  for (const auto& segment : local_segments) {
    const Eigen::Vector2d projected =
        geometry_map_detail::projectToSegment(state.position(), segment);
    const Eigen::Vector2d body_projected =
        rotateIntoBody(projected - state.position(), state.yaw);
    const Eigen::Vector2d body_a =
        rotateIntoBody(segment.a - state.position(), state.yaw);
    const Eigen::Vector2d body_b =
        rotateIntoBody(segment.b - state.position(), state.yaw);
    candidates.row(row++) = body_projected.transpose();
    candidates.row(row++) = body_a.transpose();
    candidates.row(row++) = body_b.transpose();
  }
  candidates.conservativeResize(row, 2);

  const auto queries = footprint_->bodyFrameSdfModel().queryBatch(candidates);
  double min_clearance = kInf;
  for (const auto& query : queries) {
    min_clearance = std::min(min_clearance, query.signed_distance);
  }
  return min_clearance;
}

double UnifiedContinuousEvaluator::evaluateGeometryTrajectory(const Trajectory& traj) const {
  if (traj.empty()) {
    return -kInf;
  }

  const double total = traj.totalDuration();
  if (total <= 1e-9) {
    return evaluateGeometryState(traj.sample(0.0));
  }

  std::function<double(double, double, int)> eval_interval =
      [&](double t0, double t1, int depth) -> double {
    const SE2State s0 = traj.sample(t0);
    const SE2State s1 = traj.sample(t1);
    double min_clearance = std::min(evaluateGeometryState(s0), evaluateGeometryState(s1));

    const double linear_delta = (s1.position() - s0.position()).norm();
    const double yaw_delta = std::abs(normalizeAngle(s1.yaw - s0.yaw));
    const double max_linear_step = 0.05;
    const double max_yaw_step = 0.25;

    if (depth >= kMaxGeometryIntervalDepth) {
      const int leaf_steps = std::min(
          kMaxGeometryLeafSamples,
          std::max(1, static_cast<int>(std::ceil(std::max(
              linear_delta / std::max(max_linear_step, 1e-6),
              yaw_delta / std::max(max_yaw_step, 1e-6))))));
      for (int i = 1; i < leaf_steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(leaf_steps);
        min_clearance =
            std::min(min_clearance, evaluateGeometryState(traj.sample(t0 + (t1 - t0) * t)));
      }
      return min_clearance;
    }

    const double tm = 0.5 * (t0 + t1);
    min_clearance = std::min(min_clearance, evaluateGeometryState(traj.sample(tm)));
    if (linear_delta <= max_linear_step && yaw_delta <= max_yaw_step) {
      return min_clearance;
    }
    min_clearance = std::min(min_clearance, eval_interval(t0, tm, depth + 1));
    min_clearance = std::min(min_clearance, eval_interval(tm, t1, depth + 1));
    return min_clearance;
  };

  return eval_interval(0.0, total, 0);
}

}  // namespace esv_planner
