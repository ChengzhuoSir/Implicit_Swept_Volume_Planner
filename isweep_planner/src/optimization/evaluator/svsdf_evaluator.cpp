#include "isweep_planner/optimization/evaluator/svsdf_evaluator.h"
#include "isweep_planner/core/trajectory_sampling.h"

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {
constexpr int kMaxIntervalDepth = 3;
constexpr int kMaxLeafSamples = 32;
constexpr size_t kNearestSegments = 8;

struct ActiveGridSample {
  bool valid = false;
  double clearance = kInf;
  Eigen::Vector2d body_sample = Eigen::Vector2d::Zero();
  Eigen::Vector2d world_sample = Eigen::Vector2d::Zero();
};

struct ActiveGeometryCandidate {
  bool valid = false;
  double clearance = kInf;
  Eigen::Vector2d local_point = Eigen::Vector2d::Zero();
  BodyFrameQuery query;
};

Eigen::Vector2d sampleRotationDerivative(const Eigen::Vector2d& sample,
                                         double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(-s * sample.x() - c * sample.y(),
                         c * sample.x() - s * sample.y());
}

double finiteDifferenceGradient(const SvsdfEvaluator& evaluator,
                                const SE2State& state,
                                double eps_pos,
                                double eps_yaw,
                                Eigen::Vector2d& grad_pos,
                                double& grad_yaw) {
  SE2State sx_plus = state;
  SE2State sx_minus = state;
  sx_plus.x += eps_pos;
  sx_minus.x -= eps_pos;
  grad_pos.x() =
      (evaluator.evaluate(sx_plus) - evaluator.evaluate(sx_minus)) /
      (2.0 * eps_pos);

  SE2State sy_plus = state;
  SE2State sy_minus = state;
  sy_plus.y += eps_pos;
  sy_minus.y -= eps_pos;
  grad_pos.y() =
      (evaluator.evaluate(sy_plus) - evaluator.evaluate(sy_minus)) /
      (2.0 * eps_pos);

  SE2State syaw_plus = state;
  SE2State syaw_minus = state;
  syaw_plus.yaw += eps_yaw;
  syaw_minus.yaw -= eps_yaw;
  grad_yaw =
      (evaluator.evaluate(syaw_plus) - evaluator.evaluate(syaw_minus)) /
      (2.0 * eps_yaw);

  return eps_pos;
}
}  // namespace

// --- Initialization ---

void SvsdfEvaluator::initGridEsdf(const GridMap& map,
                                   const FootprintModel& footprint) {
  mode_ = Backend::GridEsdf;
  map_ = &map;
  footprint_ = &footprint;
  geometry_map_ = &map.geometryMap();
}

// --- Core evaluate dispatch ---

double SvsdfEvaluator::evaluate(const SE2State& state) const {
  if (mode_ == Backend::GeometryBodyFrame) {
    return evaluateGeometry(state);
  }
  return evaluateGridEsdf(state);
}

// --- GridEsdf backend ---

double SvsdfEvaluator::evaluateGridEsdf(const SE2State& state) const {
  if (!map_ || !footprint_) return -kInf;

  const double res = map_->resolution();
  const auto& samples = footprint_->denseBodySamples(res, 2.0 * res);
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  double min_dist = kInf;
  for (const auto& sample : samples) {
    const double wx = state.x + c * sample.x() - s * sample.y();
    const double wy = state.y + s * sample.x() + c * sample.y();
    min_dist = std::min(min_dist, map_->getEsdf(wx, wy));
  }
  return min_dist;
}

ActiveGridSample findActiveGridSample(const GridMap& map,
                                      const FootprintModel& footprint,
                                      const SE2State& state) {
  ActiveGridSample active;
  const double res = map.resolution();
  const auto& samples = footprint.denseBodySamples(res, 2.0 * res);
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  for (const auto& sample : samples) {
    const double wx = state.x + c * sample.x() - s * sample.y();
    const double wy = state.y + s * sample.x() + c * sample.y();
    const double clearance = map.getEsdf(wx, wy);
    if (clearance < active.clearance) {
      active.valid = true;
      active.clearance = clearance;
      active.body_sample = sample;
      active.world_sample = Eigen::Vector2d(wx, wy);
    }
  }
  return active;
}

Eigen::Vector2d estimateEsdfGradient(const GridMap& map,
                                     const Eigen::Vector2d& world_point) {
  const GridIndex cell = map.worldToGrid(world_point.x(), world_point.y());
  if (!map.isInside(cell.x, cell.y)) {
    return Eigen::Vector2d::Zero();
  }

  const double res = std::max(1e-9, map.resolution());
  const auto sample_axis = [&map, res](int gx, int gy, int dx, int dy) {
    const bool has_neg = map.isInside(gx - dx, gy - dy);
    const bool has_pos = map.isInside(gx + dx, gy + dy);
    if (has_neg && has_pos) {
      const double pos = map.getEsdfCell(gx + dx, gy + dy);
      const double neg = map.getEsdfCell(gx - dx, gy - dy);
      return (pos - neg) / (2.0 * res);
    }
    if (has_pos) {
      const double pos = map.getEsdfCell(gx + dx, gy + dy);
      const double center = map.getEsdfCell(gx, gy);
      return (pos - center) / res;
    }
    if (has_neg) {
      const double center = map.getEsdfCell(gx, gy);
      const double neg = map.getEsdfCell(gx - dx, gy - dy);
      return (center - neg) / res;
    }
    return 0.0;
  };

  return Eigen::Vector2d(sample_axis(cell.x, cell.y, 1, 0),
                         sample_axis(cell.x, cell.y, 0, 1));
}

// --- GeometryBodyFrame backend ---

double SvsdfEvaluator::evaluateGeometry(const SE2State& state) const {
  if (!geometry_map_ || !footprint_) return -kInf;

  double nearest_dist = kInf;
  const Eigen::Vector2d nearest_world =
      geometry_map_->nearestObstacle(state.position(), &nearest_dist);
  if (!std::isfinite(nearest_dist)) return kInf;

  auto local_segments =
      geometry_map_->queryNearestSegments(state.position(), kNearestSegments);

  const bool use_segment_candidates =
      !local_segments.empty() &&
      nearest_dist <= footprint_->circumscribedRadius() + 0.25;

  if (!use_segment_candidates) {
    Eigen::Matrix<double, 1, 2> single;
    single.row(0) =
        rotateIntoBody(nearest_world - state.position(), state.yaw).transpose();
    return footprint_->bodyFrameSdfModel().queryBatch(single).front().signed_distance;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 2> candidates(
      static_cast<Eigen::Index>(local_segments.size() * 3 + 1), 2);
  Eigen::Index row = 0;
  candidates.row(row++) =
      rotateIntoBody(nearest_world - state.position(), state.yaw).transpose();
  for (const auto& segment : local_segments) {
    const Eigen::Vector2d projected =
        geometry_map_detail::projectToSegment(state.position(), segment);
    candidates.row(row++) =
        rotateIntoBody(projected - state.position(), state.yaw).transpose();
    candidates.row(row++) =
        rotateIntoBody(segment.a - state.position(), state.yaw).transpose();
    candidates.row(row++) =
        rotateIntoBody(segment.b - state.position(), state.yaw).transpose();
  }
  candidates.conservativeResize(row, 2);

  const auto queries = footprint_->bodyFrameSdfModel().queryBatch(candidates);
  double min_clearance = kInf;
  for (const auto& query : queries) {
    min_clearance = std::min(min_clearance, query.signed_distance);
  }
  return min_clearance;
}

ActiveGeometryCandidate findActiveGeometryCandidate(
    const GeometryMap& geometry_map, const FootprintModel& footprint,
    const SE2State& state) {
  ActiveGeometryCandidate active;

  double nearest_dist = kInf;
  const Eigen::Vector2d nearest_world =
      geometry_map.nearestObstacle(state.position(), &nearest_dist);
  if (!std::isfinite(nearest_dist)) {
    return active;
  }

  const auto local_segments =
      geometry_map.queryNearestSegments(state.position(), kNearestSegments);
  const bool use_segment_candidates =
      !local_segments.empty() &&
      nearest_dist <= footprint.circumscribedRadius() + 0.25;

  std::vector<Eigen::Vector2d> candidates;
  candidates.reserve(local_segments.size() * 3 + 1);
  candidates.push_back(
      rotateIntoBody(nearest_world - state.position(), state.yaw));

  if (use_segment_candidates) {
    for (const auto& segment : local_segments) {
      const Eigen::Vector2d projected =
          geometry_map_detail::projectToSegment(state.position(), segment);
      candidates.push_back(
          rotateIntoBody(projected - state.position(), state.yaw));
      candidates.push_back(
          rotateIntoBody(segment.a - state.position(), state.yaw));
      candidates.push_back(
          rotateIntoBody(segment.b - state.position(), state.yaw));
    }
  }

  Eigen::Matrix<double, Eigen::Dynamic, 2> candidate_matrix(
      static_cast<Eigen::Index>(candidates.size()), 2);
  for (Eigen::Index i = 0; i < candidate_matrix.rows(); ++i) {
    candidate_matrix.row(i) = candidates[static_cast<size_t>(i)].transpose();
  }

  const auto queries = footprint.bodyFrameSdfModel().queryBatch(candidate_matrix);
  for (size_t i = 0; i < queries.size(); ++i) {
    if (queries[i].signed_distance < active.clearance) {
      active.valid = true;
      active.clearance = queries[i].signed_distance;
      active.local_point = candidates[i];
      active.query = queries[i];
    }
  }

  return active;
}

// --- Trajectory evaluation with adaptive interval subdivision ---

double SvsdfEvaluator::evaluateInterval(const Trajectory& traj,
                                         double t0, double t1,
                                         int depth) const {
  const SE2State s0 = traj.sample(t0);
  const SE2State s1 = traj.sample(t1);
  double min_clearance = std::min(evaluate(s0), evaluate(s1));

  const double linear_delta = (s1.position() - s0.position()).norm();
  const double yaw_delta = std::abs(normalizeAngle(s1.yaw - s0.yaw));
  const double max_linear_step = map_ ? std::max(map_->resolution(), 0.05) : 0.05;
  const double max_yaw_step = 0.25;

  if (depth >= kMaxIntervalDepth) {
    const int leaf_steps = std::min(
        kMaxLeafSamples,
        std::max(1, static_cast<int>(std::ceil(std::max(
            linear_delta / std::max(max_linear_step, 1e-6),
            yaw_delta / std::max(max_yaw_step, 1e-6))))));
    for (int i = 1; i < leaf_steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(leaf_steps);
      min_clearance = std::min(min_clearance, evaluate(traj.sample(t0 + (t1 - t0) * t)));
    }
    return min_clearance;
  }

  const double tm = 0.5 * (t0 + t1);
  min_clearance = std::min(min_clearance, evaluate(traj.sample(tm)));

  if (linear_delta <= max_linear_step && yaw_delta <= max_yaw_step) {
    return min_clearance;
  }

  const double margin_band = map_ ? std::max(0.10, map_->resolution()) : 0.10;
  if (min_clearance > margin_band &&
      linear_delta <= 2.0 * max_linear_step &&
      yaw_delta <= 1.5 * max_yaw_step) {
    return min_clearance;
  }

  min_clearance = std::min(min_clearance, evaluateInterval(traj, t0, tm, depth + 1));
  min_clearance = std::min(min_clearance, evaluateInterval(traj, tm, t1, depth + 1));
  return min_clearance;
}

double SvsdfEvaluator::evaluateTrajectory(const Trajectory& traj) const {
  if (!footprint_ || traj.empty()) return -kInf;

  const double total = traj.totalDuration();
  if (total <= 1e-9) return evaluate(traj.sample(0.0));

  if (mode_ == Backend::GeometryBodyFrame) {
    const auto samples = sampleTrajectoryByArcLength(traj, 0.05, 0.02);
    double min_clearance = kInf;
    for (const auto& state : samples) {
      min_clearance = std::min(min_clearance, evaluate(state));
    }
    return min_clearance;
  }

  return evaluateInterval(traj, 0.0, total, 0);
}

// --- Gradient (semi-analytic with finite-difference fallback) ---

void SvsdfEvaluator::gradient(const SE2State& state,
                               Eigen::Vector2d& grad_pos,
                               double& grad_yaw) const {
  grad_pos.setZero();
  grad_yaw = 0.0;

  if (mode_ == Backend::GridEsdf && map_ && footprint_) {
    const ActiveGridSample active = findActiveGridSample(*map_, *footprint_, state);
    if (active.valid && std::isfinite(active.clearance)) {
      const Eigen::Vector2d world_grad =
          estimateEsdfGradient(*map_, active.world_sample);
      if (world_grad.squaredNorm() > 1e-12) {
        grad_pos = world_grad;
        grad_yaw = world_grad.dot(
            sampleRotationDerivative(active.body_sample, state.yaw));
        return;
      }
    }
  }

  if (mode_ == Backend::GeometryBodyFrame && geometry_map_ && footprint_) {
    const ActiveGeometryCandidate active =
        findActiveGeometryCandidate(*geometry_map_, *footprint_, state);
    if (active.valid && std::isfinite(active.clearance) &&
        active.query.gradient.squaredNorm() > 1e-12) {
      grad_pos = -rotateIntoWorld(active.query.gradient, state.yaw);
      grad_yaw = active.query.gradient.dot(
          Eigen::Vector2d(active.local_point.y(), -active.local_point.x()));
      return;
    }
  }

  const double eps_pos = (mode_ == Backend::GridEsdf && map_)
                             ? map_->resolution() * 0.25
                             : 0.01;
  const double eps_yaw = (mode_ == Backend::GridEsdf) ? 0.005 : 0.01;
  finiteDifferenceGradient(*this, state, eps_pos, eps_yaw, grad_pos, grad_yaw);
}

// --- Feasibility queries ---

double SvsdfEvaluator::transitionClearance(const SE2State& from,
                                            const SE2State& to,
                                            double sample_step) const {
  return interpolatedClearance(from, to, sample_step,
                               [this](const SE2State& s) { return evaluate(s); });
}

double SvsdfEvaluator::segmentClearance(const std::vector<SE2State>& waypoints,
                                         double sample_step) const {
  if (waypoints.size() < 2) return -kInf;
  double min_clearance = kInf;
  for (const auto& state : waypoints) {
    min_clearance = std::min(min_clearance, evaluate(state));
  }
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    min_clearance = std::min(
        min_clearance, transitionClearance(waypoints[i], waypoints[i + 1], sample_step));
  }
  return min_clearance;
}

// --- SafeYaw ---

bool SvsdfEvaluator::safeYaw(SE2State& state, double desired_yaw,
                              const CollisionChecker* checker) const {
  if (!checker) {
    state.yaw = desired_yaw;
    return true;
  }

  const int bins = checker->numYawBins();
  const auto wrap_bin = [bins](int bin) {
    int wrapped = bin % bins;
    return wrapped < 0 ? wrapped + bins : wrapped;
  };

  int desired_bin = checker->binFromYaw(desired_yaw);
  if (checker->isYawSafe(state.x, state.y, desired_bin)) {
    state.yaw = checker->yawFromBin(desired_bin);
    return true;
  }

  for (int offset = 1; offset <= bins / 2; ++offset) {
    for (int sign : {-1, 1}) {
      const int bin = wrap_bin(desired_bin + sign * offset);
      if (checker->isYawSafe(state.x, state.y, bin)) {
        state.yaw = checker->yawFromBin(bin);
        return true;
      }
    }
  }

  return false;
}

}  // namespace isweep_planner
