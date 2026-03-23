#include "isweep_planner/optimization/midend/se2_sequence_generator.h"
#include "isweep_planner/core/math_utils.h"

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {



int WrappedBinDistance(int lhs, int rhs, int bins) {
  const int raw = std::abs(lhs - rhs);
  return std::min(raw, bins - raw);
}

bool IsHighRiskConfiguration(const GridMap& map, const CollisionChecker& checker,
                             const SE2State& desired, const SE2State& assigned) {
  const double esdf = map.getEsdf(desired.x, desired.y);
  const int safe_count =
      static_cast<int>(checker.safeYawCount(desired.x, desired.y));
  const int bins = checker.numYawBins();
  const int desired_bin = checker.binFromYaw(desired.yaw);
  const int assigned_bin = checker.binFromYaw(assigned.yaw);
  const int yaw_distance = WrappedBinDistance(desired_bin, assigned_bin, bins);

  const double critical_clearance =
      std::max(2.0 * checker.footprint().inscribedRadius(),
               1.35 * checker.footprint().circumscribedRadius());
  const double caution_clearance =
      std::max(2.5 * checker.footprint().inscribedRadius(),
               1.65 * checker.footprint().circumscribedRadius());
  const int constrained_yaw_threshold =
      std::max(2, static_cast<int>(std::ceil(0.30 * static_cast<double>(bins))));
  const int limited_yaw_threshold =
      std::max(3, static_cast<int>(std::ceil(0.45 * static_cast<double>(bins))));

  const bool severe_clearance = !std::isfinite(esdf) || esdf < critical_clearance;
  const bool caution_clearance_hit = esdf < caution_clearance;
  const bool constrained_yaw = safe_count <= constrained_yaw_threshold;
  const bool limited_yaw = safe_count <= limited_yaw_threshold;
  const bool strong_yaw_mismatch = yaw_distance > 4;

  return severe_clearance || constrained_yaw ||
         (caution_clearance_hit && (limited_yaw || strong_yaw_mismatch));
}

 double AdaptiveDiscretizationStep(const GridMap& map, double base_step,
                                  const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& goal) {
  const Eigen::Vector2d midpoint = 0.5 * (start + goal);
  const double min_clearance = std::min(
      map.getEsdf(start.x(), start.y()),
      std::min(map.getEsdf(goal.x(), goal.y()),
               map.getEsdf(midpoint.x(), midpoint.y())));
  const double tight_clearance = std::max(0.40, 3.0 * map.resolution());
  const double open_clearance = std::max(1.20, 8.0 * map.resolution());
  if (!std::isfinite(min_clearance) || min_clearance <= tight_clearance) {
    return std::max(0.75 * base_step, 0.75 * map.resolution());
  }
  if (min_clearance >= open_clearance) {
    return std::max(2.25 * base_step, 1.50 * map.resolution());
  }
  const double ratio = (min_clearance - tight_clearance) /
                       std::max(1e-6, open_clearance - tight_clearance);
  const double scale = 0.75 + 1.50 * ratio;
  return std::max(scale * base_step, 0.75 * map.resolution());
}

void MergeAdjacentLowRiskSegments(std::vector<MotionSegment>* segments) {
  if (segments == nullptr || segments->empty()) {
    return;
  }

  std::vector<MotionSegment> merged;
  merged.reserve(segments->size());
  merged.push_back((*segments)[0]);
  for (size_t i = 1; i < segments->size(); ++i) {
    const MotionSegment& current = (*segments)[i];
    MotionSegment& back = merged.back();
    if (back.risk != RiskLevel::LOW || current.risk != RiskLevel::LOW) {
      merged.push_back(current);
      continue;
    }

    const size_t begin = (!back.waypoints.empty() && !current.waypoints.empty() &&
                          (back.waypoints.back().position() -
                           current.waypoints.front().position()).norm() < 1e-6)
                             ? 1
                             : 0;
    back.waypoints.insert(back.waypoints.end(), current.waypoints.begin() + begin,
                          current.waypoints.end());
  }
  segments->swap(merged);
}

struct FootprintClearanceProbe {
  double clearance = kInf;
  int safe_yaw_count = 0;
  Eigen::Vector2d worst_world = Eigen::Vector2d::Zero();
};

FootprintClearanceProbe EvaluateFootprintClearance(const GridMap& map,
                                                   const CollisionChecker& checker,
                                                   const SE2State& state) {
  FootprintClearanceProbe probe;
  probe.safe_yaw_count =
      static_cast<int>(checker.safeYawCount(state.x, state.y));

  const auto& samples = checker.footprint().denseBodySamples(
      map.resolution() * 0.5, map.resolution());
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  for (const Eigen::Vector2d& body : samples) {
    const Eigen::Vector2d world(state.x + c * body.x() - s * body.y(),
                                state.y + s * body.x() + c * body.y());
    const double clearance = map.getEsdf(world.x(), world.y());
    if (clearance >= probe.clearance) {
      continue;
    }
    probe.clearance = clearance;
    probe.worst_world = world;
  }

  if (!std::isfinite(probe.clearance)) {
    probe.clearance = map.getEsdf(state.x, state.y);
    probe.worst_world = state.position();
  }
  return probe;
}

}  // namespace

SE2SequenceGenerator::SE2SequenceGenerator() {}

void SE2SequenceGenerator::init(const GridMap& map, const CollisionChecker& checker,
                                double discretization_step, int max_push_attempts) {
  map_ = &map;
  checker_ = &checker;
  disc_step_ = discretization_step;
  max_push_ = max_push_attempts;
}

std::vector<SE2State> SE2SequenceGenerator::discretizePath(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  std::vector<SE2State> states;
  if (path.waypoints.size() < 2) {
    return states;
  }

  states.push_back(start);
  for (size_t i = 1; i < path.waypoints.size(); ++i) {
    const Eigen::Vector2d delta = path.waypoints[i].pos - path.waypoints[i - 1].pos;
    const double segment_length = delta.norm();
    if (segment_length < 1e-9) {
      continue;
    }

    double tangent_yaw = std::atan2(delta.y(), delta.x());
    if (path.waypoints[i].has_yaw) {
      tangent_yaw = path.waypoints[i].yaw;
    }

    const double local_step =
        AdaptiveDiscretizationStep(*map_, disc_step_, path.waypoints[i - 1].pos,
                                   path.waypoints[i].pos);
    const int samples =
        std::max(1, static_cast<int>(std::ceil(segment_length / local_step)));
    for (int s = 1; s <= samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(samples);
      const Eigen::Vector2d position = path.waypoints[i - 1].pos + t * delta;
      SE2State state(position.x(), position.y(), tangent_yaw);
      if (i == path.waypoints.size() - 1 && s == samples) {
        state.yaw = goal.yaw;
      }
      states.push_back(state);
    }
  }

  if (!states.empty()) {
    states.back() = goal;
  }
  return states;
}

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  const int bins = checker_->numYawBins();
  const auto wrap_bin = [bins](int bin) {
    int wrapped = bin % bins;
    return wrapped < 0 ? wrapped + bins : wrapped;
  };

  const int desired_bin = checker_->binFromYaw(desired_yaw);
  if (checker_->isYawSafe(state.x, state.y, desired_bin)) {
    state.yaw = checker_->yawFromBin(desired_bin);
    return true;
  }

  for (int offset = 1; offset <= bins / 2; ++offset) {
    for (int sign : {-1, 1}) {
      const int candidate = wrap_bin(desired_bin + sign * offset);
      if (checker_->isYawSafe(state.x, state.y, candidate)) {
        state.yaw = checker_->yawFromBin(candidate);
        return true;
      }
    }
  }

  return false;
}

bool SE2SequenceGenerator::pushStateFromObstacle(SE2State& state, double desired_yaw) {
  const double eps = map_->resolution();
  const double target_clearance =
      checker_->footprint().inscribedRadius() + map_->resolution();
  for (int attempt = 0; attempt < max_push_; ++attempt) {
    SE2State assigned = state;
    if (safeYaw(assigned, desired_yaw)) {
      state = assigned;
      return true;
    }

    bool found_nearby_safe = false;
    SE2State best_nearby = state;
    double best_score = -kInf;
    const double radius = static_cast<double>(attempt + 1) * eps;
    for (int k = 0; k < 24; ++k) {
      const double theta = kTwoPi * static_cast<double>(k) / 24.0;
      SE2State nearby(state.x + radius * std::cos(theta), state.y + radius * std::sin(theta),
                      desired_yaw);
      SE2State nearby_assigned = nearby;
      if (!safeYaw(nearby_assigned, desired_yaw)) {
        continue;
      }
      const FootprintClearanceProbe probe =
          EvaluateFootprintClearance(*map_, *checker_, nearby_assigned);
      const double score =
          probe.clearance + 0.1 * static_cast<double>(probe.safe_yaw_count);
      if (!found_nearby_safe || score > best_score) {
        found_nearby_safe = true;
        best_score = score;
        best_nearby = nearby_assigned;
      }
    }
    if (found_nearby_safe) {
      state = best_nearby;
      return true;
    }

    const FootprintClearanceProbe probe =
        EvaluateFootprintClearance(*map_, *checker_, state);
    Eigen::Vector2d direction =
        math::EsdfAscentDirection(*map_, probe.worst_world, eps);
    if (direction.norm() < 1e-9) {
      direction = math::EsdfAscentDirection(*map_, state.position(), eps);
    }
    if (direction.norm() < 1e-9) {
      return false;
    }

    const double deficit = std::max(0.0, target_clearance - probe.clearance);
    const double push = std::min(std::max(eps, deficit + eps), 4.0 * eps);
    state.x += push * direction.x();
    state.y += push * direction.y();
    state.yaw = desired_yaw;
  }

  return safeYaw(state, desired_yaw);
}

std::vector<SE2State> SE2SequenceGenerator::buildLinearSegment(const SE2State& start,
                                                               const SE2State& goal) const {
  std::vector<SE2State> segment;
  segment.push_back(start);

  const Eigen::Vector2d delta = goal.position() - start.position();
  const double length = delta.norm();
  if (length < 1e-9) {
    segment.push_back(goal);
    return segment;
  }

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  const int samples = std::max(1, static_cast<int>(std::ceil(length / disc_step_)));
  for (int s = 1; s < samples; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(samples);
    const Eigen::Vector2d position = start.position() + t * delta;
    segment.push_back(SE2State(position.x(), position.y(), tangent_yaw));
  }

  segment.push_back(goal);
  return segment;
}

bool SE2SequenceGenerator::segAdjustRecursive(const std::vector<SE2State>& seed, int depth,
                                              std::vector<SE2State>& repaired) {
  if (seed.size() < 2) {
    return false;
  }

  const int max_depth = std::max(8, max_push_ * 3);
  std::vector<SE2State> trial = seed;
  size_t fail_index = trial.size();
  for (size_t i = 1; i + 1 < trial.size(); ++i) {
    SE2State assigned = trial[i];
    if (!safeYaw(assigned, seed[i].yaw)) {
      fail_index = i;
      break;
    }
    trial[i] = assigned;
  }

  if (fail_index == trial.size()) {
    repaired = trial;
    return true;
  }

  if (depth >= max_depth || trial.size() < 3) {
    return false;
  }

  SE2State pivot = seed[fail_index];
  if (!pushStateFromObstacle(pivot, seed[fail_index].yaw)) {
    return false;
  }

  std::vector<SE2State> left;
  std::vector<SE2State> right;
  if (!segAdjustRecursive(buildLinearSegment(trial.front(), pivot), depth + 1, left)) {
    return false;
  }
  if (!segAdjustRecursive(buildLinearSegment(pivot, trial.back()), depth + 1, right)) {
    return false;
  }

  repaired = left;
  repaired.insert(repaired.end(), right.begin() + 1, right.end());
  return true;
}

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                          const SE2State& start,
                                                          const SE2State& goal) {
  if (path.waypoints.size() < 2) {
    return {};
  }

  std::vector<SE2State> states = discretizePath(path, start, goal);
  if (states.size() < 2) {
    return {};
  }

  std::vector<MotionSegment> segments;
  MotionSegment current;
  current.risk = RiskLevel::LOW;
  current.waypoints.push_back(states.front());

  size_t i = 1;
  while (i + 1 < states.size()) {
    SE2State assigned = states[i];
    const bool has_safe_yaw = safeYaw(assigned, states[i].yaw);
    const bool high_risk =
        !has_safe_yaw || IsHighRiskConfiguration(*map_, *checker_, states[i], assigned);
    if (!high_risk) {
      current.waypoints.push_back(assigned);
      ++i;
      continue;
    }

    if (current.waypoints.size() > 1) {
      segments.push_back(current);
    }

    size_t next_safe_index = i;
    SE2State next_safe = states[next_safe_index];
    bool found_next_anchor = false;
    for (; next_safe_index < states.size(); ++next_safe_index) {
      next_safe = states[next_safe_index];
      const bool candidate_safe = safeYaw(next_safe, states[next_safe_index].yaw);
      const bool candidate_high =
          !candidate_safe ||
          IsHighRiskConfiguration(*map_, *checker_, states[next_safe_index], next_safe);
      if (next_safe_index == states.size() - 1 || !candidate_high) {
        found_next_anchor = true;
        break;
      }
    }
    if (!found_next_anchor) {
      next_safe_index = states.size() - 1;
      next_safe = states.back();
    }

    std::vector<SE2State> seed;
    seed.reserve(next_safe_index - i + 2);
    seed.push_back(current.waypoints.back());
    for (size_t k = i; k <= next_safe_index; ++k) {
      seed.push_back(states[k]);
    }
    seed.back() = next_safe;

    std::vector<SE2State> repaired;
    if (segAdjustRecursive(seed, 0, repaired)) {
      bool repaired_high_risk = false;
      for (size_t k = 1; k + 1 < repaired.size(); ++k) {
        SE2State assigned = repaired[k];
        if (!safeYaw(assigned, repaired[k].yaw) ||
            IsHighRiskConfiguration(*map_, *checker_, repaired[k], assigned)) {
          repaired_high_risk = true;
          break;
        }
        repaired[k] = assigned;
      }

      MotionSegment repaired_segment;
      repaired_segment.risk = repaired_high_risk ? RiskLevel::HIGH : RiskLevel::LOW;
      repaired_segment.waypoints = repaired;
      segments.push_back(repaired_segment);
      i = next_safe_index + 1;
    } else {
      MotionSegment high_risk_segment;
      high_risk_segment.risk = RiskLevel::HIGH;
      high_risk_segment.waypoints = seed;
      segments.push_back(high_risk_segment);
      i = next_safe_index + 1;
    }

    current = MotionSegment();
    current.risk = RiskLevel::LOW;
    current.waypoints.push_back(segments.back().waypoints.back());
  }

  if (current.waypoints.empty()) {
    current.waypoints.push_back(states.front());
  }
  if ((current.waypoints.back().position() - states.back().position()).norm() > 1e-6) {
    current.waypoints.push_back(states.back());
  } else {
    current.waypoints.back() = states.back();
  }
  if (current.waypoints.size() >= 2) {
    segments.push_back(current);
  }

  MergeAdjacentLowRiskSegments(&segments);
  return segments;
}

}  // namespace isweep_planner
