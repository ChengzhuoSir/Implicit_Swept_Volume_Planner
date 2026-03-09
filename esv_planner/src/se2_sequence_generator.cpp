#include "esv_planner/se2_sequence_generator.h"
#include <cmath>
#include <algorithm>

namespace esv_planner {

namespace {

Eigen::Vector2d esdfAscentDirection(const GridMap& map,
                                    const Eigen::Vector2d& world,
                                    double eps) {
  const double ex_p = map.getEsdf(world.x() + eps, world.y());
  const double ex_n = map.getEsdf(world.x() - eps, world.y());
  const double ey_p = map.getEsdf(world.x(), world.y() + eps);
  const double ey_n = map.getEsdf(world.x(), world.y() - eps);
  Eigen::Vector2d grad(ex_p - ex_n, ey_p - ey_n);
  if (grad.norm() > 1e-9) {
    return grad.normalized();
  }

  const double base = map.getEsdf(world.x(), world.y());
  Eigen::Vector2d best = Eigen::Vector2d::Zero();
  double best_gain = 0.0;
  for (int k = 0; k < 16; ++k) {
    const double theta = kTwoPi * static_cast<double>(k) / 16.0;
    const Eigen::Vector2d dir(std::cos(theta), std::sin(theta));
    for (double scale : {1.0, 2.0, 3.0}) {
      const double value = map.getEsdf(world.x() + scale * eps * dir.x(),
                                       world.y() + scale * eps * dir.y());
      const double gain = value - base;
      if (gain > best_gain) {
        best_gain = gain;
        best = dir;
      }
    }
  }
  return best;
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
  if (path.waypoints.size() < 2) return states;

  states.push_back(start);

  for (size_t i = 1; i < path.waypoints.size(); ++i) {
    Eigen::Vector2d dir = path.waypoints[i].pos - path.waypoints[i - 1].pos;
    double seg_len = dir.norm();
    if (seg_len < 1e-9) continue;

    // Tangent yaw for this segment
    double tangent_yaw = std::atan2(dir.y(), dir.x());
    if (path.waypoints[i].has_yaw) {
      tangent_yaw = path.waypoints[i].yaw;
    }

    int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / disc_step_)));
    for (int s = 1; s <= n_steps; ++s) {
      double t = static_cast<double>(s) / n_steps;
      Eigen::Vector2d p = path.waypoints[i - 1].pos + t * dir;

      // Initialize yaw from tangent direction
      SE2State st(p.x(), p.y(), tangent_yaw);

      // For last point of last segment, blend toward goal yaw
      if (i == path.waypoints.size() - 1 && s == n_steps) {
        st.yaw = goal.yaw;
      }

      states.push_back(st);
    }
  }

  // Ensure last state matches goal exactly
  if (!states.empty()) {
    states.back() = goal;
  }

  return states;
}

double SE2SequenceGenerator::footprintClearance(const SE2State& state) const {
  const auto boundary = checker_->footprint().sampleBoundary(
      state.yaw, map_->resolution() * 0.5);
  if (boundary.empty()) {
    return map_->getEsdf(state.x, state.y);
  }

  double min_dist = map_->getEsdf(state.x, state.y);
  for (const auto& sample : boundary) {
    const double dist = map_->getEsdf(state.x + sample.x(), state.y + sample.y());
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

double SE2SequenceGenerator::requiredClearance() const {
  return std::max(2.0 * map_->resolution(), 1e-3);
}

bool SE2SequenceGenerator::transitionSafe(const SE2State& from,
                                          const SE2State& to) const {
  const double len = (to.position() - from.position()).norm();
  const double sample_step = std::max(0.5 * map_->resolution(), 1e-3);
  const int n_steps =
      std::max(1, static_cast<int>(std::ceil(len / sample_step)));
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = from.x + (to.x - from.x) * t;
    st.y = from.y + (to.y - from.y) * t;
    st.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
    if (footprintClearance(st) < 0.0) {
      return false;
    }
  }
  return true;
}

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  const int bins = checker_->numYawBins();
  const double required_clearance = requiredClearance();
  const auto wrap_bin = [bins](int bin) {
    int wrapped = bin % bins;
    return wrapped < 0 ? wrapped + bins : wrapped;
  };

  int desired_bin = checker_->binFromYaw(desired_yaw);
  if (checker_->isYawSafe(state.x, state.y, desired_bin)) {
    SE2State candidate = state;
    candidate.yaw = checker_->yawFromBin(desired_bin);
    if (footprintClearance(candidate) >= required_clearance) {
      state = candidate;
      return true;
    }
  }

  for (int offset = 1; offset <= bins / 2; ++offset) {
    for (int sign : {-1, 1}) {
      const int bin = wrap_bin(desired_bin + sign * offset);
      if (checker_->isYawSafe(state.x, state.y, bin)) {
        SE2State candidate = state;
        candidate.yaw = checker_->yawFromBin(bin);
        if (footprintClearance(candidate) >= required_clearance) {
          state = candidate;
          return true;
        }
      }
    }
  }

  return false;
}

bool SE2SequenceGenerator::pushStateFromObstacle(SE2State& state,
                                                 double desired_yaw) {
  const double eps = map_->resolution();
  const double radial_step = std::max(eps, 0.5 * disc_step_);
  const int angular_samples = 32;
  const int num_rings = std::max(4, max_push_);
  for (int attempt = 0; attempt < max_push_; ++attempt) {
    SE2State trial = state;
    if (safeYaw(trial, desired_yaw)) {
      state = trial;
      return true;
    }

    bool found_nearby_safe = false;
    SE2State best_nearby = state;
    double best_score = -kInf;
    for (int ring = 1; ring <= num_rings; ++ring) {
      const double radius = ring * radial_step;
      for (int k = 0; k < angular_samples; ++k) {
        const double theta = kTwoPi * static_cast<double>(k) /
                             static_cast<double>(angular_samples);
        SE2State nearby(state.x + radius * std::cos(theta),
                        state.y + radius * std::sin(theta),
                        desired_yaw);
        SE2State assigned = nearby;
        if (!safeYaw(assigned, desired_yaw)) {
          continue;
        }
        const int safe_count = static_cast<int>(
            checker_->safeYawIndices(nearby.x, nearby.y).size());
        const double yaw_penalty = std::abs(normalizeAngle(assigned.yaw - desired_yaw));
        const double score = 0.6 * footprintClearance(assigned) +
                             0.15 * static_cast<double>(safe_count) -
                             0.5 * radius - 0.1 * yaw_penalty;
        if (!found_nearby_safe || score > best_score) {
          found_nearby_safe = true;
          best_score = score;
          best_nearby = assigned;
        }
      }
    }
    if (found_nearby_safe) {
      state = best_nearby;
      return true;
    }

    const Eigen::Vector2d dir = esdfAscentDirection(*map_, state.position(), eps);
    if (dir.norm() < 1e-9) {
      return false;
    }

    const double esdf = map_->getEsdf(state.x, state.y);
    const double push = std::min(std::max(eps, std::abs(std::min(0.0, esdf)) + eps),
                                 4.0 * eps);
    state.x += push * dir.x();
    state.y += push * dir.y();
    state.yaw = desired_yaw;
  }

  return safeYaw(state, desired_yaw);
}

std::vector<SE2State> SE2SequenceGenerator::buildLinearSegment(
    const SE2State& start,
    const SE2State& goal) const {
  std::vector<SE2State> segment;
  segment.push_back(start);

  const Eigen::Vector2d delta = goal.position() - start.position();
  const double len = delta.norm();
  if (len < 1e-9) {
    segment.push_back(goal);
    return segment;
  }

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  const int n_steps = std::max(1, static_cast<int>(std::ceil(len / disc_step_)));
  for (int s = 1; s < n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    const Eigen::Vector2d p = start.position() + t * delta;
    segment.emplace_back(p.x(), p.y(), tangent_yaw);
  }

  segment.push_back(goal);
  return segment;
}

bool SE2SequenceGenerator::segAdjustRecursive(const std::vector<SE2State>& seed,
                                              int depth,
                                              std::vector<SE2State>& repaired) {
  if (seed.size() < 2) return false;
  const int max_depth = std::max(8, max_push_ * 3);

  std::vector<SE2State> trial = seed;
  size_t fail_idx = trial.size();
  for (size_t i = 1; i + 1 < trial.size(); ++i) {
    SE2State assigned = trial[i];
    if (!safeYaw(assigned, seed[i].yaw)) {
      fail_idx = i;
      break;
    }
    trial[i] = assigned;
    if (!transitionSafe(trial[i - 1], trial[i])) {
      fail_idx = i;
      break;
    }
  }

  if (fail_idx == trial.size()) {
    for (size_t i = 1; i < trial.size(); ++i) {
      if (!transitionSafe(trial[i - 1], trial[i])) {
        fail_idx = i;
        break;
      }
    }
  }

  if (fail_idx == trial.size()) {
    repaired = trial;
    return true;
  }

  if (depth >= max_depth || trial.size() < 3) {
    return false;
  }

  SE2State pivot = seed[fail_idx];
  if (!pushStateFromObstacle(pivot, seed[fail_idx].yaw)) {
    return false;
  }

  const auto left_seed = buildLinearSegment(trial.front(), pivot);
  const auto right_seed = buildLinearSegment(pivot, trial.back());

  std::vector<SE2State> left_repaired;
  std::vector<SE2State> right_repaired;
  if (!segAdjustRecursive(left_seed, depth + 1, left_repaired)) {
    return false;
  }
  if (!segAdjustRecursive(right_seed, depth + 1, right_repaired)) {
    return false;
  }

  repaired = left_repaired;
  repaired.insert(repaired.end(), right_repaired.begin() + 1, right_repaired.end());
  return true;
}

bool SE2SequenceGenerator::repairShortWindow(const std::vector<SE2State>& seed,
                                             std::vector<SE2State>& repaired) {
  if (seed.size() < 2) return false;
  if (seed.size() > 8) return false;

  repaired = seed;
  for (size_t i = 1; i + 1 < repaired.size(); ++i) {
    if (!pushStateFromObstacle(repaired[i], seed[i].yaw)) {
      return false;
    }
  }

  for (size_t i = 1; i + 1 < repaired.size(); ++i) {
    SE2State assigned = repaired[i];
    if (!safeYaw(assigned, seed[i].yaw)) {
      return false;
    }
    repaired[i] = assigned;
  }

  for (size_t i = 1; i < repaired.size(); ++i) {
    if (!transitionSafe(repaired[i - 1], repaired[i])) {
      return false;
    }
  }

  return true;
}

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  if (path.waypoints.size() < 2) return {};

  // Step 1: Discretize path into SE2 states with tangent yaw
  auto states = discretizePath(path, start, goal);
  if (states.size() < 2) return {};

  std::vector<MotionSegment> segments;
  MotionSegment current;
  current.risk = RiskLevel::LOW;
  current.waypoints.push_back(states.front());

  size_t i = 1;
  while (i + 1 < states.size()) {
    SE2State assigned = states[i];
    const bool yaw_ok = safeYaw(assigned, states[i].yaw);
    const bool edge_ok = yaw_ok &&
                         transitionSafe(current.waypoints.back(), assigned);
    if (yaw_ok && edge_ok) {
      current.waypoints.push_back(assigned);
      ++i;
      continue;
    }

    size_t next_safe_idx = i + 1;
    SE2State next_safe = states[next_safe_idx];
    bool found_next_anchor = false;
    const bool edge_failure = yaw_ok && !edge_ok;
    const size_t search_end = edge_failure
        ? std::min(states.size() - 1, i + 2)
        : states.size() - 1;
    for (; next_safe_idx <= search_end; ++next_safe_idx) {
      next_safe = states[next_safe_idx];
      if (next_safe_idx == states.size() - 1) {
        found_next_anchor = true;
        break;
      }
      if (safeYaw(next_safe, states[next_safe_idx].yaw)) {
        found_next_anchor = true;
        break;
      }
    }

    if (!found_next_anchor) {
      if (edge_failure) {
        next_safe_idx = std::min(states.size() - 1, i + 1);
        next_safe = states[next_safe_idx];
        if (!safeYaw(next_safe, states[next_safe_idx].yaw) &&
            next_safe_idx == states.size() - 1) {
          next_safe = states.back();
        }
      } else {
        next_safe_idx = states.size() - 1;
        next_safe = states.back();
      }
    }

    std::vector<SE2State> seed;
    seed.reserve(next_safe_idx - i + 2);
    seed.push_back(current.waypoints.back());
    for (size_t k = i; k <= next_safe_idx; ++k) {
      seed.push_back(states[k]);
    }
    seed.back() = next_safe;

    std::vector<SE2State> repaired;
    if (segAdjustRecursive(seed, 0, repaired)) {
      for (size_t k = 1; k < repaired.size(); ++k) {
        current.waypoints.push_back(repaired[k]);
      }
      i = next_safe_idx + 1;
      continue;
    }

    if (repairShortWindow(seed, repaired)) {
      for (size_t k = 1; k < repaired.size(); ++k) {
        current.waypoints.push_back(repaired[k]);
      }
      i = next_safe_idx + 1;
      continue;
    }

    if (current.waypoints.size() > 1) {
      segments.push_back(current);
    }

    MotionSegment high;
    high.risk = RiskLevel::HIGH;
    high.waypoints = seed;
    for (int pass = 0; pass < 2; ++pass) {
      for (size_t k = 1; k + 1 < high.waypoints.size(); ++k) {
        SE2State candidate = high.waypoints[k];
        bool ok = safeYaw(candidate, seed[k].yaw);
        if (ok) {
          ok = transitionSafe(high.waypoints[k - 1], candidate) &&
               transitionSafe(candidate, high.waypoints[k + 1]);
        }
        if (!ok) {
          candidate = high.waypoints[k];
          if (pushStateFromObstacle(candidate, seed[k].yaw)) {
            high.waypoints[k] = candidate;
          }
        } else {
          high.waypoints[k] = candidate;
        }
      }
    }
    segments.push_back(high);

    current = MotionSegment();
    current.risk = RiskLevel::LOW;
    current.waypoints.push_back(seed.back());
    i = next_safe_idx + 1;
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

  return segments;
}

}  // namespace esv_planner
