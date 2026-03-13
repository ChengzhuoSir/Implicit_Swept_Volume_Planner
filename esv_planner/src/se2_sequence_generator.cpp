#include "esv_planner/se2_sequence_generator.h"
#include <cmath>
#include <algorithm>

namespace esv_planner {

// ---------------------------------------------------------------------------
// Construction / initialization
// ---------------------------------------------------------------------------

SE2SequenceGenerator::SE2SequenceGenerator() {}

void SE2SequenceGenerator::init(const GridMap& map,
                                const CollisionChecker& checker,
                                const SvsdfEvaluator& evaluator,
                                double discretization_step,
                                int max_push_attempts) {
  map_ = &map;
  checker_ = &checker;
  evaluator_ = &evaluator;
  disc_step_ = discretization_step;
  max_push_ = max_push_attempts;
}

// ---------------------------------------------------------------------------
// UniformDiscretize: topo path -> SE2 waypoints with tangent yaw
// ---------------------------------------------------------------------------

std::vector<SE2State> SE2SequenceGenerator::discretizePath(
    const TopoPath& path,
    const SE2State& start,
    const SE2State& goal) {
  std::vector<SE2State> states;
  if (path.waypoints.size() < 2) return states;

  states.push_back(start);

  for (size_t i = 1; i < path.waypoints.size(); ++i) {
    const Eigen::Vector2d dir = path.waypoints[i].pos - path.waypoints[i - 1].pos;
    const double seg_len = dir.norm();
    if (seg_len < 1e-9) continue;

    double tangent_yaw = std::atan2(dir.y(), dir.x());
    if (path.waypoints[i].has_yaw) {
      tangent_yaw = path.waypoints[i].yaw;
    }

    const int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / disc_step_)));
    for (int s = 1; s <= n_steps; ++s) {
      const double t = static_cast<double>(s) / n_steps;
      const Eigen::Vector2d p = path.waypoints[i - 1].pos + t * dir;
      SE2State st(p.x(), p.y(), tangent_yaw);

      // Last point of last segment: use goal yaw
      if (i == path.waypoints.size() - 1 && s == n_steps) {
        st.yaw = goal.yaw;
      }
      states.push_back(st);
    }
  }

  if (!states.empty()) {
    states.back() = goal;
  }
  return states;
}

// ---------------------------------------------------------------------------
// SafeYaw: delegate to evaluator
// ---------------------------------------------------------------------------

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  return evaluator_->safeYaw(state, desired_yaw, checker_);
}

// ---------------------------------------------------------------------------
// PushAway: ESDF gradient ascent push (Algorithm 1/2)
// ---------------------------------------------------------------------------

bool SE2SequenceGenerator::pushStateFromObstacle(SE2State& state,
                                                  double desired_yaw) {
  const double eps = map_->resolution();
  const double radial_step = std::max(eps, 0.5 * disc_step_);
  constexpr int num_rings = 3;

  for (int attempt = 0; attempt < max_push_; ++attempt) {
    // Try safeYaw at current position first
    SE2State trial = state;
    if (safeYaw(trial, desired_yaw)) {
      state = trial;
      return true;
    }

    // Compute ESDF gradient direction
    const Eigen::Vector2d pos = state.position();
    const double ex_p = map_->getEsdf(pos.x() + eps, pos.y());
    const double ex_n = map_->getEsdf(pos.x() - eps, pos.y());
    const double ey_p = map_->getEsdf(pos.x(), pos.y() + eps);
    const double ey_n = map_->getEsdf(pos.x(), pos.y() - eps);
    Eigen::Vector2d grad(ex_p - ex_n, ey_p - ey_n);

    // If central differences fail, try radial sampling
    if (grad.norm() < 1e-9) {
      const double base = map_->getEsdf(pos.x(), pos.y());
      double best_gain = 0.0;
      for (int k = 0; k < 16; ++k) {
        const double theta = kTwoPi * static_cast<double>(k) / 16.0;
        const Eigen::Vector2d d(std::cos(theta), std::sin(theta));
        for (double scale : {1.0, 2.0, 3.0}) {
          const double val = map_->getEsdf(pos.x() + scale * eps * d.x(),
                                           pos.y() + scale * eps * d.y());
          if (val - base > best_gain) {
            best_gain = val - base;
            grad = d;
          }
        }
      }
    } else {
      grad.normalize();
    }

    // Build candidate push directions
    const Eigen::Vector2d desired_dir(std::cos(desired_yaw), std::sin(desired_yaw));
    std::vector<Eigen::Vector2d> dirs;
    dirs.reserve(6);
    auto addDir = [&](const Eigen::Vector2d& d) {
      if (d.norm() < 1e-9) return;
      const Eigen::Vector2d u = d.normalized();
      for (const auto& e : dirs) {
        if (std::abs(e.dot(u)) > 0.985) return;
      }
      dirs.push_back(u);
    };
    addDir(grad);
    addDir(desired_dir);
    addDir(-desired_dir);
    if (grad.norm() > 1e-9) {
      addDir(Eigen::Vector2d(-grad.y(), grad.x()));
      addDir(Eigen::Vector2d(grad.y(), -grad.x()));
    }

    // Search rings for a safe nearby state
    bool found = false;
    SE2State best = state;
    double best_score = -kInf;
    for (int ring = 1; ring <= num_rings; ++ring) {
      const double radius = ring * radial_step;
      for (const auto& d : dirs) {
        SE2State nearby(state.x + radius * d.x(),
                        state.y + radius * d.y(),
                        desired_yaw);
        if (!safeYaw(nearby, desired_yaw)) continue;
        const double yaw_penalty = std::abs(normalizeAngle(nearby.yaw - desired_yaw));
        const double score = evaluator_->evaluate(nearby) - 0.35 * radius - 0.1 * yaw_penalty;
        if (!found || score > best_score) {
          found = true;
          best_score = score;
          best = nearby;
        }
      }
    }
    if (found) {
      state = best;
      return true;
    }

    // Fall back: step along gradient
    if (grad.norm() < 1e-9) return false;
    const double esdf = map_->getEsdf(state.x, state.y);
    const double push = std::min(std::max(eps, std::abs(std::min(0.0, esdf)) + eps),
                                 4.0 * eps);
    state.x += push * grad.x();
    state.y += push * grad.y();
    state.yaw = desired_yaw;
  }

  return safeYaw(state, desired_yaw);
}

// ---------------------------------------------------------------------------
// buildLinearSegment: linear interpolation helper for SegAdjust
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// bridgeUnsafeTransition: single midpoint bridge insertion
// ---------------------------------------------------------------------------

bool SE2SequenceGenerator::bridgeUnsafeTransition(
    const SE2State& from,
    const SE2State& to,
    std::vector<SE2State>& bridge) {
  bridge.clear();

  // Check if already safe
  const double tc = evaluator_->transitionClearance(from, to, disc_step_);
  if (tc >= 0.0 && checker_->isFree(from) && checker_->isFree(to)) {
    bridge = {from, to};
    return true;
  }

  const Eigen::Vector2d delta = to.position() - from.position();
  if (delta.norm() < 1e-9) return false;

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  for (double ratio : {0.5, 0.35, 0.65}) {
    SE2State mid(from.x + ratio * delta.x(),
                 from.y + ratio * delta.y(),
                 tangent_yaw);
    if (!pushStateFromObstacle(mid, tangent_yaw)) continue;

    const double tc1 = evaluator_->transitionClearance(from, mid, disc_step_);
    const double tc2 = evaluator_->transitionClearance(mid, to, disc_step_);
    if (tc1 >= 0.0 && tc2 >= 0.0) {
      bridge = {from, mid, to};
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// segAdjustRecursive: recursive segment repair (Algorithm 2 core)
// ---------------------------------------------------------------------------

bool SE2SequenceGenerator::segAdjustRecursive(
    const std::vector<SE2State>& seed,
    int depth,
    std::vector<SE2State>& repaired) {
  if (seed.size() < 2) return false;
  constexpr int max_depth = 4;
  if (seed.size() > 8) return false;

  // Try to validate the seed as-is: safeYaw each interior point, check transitions
  std::vector<SE2State> trial = seed;
  size_t fail_idx = trial.size();

  for (size_t i = 1; i + 1 < trial.size(); ++i) {
    SE2State assigned = trial[i];
    if (!safeYaw(assigned, seed[i].yaw)) {
      fail_idx = i;
      break;
    }
    trial[i] = assigned;
    const double tc = evaluator_->transitionClearance(trial[i - 1], trial[i], disc_step_);
    if (tc < 0.0) {
      fail_idx = i;
      break;
    }
  }

  // Also check remaining transitions if no failure yet
  if (fail_idx == trial.size()) {
    for (size_t i = 1; i < trial.size(); ++i) {
      const double tc = evaluator_->transitionClearance(trial[i - 1], trial[i], disc_step_);
      if (tc < 0.0) {
        fail_idx = i;
        break;
      }
    }
  }

  // All safe: done
  if (fail_idx == trial.size()) {
    repaired = trial;
    return true;
  }

  // Recursion limit
  if (depth >= max_depth || trial.size() < 3) return false;

  // Push the failing point away from obstacles
  SE2State pivot = seed[fail_idx];
  if (!pushStateFromObstacle(pivot, seed[fail_idx].yaw)) return false;

  // Split into left/right sub-segments and recurse
  const auto left_seed = buildLinearSegment(trial.front(), pivot);
  const auto right_seed = buildLinearSegment(pivot, trial.back());

  std::vector<SE2State> left_repaired, right_repaired;
  if (!segAdjustRecursive(left_seed, depth + 1, left_repaired)) return false;
  if (!segAdjustRecursive(right_seed, depth + 1, right_repaired)) return false;

  // Merge left and right, bridging any unsafe joins
  repaired = left_repaired;
  for (size_t i = 1; i < right_repaired.size(); ++i) {
    const double tc = evaluator_->transitionClearance(repaired.back(), right_repaired[i], disc_step_);
    if (tc >= 0.0) {
      repaired.push_back(right_repaired[i]);
      continue;
    }
    std::vector<SE2State> bridge;
    if (!bridgeUnsafeTransition(repaired.back(), right_repaired[i], bridge)) return false;
    repaired.insert(repaired.end(), bridge.begin() + 1, bridge.end());
  }
  return true;
}

// ---------------------------------------------------------------------------
// generateCoreSegments: SafeYaw -> SegAdjust -> LOW/HIGH classification
// ---------------------------------------------------------------------------

std::vector<MotionSegment> SE2SequenceGenerator::generateCoreSegments(
    const std::vector<SE2State>& states) {
  std::vector<MotionSegment> segments;
  if (states.size() < 2) return segments;

  MotionSegment current;
  current.risk = RiskLevel::LOW;
  current.waypoints.push_back(states.front());

  constexpr size_t repair_horizon = 4;
  size_t i = 1;

  while (i < states.size()) {
    SE2State assigned = states[i];
    const bool yaw_ok = safeYaw(assigned, states[i].yaw);
    const bool edge_ok = yaw_ok &&
        evaluator_->transitionClearance(current.waypoints.back(), assigned, disc_step_) >= 0.0;

    if (yaw_ok && edge_ok) {
      current.waypoints.push_back(assigned);
      ++i;
      continue;
    }

    // Try SegAdjust over a short look-ahead window
    const size_t search_end = std::min(states.size(), i + repair_horizon);
    bool repaired_ok = false;
    size_t repaired_end = i;

    for (size_t j = i + 1; j <= search_end; ++j) {
      // Build seed: [last safe state, states[i..j-1], anchor at j or end]
      SE2State anchor = (j < states.size()) ? states[j] : states.back();
      if (j < states.size() && j != states.size() - 1) {
        if (!safeYaw(anchor, states[j].yaw)) continue;
      }

      std::vector<SE2State> seed;
      seed.push_back(current.waypoints.back());
      for (size_t k = i; k < j; ++k) seed.push_back(states[k]);
      seed.push_back(anchor);

      std::vector<SE2State> repaired;
      if (segAdjustRecursive(seed, 0, repaired)) {
        // Append repaired states (skip duplicate first element)
        for (size_t r = 1; r < repaired.size(); ++r) {
          current.waypoints.push_back(repaired[r]);
        }
        repaired_ok = true;
        repaired_end = j;
        break;
      }
    }

    if (repaired_ok) {
      i = repaired_end + 1;
      continue;
    }

    // SegAdjust failed: emit current LOW segment, start a HIGH segment
    if (current.waypoints.size() > 1) {
      segments.push_back(current);
    }

    // Determine HIGH segment extent
    const size_t high_end = std::min(states.size() - 1, i + 2);
    SE2State high_anchor = states[high_end];
    safeYaw(high_anchor, states[high_end].yaw);

    MotionSegment high;
    high.risk = RiskLevel::HIGH;
    high.waypoints.push_back(current.waypoints.back());
    for (size_t k = i; k < high_end; ++k) {
      SE2State s = states[k];
      pushStateFromObstacle(s, states[k].yaw);
      high.waypoints.push_back(s);
    }
    high.waypoints.push_back(high_anchor);
    segments.push_back(high);

    // Start new LOW segment from the HIGH anchor
    current = MotionSegment();
    current.risk = RiskLevel::LOW;
    current.waypoints.push_back(high.waypoints.back());
    i = high_end + 1;
  }

  // Flush remaining LOW segment
  if (current.waypoints.size() >= 2) {
    segments.push_back(current);
  }

  return segments;
}

// ---------------------------------------------------------------------------
// generate: entry point (Algorithm 2)
//   discretizePath -> generateCoreSegments -> merge adjacent same-risk
// ---------------------------------------------------------------------------

std::vector<MotionSegment> SE2SequenceGenerator::generate(
    const TopoPath& path,
    const SE2State& start,
    const SE2State& goal) {
  if (path.waypoints.size() < 2) return {};

  auto states = discretizePath(path, start, goal);
  if (states.size() < 2) return {};

  auto segments = generateCoreSegments(states);

  // Merge adjacent segments with the same risk level
  std::vector<MotionSegment> merged;
  merged.reserve(segments.size());
  for (auto& seg : segments) {
    if (seg.waypoints.size() < 2) continue;
    if (!merged.empty() && merged.back().risk == seg.risk) {
      // Append waypoints, skipping duplicate junction point
      const bool dup =
          (merged.back().waypoints.back().position() - seg.waypoints.front().position()).norm() < 1e-6 &&
          std::abs(normalizeAngle(merged.back().waypoints.back().yaw - seg.waypoints.front().yaw)) < 1e-6;
      merged.back().waypoints.insert(
          merged.back().waypoints.end(),
          seg.waypoints.begin() + (dup ? 1 : 0),
          seg.waypoints.end());
    } else {
      merged.push_back(std::move(seg));
    }
  }

  return merged;
}

}  // namespace esv_planner
