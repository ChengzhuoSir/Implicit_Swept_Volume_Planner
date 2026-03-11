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

bool pointInPolygon(double px, double py,
                    const std::vector<Eigen::Vector2d>& verts) {
  bool inside = false;
  int n = static_cast<int>(verts.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double yi = verts[i].y(), yj = verts[j].y();
    double xi = verts[i].x(), xj = verts[j].x();
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

void appendUniqueDirection(std::vector<Eigen::Vector2d>& dirs,
                           const Eigen::Vector2d& dir) {
  if (dir.norm() < 1e-9) return;
  const Eigen::Vector2d unit = dir.normalized();
  for (const auto& existing : dirs) {
    if (std::abs(existing.dot(unit)) > 0.985) {
      return;
    }
  }
  dirs.push_back(unit);
}

}  // namespace

SE2SequenceGenerator::SE2SequenceGenerator() {}

void SE2SequenceGenerator::init(const GridMap& map, const CollisionChecker& checker,
                                 double discretization_step, int max_push_attempts) {
  init(map, checker, discretization_step, max_push_attempts,
       std::unique_ptr<ContinuousFeasibilityChecker>());
}

void SE2SequenceGenerator::init(
    const GridMap& map, const CollisionChecker& checker,
    double discretization_step, int max_push_attempts,
    std::unique_ptr<ContinuousFeasibilityChecker> feasibility) {
  map_ = &map;
  checker_ = &checker;
  disc_step_ = discretization_step;
  max_push_ = max_push_attempts;
  if (feasibility) {
    feasibility_ = std::move(feasibility);
  } else {
    feasibility_.reset(
        new GridContinuousFeasibilityChecker(map, checker, discretization_step));
  }
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
  return feasibility_->stateClearance(state);
}

double SE2SequenceGenerator::requiredClearance() const {
  return feasibility_->requiredClearance();
}

double SE2SequenceGenerator::transitionClearance(const SE2State& from,
                                                 const SE2State& to) const {
  return feasibility_->transitionClearance(from, to);
}

bool SE2SequenceGenerator::transitionSafe(const SE2State& from,
                                          const SE2State& to) const {
  return feasibility_->transitionFeasible(from, to);
}

bool SE2SequenceGenerator::bridgeUnsafeTransition(
    const SE2State& from,
    const SE2State& to,
    std::vector<SE2State>& bridge) {
  bridge.clear();
  if (transitionSafe(from, to)) {
    bridge = {from, to};
    return true;
  }

  const Eigen::Vector2d delta = to.position() - from.position();
  const double len = delta.norm();
  if (len < 1e-9) {
    return false;
  }

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  for (double ratio : {0.5, 0.35, 0.65}) {
    SE2State mid(from.x + ratio * delta.x(),
                 from.y + ratio * delta.y(),
                 tangent_yaw);
    if (!pushStateFromObstacle(mid, tangent_yaw)) {
      continue;
    }
    if (!transitionSafe(from, mid) || !transitionSafe(mid, to)) {
      continue;
    }
    bridge = {from, mid, to};
    return true;
  }

  return false;
}

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  return feasibility_->safeYaw(state, desired_yaw);
}

bool SE2SequenceGenerator::pushStateFromObstacle(SE2State& state,
                                                 double desired_yaw) {
  const double eps = map_->resolution();
  const double radial_step = std::max(eps, 0.5 * disc_step_);
  const int num_rings = 3;
  for (int attempt = 0; attempt < max_push_; ++attempt) {
    SE2State trial = state;
    if (safeYaw(trial, desired_yaw)) {
      state = trial;
      return true;
    }

    std::vector<Eigen::Vector2d> dirs;
    const Eigen::Vector2d grad_dir =
        esdfAscentDirection(*map_, state.position(), eps);
    const Eigen::Vector2d desired_dir(std::cos(desired_yaw), std::sin(desired_yaw));
    appendUniqueDirection(dirs, grad_dir);
    appendUniqueDirection(dirs, desired_dir);
    appendUniqueDirection(dirs, -desired_dir);
    if (grad_dir.norm() > 1e-9) {
      appendUniqueDirection(dirs, Eigen::Vector2d(-grad_dir.y(), grad_dir.x()));
      appendUniqueDirection(dirs, Eigen::Vector2d(grad_dir.y(), -grad_dir.x()));
    }

    bool found_nearby_safe = false;
    SE2State best_nearby = state;
    double best_score = -kInf;
    for (int ring = 1; ring <= num_rings; ++ring) {
      const double radius = ring * radial_step;
      for (const auto& dir : dirs) {
        SE2State nearby(state.x + radius * dir.x(),
                        state.y + radius * dir.y(),
                        desired_yaw);
        SE2State assigned = nearby;
        if (!safeYaw(assigned, desired_yaw)) {
          continue;
        }
        const double yaw_penalty = std::abs(normalizeAngle(assigned.yaw - desired_yaw));
        const double score =
            footprintClearance(assigned) - 0.35 * radius - 0.1 * yaw_penalty;
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

bool SE2SequenceGenerator::repairStateBetweenNeighbors(
    const SE2State& prev,
    const SE2State& current,
    const SE2State& next,
    double desired_yaw,
    SE2State& repaired) {
  const double radial_step = std::max(map_->resolution(), 0.5 * disc_step_);
  const int num_rings = 3;
  const double required = requiredClearance();

  const Eigen::Vector2d chord = next.position() - prev.position();
  const Eigen::Vector2d chord_dir =
      chord.norm() > 1e-9 ? chord.normalized() : Eigen::Vector2d::Zero();
  const Eigen::Vector2d grad_dir =
      esdfAscentDirection(*map_, current.position(), map_->resolution());
  std::vector<Eigen::Vector2d> dirs;
  appendUniqueDirection(dirs, grad_dir);
  appendUniqueDirection(dirs, chord_dir);
  appendUniqueDirection(dirs, -chord_dir);
  if (chord_dir.norm() > 1e-9) {
    appendUniqueDirection(dirs, Eigen::Vector2d(-chord_dir.y(), chord_dir.x()));
    appendUniqueDirection(dirs, Eigen::Vector2d(chord_dir.y(), -chord_dir.x()));
  }
  if (grad_dir.norm() > 1e-9) {
    appendUniqueDirection(dirs, Eigen::Vector2d(-grad_dir.y(), grad_dir.x()));
    appendUniqueDirection(dirs, Eigen::Vector2d(grad_dir.y(), -grad_dir.x()));
  }

  bool found = false;
  double best_score = -kInf;
  SE2State best = current;

  for (int ring = 0; ring <= num_rings; ++ring) {
    const double radius = ring * radial_step;
    if (ring == 0) {
      SE2State nearby(current.x, current.y, desired_yaw);
      SE2State assigned = nearby;
      if (safeYaw(assigned, desired_yaw)) {
        const double local_clearance = std::min(
            footprintClearance(assigned),
            std::min(transitionClearance(prev, assigned),
                     transitionClearance(assigned, next)));
        const double yaw_penalty =
            std::abs(normalizeAngle(assigned.yaw - desired_yaw));
        const double score = local_clearance - 0.05 * yaw_penalty;
        found = true;
        best_score = score;
        best = assigned;
        if (local_clearance >= required) {
          repaired = assigned;
          return true;
        }
      }
      continue;
    }

    for (const auto& dir : dirs) {
      SE2State nearby(current.x + radius * dir.x(),
                      current.y + radius * dir.y(),
                      desired_yaw);
      SE2State assigned = nearby;
      if (!safeYaw(assigned, desired_yaw)) {
        continue;
      }

      const double local_clearance = std::min(
          footprintClearance(assigned),
          std::min(transitionClearance(prev, assigned),
                   transitionClearance(assigned, next)));
      const double yaw_penalty =
          std::abs(normalizeAngle(assigned.yaw - desired_yaw));
      const double score = local_clearance - 0.12 * radius - 0.05 * yaw_penalty;
      if (!found || score > best_score) {
        found = true;
        best_score = score;
        best = assigned;
      }
      if (local_clearance >= required) {
        repaired = assigned;
        return true;
      }
    }
  }

  if (!found) {
    return false;
  }

  repaired = best;
  return best_score > -kInf;
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
  const int max_depth = 4;
  if (seed.size() > 8) return false;

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
  for (size_t i = 1; i < right_repaired.size(); ++i) {
    if (transitionSafe(repaired.back(), right_repaired[i])) {
      repaired.push_back(right_repaired[i]);
      continue;
    }

    std::vector<SE2State> bridge;
    if (!bridgeUnsafeTransition(repaired.back(), right_repaired[i], bridge)) {
      return false;
    }
    repaired.insert(repaired.end(), bridge.begin() + 1, bridge.end());
  }
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

  std::vector<SE2State> expanded;
  expanded.reserve(repaired.size() + 2);
  expanded.push_back(repaired.front());
  for (size_t i = 1; i < repaired.size(); ++i) {
    if (transitionSafe(expanded.back(), repaired[i])) {
      expanded.push_back(repaired[i]);
      continue;
    }

    std::vector<SE2State> bridge;
    if (!bridgeUnsafeTransition(expanded.back(), repaired[i], bridge)) {
      return false;
    }
    expanded.insert(expanded.end(), bridge.begin() + 1, bridge.end());
  }

  repaired = expanded;
  return true;
}

bool SE2SequenceGenerator::tryRepairCombinedWindow(
    const std::vector<SE2State>& seed,
    std::vector<SE2State>& repaired) {
  if (seed.size() < 2) return false;
  if (seed.size() > 8) return false;
  if (segAdjustRecursive(seed, 0, repaired)) {
    return true;
  }
  return repairShortWindow(seed, repaired);
}

std::vector<SE2State> SE2SequenceGenerator::appendStates(
    const std::vector<SE2State>& lhs,
    const std::vector<SE2State>& rhs) const {
  if (lhs.empty()) return rhs;
  if (rhs.empty()) return lhs;

  std::vector<SE2State> out = lhs;
  const bool same_endpoint =
      (lhs.back().position() - rhs.front().position()).norm() < 1e-6 &&
      std::abs(normalizeAngle(lhs.back().yaw - rhs.front().yaw)) < 1e-6;
  out.insert(out.end(), rhs.begin() + (same_endpoint ? 1 : 0), rhs.end());
  return out;
}

std::vector<MotionSegment> SE2SequenceGenerator::compactSegments(
    const std::vector<MotionSegment>& segments) {
  std::vector<MotionSegment> merged;
  merged.reserve(segments.size());
  for (const auto& seg : segments) {
    if (seg.waypoints.size() < 2) {
      continue;
    }
    if (!merged.empty() &&
        merged.back().risk == RiskLevel::HIGH &&
        seg.risk == RiskLevel::HIGH) {
      std::vector<SE2State> combined =
          appendStates(merged.back().waypoints, seg.waypoints);
      if (combined.size() <= 12) {
        merged.back().waypoints = std::move(combined);
        continue;
      }
    }
    if (!merged.empty() &&
        merged.back().risk == seg.risk &&
        seg.risk == RiskLevel::LOW) {
      std::vector<SE2State> combined =
          appendStates(merged.back().waypoints, seg.waypoints);
      if (conservativeTrajectoryClearance(combined) + 1e-6 >=
          requiredClearance()) {
        merged.back().waypoints = std::move(combined);
      } else {
        merged.push_back(seg);
      }
    } else {
      merged.push_back(seg);
    }
  }

  bool changed = true;
  while (changed) {
    changed = false;

    for (size_t i = 0; i + 2 < merged.size(); ++i) {
      if (merged[i].risk != RiskLevel::HIGH ||
          merged[i + 1].risk != RiskLevel::LOW ||
          merged[i + 2].risk != RiskLevel::HIGH ||
          merged[i].waypoints.size() > 4 ||
          merged[i + 2].waypoints.size() > 4 ||
          merged[i + 1].waypoints.size() > 3) {
        continue;
      }

      std::vector<SE2State> combined =
          appendStates(merged[i].waypoints, merged[i + 1].waypoints);
      combined = appendStates(combined, merged[i + 2].waypoints);

      std::vector<SE2State> repaired;
      if (tryRepairCombinedWindow(combined, repaired) &&
          conservativeTrajectoryClearance(repaired) >= requiredClearance()) {
        MotionSegment replacement;
        replacement.risk = RiskLevel::LOW;
        replacement.waypoints = std::move(repaired);
        merged.erase(merged.begin() + static_cast<long>(i),
                     merged.begin() + static_cast<long>(i + 3));
        merged.insert(merged.begin() + static_cast<long>(i), std::move(replacement));
        changed = true;
        break;
      }
      if (combined.size() <= 12) {
        MotionSegment replacement;
        replacement.risk = RiskLevel::HIGH;
        replacement.waypoints = std::move(combined);
        merged.erase(merged.begin() + static_cast<long>(i),
                     merged.begin() + static_cast<long>(i + 3));
        merged.insert(merged.begin() + static_cast<long>(i), std::move(replacement));
        changed = true;
        break;
      }
    }

    if (changed) {
      continue;
    }

    for (size_t i = 0; i + 1 < merged.size(); ++i) {
      if (merged[i].risk != RiskLevel::HIGH ||
          merged[i + 1].risk != RiskLevel::HIGH ||
          merged[i].waypoints.size() > 4 ||
          merged[i + 1].waypoints.size() > 4) {
        continue;
      }

      std::vector<SE2State> combined =
          appendStates(merged[i].waypoints, merged[i + 1].waypoints);
      std::vector<SE2State> repaired;
      if (!tryRepairCombinedWindow(combined, repaired) ||
          conservativeTrajectoryClearance(repaired) < requiredClearance()) {
        continue;
      }

      MotionSegment replacement;
      replacement.risk = RiskLevel::LOW;
      replacement.waypoints = std::move(repaired);
      merged.erase(merged.begin() + static_cast<long>(i),
                   merged.begin() + static_cast<long>(i + 2));
      merged.insert(merged.begin() + static_cast<long>(i), std::move(replacement));
      changed = true;
      break;
    }
  }

  return merged;
}

std::vector<MotionSegment> SE2SequenceGenerator::expandTightHighSegments(
    const std::vector<MotionSegment>& segments) {
  if (segments.empty()) return {};

  std::vector<MotionSegment> expanded = segments;
  const double required = requiredClearance();

  bool changed = true;
  while (changed) {
    changed = false;
    for (size_t i = 0; i < expanded.size(); ++i) {
      if (expanded[i].risk != RiskLevel::HIGH) continue;
      if (expanded[i].waypoints.size() < 2 || expanded[i].waypoints.size() > 6) continue;

      const double current_clearance =
          conservativeTrajectoryClearance(expanded[i].waypoints);
      if (current_clearance + 1e-6 >= required) {
        continue;
      }

      const bool has_left =
          i > 0 && expanded[i - 1].risk == RiskLevel::LOW &&
          expanded[i - 1].waypoints.size() >= 3;
      const bool has_right =
          i + 1 < expanded.size() && expanded[i + 1].risk == RiskLevel::LOW &&
          expanded[i + 1].waypoints.size() >= 3;
      if (!has_left && !has_right) {
        continue;
      }

      std::vector<SE2State> best_high = expanded[i].waypoints;
      int best_left_take = 0;
      int best_right_take = 0;
      double best_score = current_clearance;

      const int max_left_take =
          has_left ? std::min<int>(2, expanded[i - 1].waypoints.size() - 2) : 0;
      const int max_right_take =
          has_right ? std::min<int>(2, expanded[i + 1].waypoints.size() - 2) : 0;

      for (int left_take = 0; left_take <= max_left_take; ++left_take) {
        for (int right_take = 0; right_take <= max_right_take; ++right_take) {
          if (left_take == 0 && right_take == 0) continue;

          std::vector<SE2State> candidate = expanded[i].waypoints;
          if (left_take > 0) {
            const auto& left = expanded[i - 1].waypoints;
            std::vector<SE2State> suffix(
                left.end() - static_cast<long>(left_take + 1), left.end());
            candidate = appendStates(suffix, candidate);
          }
          if (right_take > 0) {
            const auto& right = expanded[i + 1].waypoints;
            std::vector<SE2State> prefix(
                right.begin(), right.begin() + static_cast<long>(right_take + 1));
            candidate = appendStates(candidate, prefix);
          }

          if (candidate.size() > 12) {
            continue;
          }

          const double candidate_clearance =
              conservativeTrajectoryClearance(candidate);
          const bool better =
              candidate_clearance > best_score + 1e-6 ||
              (std::abs(candidate_clearance - best_score) <= 1e-6 &&
               (left_take + right_take) > (best_left_take + best_right_take));
          if (!better) {
            continue;
          }

          best_score = candidate_clearance;
          best_high = std::move(candidate);
          best_left_take = left_take;
          best_right_take = right_take;
        }
      }

      if (best_left_take == 0 && best_right_take == 0) {
        continue;
      }

      expanded[i].waypoints = std::move(best_high);
      if (best_left_take > 0) {
        auto& left = expanded[i - 1].waypoints;
        left.erase(left.end() - best_left_take, left.end());
      }
      if (best_right_take > 0) {
        auto& right = expanded[i + 1].waypoints;
        right.erase(right.begin(),
                    right.begin() + static_cast<long>(best_right_take));
      }

      std::vector<MotionSegment> compacted;
      compacted.reserve(expanded.size());
      for (auto& seg : expanded) {
        if (seg.waypoints.size() >= 2) {
          compacted.push_back(seg);
        }
      }
      expanded.swap(compacted);
      changed = true;
      break;
    }
  }

  return compactSegments(expanded);
}

double SE2SequenceGenerator::conservativeTrajectoryClearance(
    const std::vector<SE2State>& waypoints) const {
  if (!feasibility_) return -kInf;
  return feasibility_->segmentClearance(waypoints);
}

bool SE2SequenceGenerator::findMarginViolationWindow(
    const std::vector<SE2State>& waypoints,
    size_t& begin_idx,
    size_t& end_idx) const {
  if (waypoints.size() < 2) return false;

  const double required = requiredClearance();
  auto finePairClearance = [&](size_t idx) {
    std::vector<SE2State> pair = {waypoints[idx], waypoints[idx + 1]};
    return conservativeTrajectoryClearance(pair);
  };
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (footprintClearance(waypoints[i]) < required) {
      begin_idx = (i > 0) ? i - 1 : 0;
      end_idx = std::min(waypoints.size() - 1, i + 1);
      return true;
    }
  }

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    if (finePairClearance(i) < required) {
      begin_idx = i;
      end_idx = std::min(waypoints.size() - 1, i + 2);
      return true;
    }
  }

  return false;
}

std::vector<MotionSegment> SE2SequenceGenerator::enforceLowSegmentMargin(
    const std::vector<MotionSegment>& segments) {
  std::vector<MotionSegment> refined;
  refined.reserve(segments.size() + 8);

  for (const auto& seg : segments) {
    if (seg.waypoints.size() < 2 || seg.risk != RiskLevel::LOW) {
      refined.push_back(seg);
      continue;
    }

    std::vector<SE2State> remaining = seg.waypoints;
    while (remaining.size() >= 2) {
      size_t begin_idx = 0;
      size_t end_idx = 0;
      if (!findMarginViolationWindow(remaining, begin_idx, end_idx)) {
        refined.push_back(MotionSegment{remaining, RiskLevel::LOW});
        break;
      }

      if (begin_idx > 0) {
        MotionSegment prefix;
        prefix.risk = RiskLevel::LOW;
        prefix.waypoints.assign(remaining.begin(),
                                remaining.begin() + static_cast<long>(begin_idx + 1));
        if (prefix.waypoints.size() >= 2) {
          refined.push_back(std::move(prefix));
        }
      }

      MotionSegment middle;
      middle.waypoints.assign(remaining.begin() + static_cast<long>(begin_idx),
                              remaining.begin() + static_cast<long>(end_idx + 1));
      std::vector<SE2State> repaired;
      if (tryRepairCombinedWindow(middle.waypoints, repaired) &&
          conservativeTrajectoryClearance(repaired) >= requiredClearance()) {
        middle.risk = RiskLevel::LOW;
        middle.waypoints = std::move(repaired);
      } else {
        middle.risk = RiskLevel::HIGH;
      }
      if (middle.waypoints.size() >= 2) {
        refined.push_back(std::move(middle));
      }

      if (end_idx + 1 >= remaining.size()) {
        break;
      }

      remaining.assign(remaining.begin() + static_cast<long>(end_idx), remaining.end());
    }
  }

  return compactSegments(refined);
}

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  if (path.waypoints.size() < 2) return {};

  auto states = discretizePath(path, start, goal);
  if (states.size() < 2) return {};

  std::vector<MotionSegment> segments;
  MotionSegment current;
  current.risk = RiskLevel::LOW;
  current.waypoints.push_back(states.front());

  const size_t repair_horizon = 4;
  size_t i = 1;
  while (i + 1 < states.size()) {
    SE2State assigned = states[i];
    const bool yaw_ok = safeYaw(assigned, states[i].yaw);
    const bool edge_ok = yaw_ok && transitionSafe(current.waypoints.back(), assigned);
    if (yaw_ok && edge_ok) {
      current.waypoints.push_back(assigned);
      ++i;
      continue;
    }

    if (i + 1 < states.size()) {
      SE2State repaired_mid = states[i];
      SE2State next_assigned = states[i + 1];
      if (repairStateBetweenNeighbors(current.waypoints.back(), repaired_mid,
                                      next_assigned, states[i].yaw, repaired_mid) &&
          safeYaw(next_assigned, states[i + 1].yaw) &&
          transitionSafe(current.waypoints.back(), repaired_mid) &&
          transitionSafe(repaired_mid, next_assigned)) {
        current.waypoints.push_back(repaired_mid);
        current.waypoints.push_back(next_assigned);
        i += 2;
        continue;
      }
    }

    const size_t search_end =
        std::min(states.size() - 1, i + repair_horizon);
    size_t chosen_anchor_idx = std::min(states.size() - 1, i + 1);
    SE2State chosen_anchor = states[chosen_anchor_idx];
    bool found_safe_anchor = false;
    bool repaired_success = false;
    size_t repaired_anchor_idx = chosen_anchor_idx;

    for (size_t j = i + 1; j <= search_end; ++j) {
      SE2State candidate_anchor = states[j];
      if (j != states.size() - 1 && !safeYaw(candidate_anchor, states[j].yaw)) {
        continue;
      }

      chosen_anchor_idx = j;
      chosen_anchor = candidate_anchor;
      found_safe_anchor = true;

      std::vector<SE2State> repair_seed;
      repair_seed.reserve(j - i + 2);
      repair_seed.push_back(current.waypoints.back());
      for (size_t k = i; k < j; ++k) {
        repair_seed.push_back(states[k]);
      }
      repair_seed.push_back(candidate_anchor);

      std::vector<SE2State> repaired;
      if (segAdjustRecursive(repair_seed, 0, repaired) &&
          conservativeTrajectoryClearance(repaired) >= requiredClearance()) {
        current.waypoints = appendStates(current.waypoints, repaired);
        repaired_success = true;
        repaired_anchor_idx = j;
        break;
      }

      if (repairShortWindow(repair_seed, repaired) &&
          conservativeTrajectoryClearance(repaired) >= requiredClearance()) {
        current.waypoints = appendStates(current.waypoints, repaired);
        repaired_success = true;
        repaired_anchor_idx = j;
        break;
      }
    }

    if (repaired_success) {
      i = repaired_anchor_idx + 1;
      continue;
    }

    if (current.waypoints.size() > 1) {
      segments.push_back(current);
    }

    size_t high_anchor_idx = chosen_anchor_idx;
    SE2State high_anchor = chosen_anchor;
    if (!found_safe_anchor) {
      high_anchor_idx = std::min(states.size() - 1, i + 2);
      high_anchor = states[high_anchor_idx];
      safeYaw(high_anchor, states[high_anchor_idx].yaw);
    }

    MotionSegment high;
    high.risk = RiskLevel::HIGH;
    high.waypoints.reserve(high_anchor_idx - i + 2);
    high.waypoints.push_back(current.waypoints.back());
    for (size_t k = i; k < high_anchor_idx; ++k) {
      high.waypoints.push_back(states[k]);
    }
    high.waypoints.push_back(high_anchor);
    for (int pass = 0; pass < 2; ++pass) {
      for (size_t k = 1; k + 1 < high.waypoints.size(); ++k) {
        SE2State candidate = high.waypoints[k];
        const double desired_yaw = high.waypoints[k].yaw;
        bool ok = safeYaw(candidate, desired_yaw);
        if (ok) {
          ok = transitionSafe(high.waypoints[k - 1], candidate) &&
               transitionSafe(candidate, high.waypoints[k + 1]);
        }
        if (!ok) {
          candidate = high.waypoints[k];
          if (repairStateBetweenNeighbors(high.waypoints[k - 1], candidate,
                                          high.waypoints[k + 1], desired_yaw,
                                          candidate) ||
              pushStateFromObstacle(candidate, desired_yaw)) {
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
    current.waypoints.push_back(high.waypoints.back());
    i = high_anchor_idx + 1;
  }

  if (current.waypoints.empty()) {
    current.waypoints.push_back(states.front());
  }

  const auto sameState = [](const SE2State& a, const SE2State& b) {
    return (a.position() - b.position()).norm() < 1e-6 &&
           std::abs(normalizeAngle(a.yaw - b.yaw)) < 1e-6;
  };

  const SE2State final_state = states.back();
  if (sameState(current.waypoints.back(), final_state)) {
    current.waypoints.back() = final_state;
  } else if (transitionSafe(current.waypoints.back(), final_state)) {
    current.waypoints.push_back(final_state);
  } else {
    const size_t tail_count = std::min<size_t>(current.waypoints.size(), 8);
    std::vector<SE2State> seed(current.waypoints.end() - tail_count,
                               current.waypoints.end());
    if (sameState(seed.back(), final_state)) {
      seed.back() = final_state;
    } else {
      seed.push_back(final_state);
    }

    std::vector<SE2State> repaired;
    if (tryRepairCombinedWindow(seed, repaired) &&
        conservativeTrajectoryClearance(repaired) >= requiredClearance()) {
      current.waypoints.resize(current.waypoints.size() - tail_count);
      if (current.waypoints.empty()) {
        current.waypoints = repaired;
      } else {
        current.waypoints = appendStates(current.waypoints, repaired);
      }
    } else {
      MotionSegment prefix = current;
      prefix.waypoints.resize(current.waypoints.size() - tail_count);
      if (prefix.waypoints.size() >= 2) {
        segments.push_back(prefix);
      }

      MotionSegment high;
      high.risk = RiskLevel::HIGH;
      high.waypoints = seed;
      segments.push_back(high);
      current = MotionSegment();
    }
  }

  if (current.waypoints.size() >= 2) {
    segments.push_back(current);
  }

  return expandTightHighSegments(enforceLowSegmentMargin(compactSegments(segments)));
}

}  // namespace esv_planner
