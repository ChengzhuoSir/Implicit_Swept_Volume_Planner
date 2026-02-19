#include "esv_planner/se2_sequence_generator.h"
#include <cmath>
#include <algorithm>

namespace esv_planner {

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
  if (path.points.size() < 2) return states;

  states.push_back(start);

  for (size_t i = 1; i < path.points.size(); ++i) {
    Eigen::Vector2d dir = path.points[i] - path.points[i - 1];
    double seg_len = dir.norm();
    if (seg_len < 1e-9) continue;

    // Tangent yaw for this segment
    double tangent_yaw = std::atan2(dir.y(), dir.x());

    int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / disc_step_)));
    for (int s = 1; s <= n_steps; ++s) {
      double t = static_cast<double>(s) / n_steps;
      Eigen::Vector2d p = path.points[i - 1] + t * dir;

      // Initialize yaw from tangent direction
      SE2State st(p.x(), p.y(), tangent_yaw);

      // For last point of last segment, blend toward goal yaw
      if (i == path.points.size() - 1 && s == n_steps) {
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

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  // Check if desired yaw is already safe
  int desired_bin = checker_->binFromYaw(desired_yaw);
  if (checker_->isYawSafe(state.x, state.y, desired_bin)) {
    state.yaw = checker_->yawFromBin(desired_bin);
    return true;
  }

  // Find closest safe yaw bin
  auto safe_bins = checker_->safeYawIndices(state.x, state.y);
  if (safe_bins.empty()) return false;

  int best_bin = safe_bins[0];
  double best_diff = kInf;
  for (int bin : safe_bins) {
    double yaw = checker_->yawFromBin(bin);
    double diff = std::abs(normalizeAngle(yaw - desired_yaw));
    if (diff < best_diff) {
      best_diff = diff;
      best_bin = bin;
    }
  }
  state.yaw = checker_->yawFromBin(best_bin);
  return true;
}

bool SE2SequenceGenerator::segAdjust(std::vector<SE2State>& segment) {
  // SegAdjust (paper Section IV-B):
  // Push colliding states away from obstacles using ESDF gradient,
  // then re-assign yaw. If still colliding, subdivide the segment.

  for (int attempt = 0; attempt < max_push_; ++attempt) {
    bool all_safe = true;

    for (auto& st : segment) {
      if (checker_->isFree(st)) continue;
      all_safe = false;

      // Push position away from obstacle using ESDF gradient
      double esdf_val = map_->getEsdf(st.x, st.y);
      double eps = map_->resolution();

      double dx = map_->getEsdf(st.x + eps, st.y) - map_->getEsdf(st.x - eps, st.y);
      double dy = map_->getEsdf(st.x, st.y + eps) - map_->getEsdf(st.x, st.y - eps);
      double grad_norm = std::sqrt(dx * dx + dy * dy);

      if (grad_norm > 1e-9) {
        double push_dist = std::max(eps, std::abs(esdf_val) + eps);
        st.x += push_dist * dx / grad_norm;
        st.y += push_dist * dy / grad_norm;
      }

      // Re-assign yaw after position change
      safeYaw(st, st.yaw);
    }

    if (all_safe) return true;
  }

  // If still not all safe, try subdividing: insert midpoints between colliding pairs
  std::vector<SE2State> refined;
  refined.push_back(segment.front());

  for (size_t i = 1; i < segment.size(); ++i) {
    // Check if midpoint helps
    SE2State mid;
    mid.x = 0.5 * (segment[i - 1].x + segment[i].x);
    mid.y = 0.5 * (segment[i - 1].y + segment[i].y);
    mid.yaw = segment[i - 1].yaw;  // initial guess

    // Push midpoint if needed
    double esdf_val = map_->getEsdf(mid.x, mid.y);
    if (esdf_val < map_->resolution()) {
      double eps = map_->resolution();
      double dx = map_->getEsdf(mid.x + eps, mid.y) - map_->getEsdf(mid.x - eps, mid.y);
      double dy = map_->getEsdf(mid.x, mid.y + eps) - map_->getEsdf(mid.x, mid.y - eps);
      double grad_norm = std::sqrt(dx * dx + dy * dy);
      if (grad_norm > 1e-9) {
        double push_dist = std::max(eps, std::abs(esdf_val) + eps);
        mid.x += push_dist * dx / grad_norm;
        mid.y += push_dist * dy / grad_norm;
      }
    }

    safeYaw(mid, mid.yaw);

    if (!checker_->isFree(segment[i - 1]) || !checker_->isFree(segment[i])) {
      refined.push_back(mid);
    }
    refined.push_back(segment[i]);
  }

  segment = refined;

  // One more round of push
  for (auto& st : segment) {
    if (!checker_->isFree(st)) {
      double eps = map_->resolution();
      double dx = map_->getEsdf(st.x + eps, st.y) - map_->getEsdf(st.x - eps, st.y);
      double dy = map_->getEsdf(st.x, st.y + eps) - map_->getEsdf(st.x, st.y - eps);
      double grad_norm = std::sqrt(dx * dx + dy * dy);
      if (grad_norm > 1e-9) {
        double push_dist = std::max(eps, std::abs(map_->getEsdf(st.x, st.y)) + eps);
        st.x += push_dist * dx / grad_norm;
        st.y += push_dist * dy / grad_norm;
      }
      safeYaw(st, st.yaw);
    }
  }

  return true;
}

RiskLevel SE2SequenceGenerator::classifyRisk(const std::vector<SE2State>& segment) {
  // A segment is HIGH risk if any state has limited safe yaw options
  // (i.e., the robot's orientation is constrained by nearby obstacles)
  int num_yaw_bins = checker_->numYawBins();
  int constrained_count = 0;

  for (const auto& st : segment) {
    auto safe = checker_->safeYawIndices(st.x, st.y);
    // If fewer than half the yaw bins are safe, it's constrained
    if (static_cast<int>(safe.size()) < num_yaw_bins / 2) {
      constrained_count++;
    }
  }

  // If more than 30% of states are constrained, mark as HIGH risk
  double ratio = static_cast<double>(constrained_count) / segment.size();
  return (ratio > 0.3) ? RiskLevel::HIGH : RiskLevel::LOW;
}

std::vector<MotionSegment> SE2SequenceGenerator::segmentByRisk(
    const std::vector<SE2State>& states) {
  std::vector<MotionSegment> segments;
  if (states.empty()) return segments;

  // Classify each state
  int num_yaw_bins = checker_->numYawBins();
  std::vector<bool> is_constrained(states.size(), false);

  for (size_t i = 0; i < states.size(); ++i) {
    auto safe = checker_->safeYawIndices(states[i].x, states[i].y);
    is_constrained[i] = (static_cast<int>(safe.size()) < num_yaw_bins / 2);
  }

  // Group contiguous states with same risk level
  MotionSegment current;
  bool prev_constrained = is_constrained[0];
  current.waypoints.push_back(states[0]);

  for (size_t i = 1; i < states.size(); ++i) {
    if (is_constrained[i] != prev_constrained) {
      // Finalize current segment
      current.risk = prev_constrained ? RiskLevel::HIGH : RiskLevel::LOW;
      segments.push_back(current);

      // Start new segment with overlap point
      current = MotionSegment();
      current.waypoints.push_back(states[i - 1]);
    }
    current.waypoints.push_back(states[i]);
    prev_constrained = is_constrained[i];
  }

  // Finalize last segment
  current.risk = prev_constrained ? RiskLevel::HIGH : RiskLevel::LOW;
  if (!current.waypoints.empty()) {
    segments.push_back(current);
  }

  // Merge very short segments (< 3 waypoints) into neighbors
  std::vector<MotionSegment> merged;
  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].waypoints.size() < 3 && !merged.empty()) {
      // Append to previous segment
      for (size_t j = 1; j < segments[i].waypoints.size(); ++j) {
        merged.back().waypoints.push_back(segments[i].waypoints[j]);
      }
      // Upgrade risk if needed
      if (segments[i].risk == RiskLevel::HIGH) {
        merged.back().risk = RiskLevel::HIGH;
      }
    } else {
      merged.push_back(segments[i]);
    }
  }

  return merged;
}

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  if (path.points.size() < 2) return {};

  // Step 1: Discretize path into SE2 states with tangent yaw
  auto states = discretizePath(path, start, goal);
  if (states.size() < 2) return {};

  // Step 2: Assign safe yaw to each state (SafeYaw)
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    safeYaw(states[i], states[i].yaw);
  }

  // Step 3: Segment by risk level
  auto segments = segmentByRisk(states);

  // Step 4: SegAdjust for high-risk segments
  for (auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      segAdjust(seg.waypoints);
    }
  }

  return segments;
}

}  // namespace esv_planner
