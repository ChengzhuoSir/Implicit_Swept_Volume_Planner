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

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  std::vector<MotionSegment> segments;
  if (path.points.size() < 2) return segments;

  // Discretize path into SE2 states
  std::vector<SE2State> states;
  states.push_back(start);

  for (size_t i = 1; i < path.points.size(); ++i) {
    Eigen::Vector2d dir = path.points[i] - path.points[i - 1];
    double seg_len = dir.norm();
    if (seg_len < 1e-9) continue;

    int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / disc_step_)));
    for (int s = 1; s <= n_steps; ++s) {
      double t = static_cast<double>(s) / n_steps;
      Eigen::Vector2d p = path.points[i - 1] + t * dir;
      SE2State st(p.x(), p.y(), 0.0);
      // Assign yaw
      assignYaw(st);
      states.push_back(st);
    }
  }

  // Override last state with goal
  if (!states.empty()) {
    states.back() = goal;
  }

  // Segment into high-risk and low-risk regions
  MotionSegment current_seg;
  bool prev_safe = checker_->isFree(states.front());

  for (size_t i = 0; i < states.size(); ++i) {
    bool safe = checker_->isFree(states[i]);

    if (i > 0 && safe != prev_safe) {
      // Risk level changed — finalize current segment
      current_seg.risk = prev_safe ? RiskLevel::LOW : RiskLevel::HIGH;
      if (!current_seg.waypoints.empty()) {
        segments.push_back(current_seg);
      }
      current_seg = MotionSegment();
      // Overlap: include the transition point in both segments
      current_seg.waypoints.push_back(states[i - 1]);
    }

    current_seg.waypoints.push_back(states[i]);
    prev_safe = safe;
  }

  // Finalize last segment
  current_seg.risk = prev_safe ? RiskLevel::LOW : RiskLevel::HIGH;
  if (!current_seg.waypoints.empty()) {
    segments.push_back(current_seg);
  }

  // SegAdjust for high-risk segments
  for (auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      segAdjust(seg.waypoints);
    }
  }

  return segments;
}

bool SE2SequenceGenerator::segAdjust(std::vector<SE2State>& segment) {
  // Push colliding states away from obstacles using ESDF gradient
  bool all_safe = true;

  for (int attempt = 0; attempt < max_push_; ++attempt) {
    all_safe = true;
    for (auto& st : segment) {
      if (!checker_->isFree(st)) {
        all_safe = false;
        // Push away from obstacle using ESDF gradient (finite difference)
        double esdf_val = map_->getEsdf(st.x, st.y);
        double eps = map_->resolution();
        double dx = map_->getEsdf(st.x + eps, st.y) - map_->getEsdf(st.x - eps, st.y);
        double dy = map_->getEsdf(st.x, st.y + eps) - map_->getEsdf(st.x, st.y - eps);
        double grad_norm = std::sqrt(dx * dx + dy * dy);
        if (grad_norm > 1e-9) {
          double push = std::max(eps, -esdf_val);
          st.x += push * dx / grad_norm;
          st.y += push * dy / grad_norm;
        }
        // Re-assign yaw
        assignYaw(st);
      }
    }
    if (all_safe) break;
  }

  return all_safe;
}

bool SE2SequenceGenerator::assignYaw(SE2State& state) {
  auto safe_bins = checker_->safeYawIndices(state.x, state.y);
  if (safe_bins.empty()) return false;

  // Pick the yaw bin closest to the current heading direction
  int best_bin = safe_bins[0];
  double best_diff = kInf;
  for (int bin : safe_bins) {
    double yaw = checker_->yawFromBin(bin);
    double diff = std::abs(normalizeAngle(yaw - state.yaw));
    if (diff < best_diff) {
      best_diff = diff;
      best_bin = bin;
    }
  }
  state.yaw = checker_->yawFromBin(best_bin);
  return true;
}

}  // namespace esv_planner
