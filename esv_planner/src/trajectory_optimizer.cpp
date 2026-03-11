#include "esv_planner/trajectory_optimizer.h"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>

namespace esv_planner {

namespace {

constexpr double kShapeSampleStep = 0.05;
constexpr double kMaxSegmentLengthRatio = 1.08;
constexpr int kMaxSegmentHeadingOscillations = 1;
constexpr double kMeaningfulTargetHitClearanceGain = 0.06;

double waypointChainClearance(const std::vector<SE2State>& waypoints,
                              const ContinuousCollisionEvaluator* svsdf) {
  if (!svsdf || waypoints.empty()) return kInf;
  double min_clearance = kInf;
  for (const auto& wp : waypoints) {
    min_clearance = std::min(min_clearance, svsdf->evaluate(wp));
  }
  return min_clearance;
}

double continuousWaypointChainClearance(const std::vector<SE2State>& waypoints,
                                        const ContinuousCollisionEvaluator* svsdf,
                                        double sample_step) {
  if (!svsdf || waypoints.empty()) return kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double len = (b.position() - a.position()).norm();
    const int n_steps = std::max(
        1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
    for (int s = 0; s <= n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      SE2State st;
      st.x = a.x + (b.x - a.x) * t;
      st.y = a.y + (b.y - a.y) * t;
      st.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
      min_clearance = std::min(min_clearance, svsdf->evaluate(st));
    }
  }
  return min_clearance;
}

double waypointChainLength(const std::vector<SE2State>& waypoints) {
  if (waypoints.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    len += (waypoints[i].position() - waypoints[i - 1].position()).norm();
  }
  return len;
}

std::vector<Eigen::Vector2d> sampleTrajectoryPositions(const Trajectory& traj,
                                                       double sample_step) {
  std::vector<Eigen::Vector2d> pts;
  if (traj.empty()) return pts;
  const double total = traj.totalDuration();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(total / std::max(sample_step, 1e-3))));
  pts.reserve(static_cast<size_t>(n_steps) + 1);
  for (int i = 0; i <= n_steps; ++i) {
    const double t = (i == n_steps) ? total : std::min(total, i * sample_step);
    pts.push_back(traj.sample(t).position());
  }
  return pts;
}

double sampledTrajectoryLength(const Trajectory& traj, double sample_step) {
  const auto pts = sampleTrajectoryPositions(traj, sample_step);
  if (pts.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += (pts[i] - pts[i - 1]).norm();
  }
  return len;
}

double pointSegmentDistance(const Eigen::Vector2d& p,
                            const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom < 1e-12) {
    return (p - a).norm();
  }
  const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom));
  return (p - (a + t * ab)).norm();
}

double maxLateralBulgeToWaypoints(const Trajectory& traj,
                                  const std::vector<SE2State>& waypoints,
                                  double sample_step) {
  if (traj.empty() || waypoints.size() < 2) return 0.0;

  const auto pts = sampleTrajectoryPositions(traj, sample_step);
  double max_bulge = 0.0;
  for (const auto& p : pts) {
    double best = kInf;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      best = std::min(
          best,
          pointSegmentDistance(p, waypoints[i - 1].position(), waypoints[i].position()));
    }
    max_bulge = std::max(max_bulge, best);
  }
  return max_bulge;
}

int headingOscillationCount(const Trajectory& traj, double sample_step) {
  const auto pts = sampleTrajectoryPositions(traj, sample_step);
  if (pts.size() < 4) return 0;

  std::vector<double> headings;
  headings.reserve(pts.size() - 1);
  for (size_t i = 1; i < pts.size(); ++i) {
    const Eigen::Vector2d d = pts[i] - pts[i - 1];
    if (d.norm() < 1e-4) continue;
    headings.push_back(std::atan2(d.y(), d.x()));
  }
  if (headings.size() < 3) return 0;

  constexpr double kTurnEps = 0.08;
  int oscillations = 0;
  int prev_sign = 0;
  for (size_t i = 1; i < headings.size(); ++i) {
    const double turn = normalizeAngle(headings[i] - headings[i - 1]);
    if (std::abs(turn) < kTurnEps) continue;
    const int sign = (turn > 0.0) ? 1 : -1;
    if (prev_sign != 0 && sign != prev_sign) {
      ++oscillations;
    }
    prev_sign = sign;
  }
  return oscillations;
}

double maxBoundaryVelocityJump(const Trajectory& traj) {
  if (traj.pos_pieces.size() < 2) return 0.0;
  double max_jump = 0.0;
  for (size_t i = 0; i + 1 < traj.pos_pieces.size(); ++i) {
    const Eigen::Vector2d left =
        traj.pos_pieces[i].velocity(traj.pos_pieces[i].duration);
    const Eigen::Vector2d right = traj.pos_pieces[i + 1].velocity(0.0);
    max_jump = std::max(max_jump, (left - right).norm());
  }
  return max_jump;
}

double maxBoundaryYawRateJump(const Trajectory& traj) {
  if (traj.yaw_pieces.size() < 2) return 0.0;
  double max_jump = 0.0;
  for (size_t i = 0; i + 1 < traj.yaw_pieces.size(); ++i) {
    const double left =
        traj.yaw_pieces[i].velocity(traj.yaw_pieces[i].duration);
    const double right = traj.yaw_pieces[i + 1].velocity(0.0);
    max_jump = std::max(max_jump, std::abs(left - right));
  }
  return max_jump;
}

bool trajectoryBoundaryContinuityAcceptable(const Trajectory& traj) {
  constexpr double kMaxVelocityJump = 0.15;
  constexpr double kMaxYawRateJump = 0.35;
  return maxBoundaryVelocityJump(traj) <= kMaxVelocityJump &&
         maxBoundaryYawRateJump(traj) <= kMaxYawRateJump;
}

bool trajectoryShapeAcceptable(const Trajectory& traj,
                               const std::vector<SE2State>& waypoints) {
  if (traj.empty()) return false;
  const double ref_len = waypointChainLength(waypoints);
  if (ref_len < 0.5) {
    return true;
  }

  const double traj_len = sampledTrajectoryLength(traj, kShapeSampleStep);
  const double length_ratio =
      (ref_len > 1e-9) ? (traj_len / ref_len) : kInf;
  if (length_ratio > kMaxSegmentLengthRatio) {
    return false;
  }

  if (headingOscillationCount(traj, kShapeSampleStep) >
      kMaxSegmentHeadingOscillations) {
    return false;
  }

  if (maxLateralBulgeToWaypoints(traj, waypoints, kShapeSampleStep) > 0.07) {
    return false;
  }

  return true;
}

bool trajectoryShapeSoftAcceptable(const Trajectory& traj,
                                   const std::vector<SE2State>& waypoints) {
  if (traj.empty()) return false;
  const double ref_len = waypointChainLength(waypoints);
  if (ref_len < 0.5) {
    return trajectoryBoundaryContinuityAcceptable(traj);
  }

  const double traj_len = sampledTrajectoryLength(traj, kShapeSampleStep);
  const double length_ratio =
      (ref_len > 1e-9) ? (traj_len / ref_len) : kInf;
  if (length_ratio > 1.20) {
    return false;
  }

  if (headingOscillationCount(traj, kShapeSampleStep) >
      kMaxSegmentHeadingOscillations + 2) {
    return false;
  }

  if (maxLateralBulgeToWaypoints(traj, waypoints, kShapeSampleStep) > 0.10) {
    return false;
  }

  return trajectoryBoundaryContinuityAcceptable(traj);
}

bool trajectoryCollisionFree(const Trajectory& traj,
                             const ContinuousCollisionEvaluator* svsdf,
                             const GridMap* map,
                             double* min_svsdf = nullptr) {
  if (traj.empty()) return false;
  if (!svsdf) {
    if (min_svsdf) *min_svsdf = kInf;
    return true;
  }

  auto denseSampledTrajectoryClearance =
      [&](const Trajectory& candidate) {
        if (!map || candidate.empty()) {
          return kInf;
        }
        constexpr double kMaxTimeStep = 0.02;
        const double max_linear_step =
            std::max(0.5 * map->resolution(), 0.02);
        constexpr double kMaxYawStep = 0.05;
        constexpr int kMaxSamplesPerPiece = 256;

        double min_clearance = kInf;
        double acc = 0.0;
        for (size_t i = 0; i < candidate.pos_pieces.size(); ++i) {
          const double piece_duration = candidate.pos_pieces[i].duration;
          const SE2State s0 = candidate.sample(acc);
          const SE2State s1 = candidate.sample(acc + piece_duration);
          const double linear_delta = (s1.position() - s0.position()).norm();
          const double yaw_delta =
              std::abs(normalizeAngle(s1.yaw - s0.yaw));
          int n_steps = std::max(
              1, static_cast<int>(std::ceil(piece_duration / kMaxTimeStep)));
          n_steps = std::max(
              n_steps,
              static_cast<int>(std::ceil(
                  linear_delta / std::max(max_linear_step, 1e-6))));
          n_steps = std::max(
              n_steps,
              static_cast<int>(std::ceil(
                  yaw_delta / std::max(kMaxYawStep, 1e-6))));
          n_steps = std::min(n_steps, kMaxSamplesPerPiece);

          const int start_step = (i == 0) ? 0 : 1;
          for (int s = start_step; s <= n_steps; ++s) {
            const double local_t =
                piece_duration * static_cast<double>(s) /
                static_cast<double>(n_steps);
            min_clearance = std::min(
                min_clearance, svsdf->evaluate(candidate.sample(acc + local_t)));
          }
          acc += piece_duration;
        }
        return min_clearance;
      };

  double clearance = svsdf->evaluateTrajectory(traj);
  clearance = std::min(clearance, denseSampledTrajectoryClearance(traj));
  if (min_svsdf) *min_svsdf = clearance;
  return clearance >= 0.0;
}

bool trajectoryMatchesEndpoints(const Trajectory& traj,
                                const SE2State& start,
                                const SE2State& goal) {
  if (traj.empty()) return false;
  const SE2State start_sample = traj.sample(0.0);
  const SE2State goal_sample = traj.sample(traj.totalDuration());
  constexpr double kPosTol = 1e-3;
  constexpr double kYawTol = 5e-3;
  return (start_sample.position() - start.position()).norm() <= kPosTol &&
         (goal_sample.position() - goal.position()).norm() <= kPosTol &&
         std::abs(normalizeAngle(start_sample.yaw - start.yaw)) <= kYawTol &&
         std::abs(normalizeAngle(goal_sample.yaw - goal.yaw)) <= kYawTol;
}

bool meetsClearanceTarget(double clearance, double target) {
  return clearance + 1e-6 >= target;
}

double transitionClearance(const SE2State& from,
                           const SE2State& to,
                           const ContinuousCollisionEvaluator* svsdf,
                           double sample_step);

std::vector<SE2State> compactSupportStatesForContinuousFit(
    const std::vector<SE2State>& support_states,
    const ContinuousCollisionEvaluator* svsdf,
    double min_spacing,
    double turn_keep_threshold,
    double clearance_keep_threshold) {
  if (support_states.size() <= 2) return support_states;

  std::vector<SE2State> compact;
  compact.reserve(support_states.size());
  compact.push_back(support_states.front());

  double accum = 0.0;
  for (size_t i = 1; i + 1 < support_states.size(); ++i) {
    accum += (support_states[i].position() - support_states[i - 1].position()).norm();

    const Eigen::Vector2d prev_dir =
        support_states[i].position() - support_states[i - 1].position();
    const Eigen::Vector2d next_dir =
        support_states[i + 1].position() - support_states[i].position();
    double turn = 0.0;
    if (prev_dir.norm() > 1e-4 && next_dir.norm() > 1e-4) {
      turn = std::abs(normalizeAngle(
          std::atan2(next_dir.y(), next_dir.x()) -
          std::atan2(prev_dir.y(), prev_dir.x())));
    }

    const double clearance =
        svsdf ? svsdf->evaluate(support_states[i]) : kInf;
    const bool near_obstacle = clearance <= clearance_keep_threshold;
    bool keep_for_obstacle = false;
    if (near_obstacle) {
      const double sample_step = 0.05;
      const double shortcut_clearance = transitionClearance(
          compact.back(), support_states[i + 1], svsdf, sample_step);
      keep_for_obstacle = shortcut_clearance + 1e-6 < clearance_keep_threshold;
    }
    if (accum >= min_spacing || turn >= turn_keep_threshold || keep_for_obstacle) {
      if ((support_states[i].position() - compact.back().position()).norm() > 1e-9 ||
          std::abs(normalizeAngle(support_states[i].yaw - compact.back().yaw)) > 1e-4) {
        compact.push_back(support_states[i]);
      }
      accum = 0.0;
    }
  }

  if ((support_states.back().position() - compact.back().position()).norm() > 1e-9 ||
      std::abs(normalizeAngle(support_states.back().yaw - compact.back().yaw)) > 1e-4) {
    compact.push_back(support_states.back());
  }
  return compact;
}

std::vector<SE2State> extractSupportStates(const std::vector<SE2State>& waypoints,
                                           double min_spacing) {
  if (waypoints.size() <= 2) return waypoints;

  std::vector<SE2State> support;
  support.reserve(waypoints.size());
  support.push_back(waypoints.front());
  double accum = 0.0;
  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    accum += (waypoints[i].position() - waypoints[i - 1].position()).norm();
    const double yaw_jump =
        std::abs(normalizeAngle(waypoints[i + 1].yaw - waypoints[i - 1].yaw));
    if (accum >= min_spacing || yaw_jump > 0.35) {
      support.push_back(waypoints[i]);
      accum = 0.0;
    }
  }
  if ((waypoints.back().position() - support.back().position()).norm() > 1e-9) {
    support.push_back(waypoints.back());
  }
  return support;
}

double transitionClearance(const SE2State& from,
                           const SE2State& to,
                           const ContinuousCollisionEvaluator* svsdf,
                           double sample_step) {
  if (!svsdf) return kInf;
  const double len = (to.position() - from.position()).norm();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = from.x + (to.x - from.x) * t;
    st.y = from.y + (to.y - from.y) * t;
    st.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

double constantYawTranslationClearance(const SE2State& from,
                                       const SE2State& to,
                                       double yaw,
                                       const ContinuousCollisionEvaluator* svsdf,
                                       double sample_step) {
  if (!svsdf) return kInf;
  const double len = (to.position() - from.position()).norm();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = from.x + (to.x - from.x) * t;
    st.y = from.y + (to.y - from.y) * t;
    st.yaw = yaw;
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

double inPlaceRotationClearance(const Eigen::Vector2d& pos,
                                double yaw0,
                                double yaw1,
                                const ContinuousCollisionEvaluator* svsdf,
                                double sample_step) {
  if (!svsdf) return kInf;
  const double yaw_delta = std::abs(normalizeAngle(yaw1 - yaw0));
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(yaw_delta / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = pos.x();
    st.y = pos.y();
    st.yaw = normalizeAngle(yaw0 + normalizeAngle(yaw1 - yaw0) * t);
    min_clearance = std::min(min_clearance, svsdf->evaluate(st));
  }
  return min_clearance;
}

void clampStateToMapBounds(SE2State& state, const GridMap* map) {
  if (!map || !map->ready() || map->width() <= 0 || map->height() <= 0) {
    return;
  }
  const double margin = 0.5 * map->resolution();
  const double x_min = map->originX() + margin;
  const double y_min = map->originY() + margin;
  const double x_max =
      map->originX() + static_cast<double>(map->width()) * map->resolution() - margin;
  const double y_max =
      map->originY() + static_cast<double>(map->height()) * map->resolution() - margin;
  if (x_min > x_max || y_min > y_max) {
    return;
  }
  state.x = std::max(x_min, std::min(x_max, state.x));
  state.y = std::max(y_min, std::min(y_max, state.y));
}

bool pushSupportStatesTowardClearance(std::vector<SE2State>& states,
                                      const ContinuousCollisionEvaluator* svsdf,
                                      const GridMap* map,
                                      double target_clearance,
                                      int max_passes) {
  if (!svsdf || !map || states.size() <= 2) return false;

  const double sample_step = 0.5 * map->resolution();
  bool any_changed = false;
  for (int pass = 0; pass < max_passes; ++pass) {
    bool changed = false;
    bool all_safe = true;
    for (size_t i = 1; i + 1 < states.size(); ++i) {
      clampStateToMapBounds(states[i], map);
      const double state_clearance = svsdf->evaluate(states[i]);
      double min_clearance = state_clearance;
      if (i > 0) {
        min_clearance = std::min(
            min_clearance,
            transitionClearance(states[i - 1], states[i], svsdf, sample_step));
      }
      if (i + 1 < states.size()) {
        min_clearance = std::min(
            min_clearance,
            transitionClearance(states[i], states[i + 1], svsdf, sample_step));
      }
      if (min_clearance >= target_clearance) {
        continue;
      }

      all_safe = false;
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;
      svsdf->gradient(states[i], grad_pos, grad_yaw);
      if (grad_pos.norm() < 1e-6) {
        if (i == 0 && states.size() > 1) {
          grad_pos = states[i].position() - states[i + 1].position();
        } else if (i + 1 == states.size() && states.size() > 1) {
          grad_pos = states[i].position() - states[i - 1].position();
        } else if (i > 0 && i + 1 < states.size()) {
          grad_pos = states[i].position() -
                     0.5 * (states[i - 1].position() + states[i + 1].position());
        }
      }
      if (grad_pos.norm() < 1e-6) {
        continue;
      }

      const double deficit = target_clearance - min_clearance;
      const double push = std::min(5.0 * map->resolution(),
                                   std::max(0.5 * map->resolution(),
                                            deficit + 0.25 * map->resolution()));
      const Eigen::Vector2d dir = grad_pos.normalized();
      SE2State best_state = states[i];
      double best_clearance = min_clearance;
      for (double sign : {1.0, -1.0}) {
        SE2State candidate = states[i];
        candidate.x += sign * push * dir.x();
        candidate.y += sign * push * dir.y();
        candidate.yaw = normalizeAngle(candidate.yaw + sign * 0.05 * grad_yaw);
        clampStateToMapBounds(candidate, map);

        double candidate_clearance = svsdf->evaluate(candidate);
        if (i > 0) {
          candidate_clearance = std::min(
              candidate_clearance,
              transitionClearance(states[i - 1], candidate, svsdf, sample_step));
        }
        if (i + 1 < states.size()) {
          candidate_clearance = std::min(
              candidate_clearance,
              transitionClearance(candidate, states[i + 1], svsdf, sample_step));
        }
        if (candidate_clearance > best_clearance + 1e-6) {
          best_clearance = candidate_clearance;
          best_state = candidate;
        }
      }
      if (best_clearance <= min_clearance + 1e-6) {
        continue;
      }
      states[i] = best_state;
      changed = true;
      any_changed = true;
    }

    if (all_safe || !changed) {
      break;
    }
  }
  return any_changed;
}

bool densifyMarginViolatingEdges(std::vector<SE2State>& states,
                                 const ContinuousCollisionEvaluator* svsdf,
                                 const GridMap* map,
                                 double target_clearance) {
  if (!svsdf || !map || states.size() < 2) return false;

  const double sample_step = 0.5 * map->resolution();
  std::vector<SE2State> expanded;
  expanded.reserve(states.size() * 2);
  expanded.push_back(states.front());
  bool changed = false;

  for (size_t i = 0; i + 1 < states.size(); ++i) {
    const SE2State& a = states[i];
    const SE2State& b = states[i + 1];
    const double edge_len = (b.position() - a.position()).norm();
    const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
    const int linear_steps =
        std::max(1, static_cast<int>(std::ceil(edge_len / sample_step)));
    const int yaw_steps =
        std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.1)));
    const int n_steps = std::max(linear_steps, yaw_steps);
    double edge_clearance = kInf;
    SE2State worst_state;
    for (int s = 0; s <= n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      SE2State sample;
      sample.x = a.x + (b.x - a.x) * t;
      sample.y = a.y + (b.y - a.y) * t;
      sample.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
      const double clearance = svsdf->evaluate(sample);
      if (clearance < edge_clearance) {
        edge_clearance = clearance;
        worst_state = sample;
      }
    }
    if (edge_clearance < target_clearance && edge_len > map->resolution()) {
      auto insertedWindowClearance = [&](const SE2State& candidate) {
        return std::min(
            svsdf->evaluate(candidate),
            std::min(transitionClearance(a, candidate, svsdf, sample_step),
                     transitionClearance(candidate, b, svsdf, sample_step)));
      };

      SE2State inserted = worst_state;
      double inserted_clearance = insertedWindowClearance(inserted);
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;
      svsdf->gradient(worst_state, grad_pos, grad_yaw);
      if (grad_pos.norm() < 1e-6) {
        grad_pos = worst_state.position() - 0.5 * (a.position() + b.position());
      }
      if (grad_pos.norm() > 1e-6) {
        const double deficit = target_clearance - edge_clearance;
        const double push = std::min(4.0 * map->resolution(),
                                     std::max(0.5 * map->resolution(),
                                              deficit + 0.5 * map->resolution()));
        const Eigen::Vector2d dir = grad_pos.normalized();
        for (double sign : {1.0, -1.0}) {
          SE2State candidate = worst_state;
          candidate.x += sign * push * dir.x();
          candidate.y += sign * push * dir.y();
          candidate.yaw = normalizeAngle(candidate.yaw + sign * 0.1 * grad_yaw);
          clampStateToMapBounds(candidate, map);
          const double candidate_clearance = insertedWindowClearance(candidate);
          if (candidate_clearance > inserted_clearance + 1e-6) {
            inserted = candidate;
            inserted_clearance = candidate_clearance;
          }
        }
      }
      expanded.push_back(inserted);
      changed = true;
    }
    expanded.push_back(b);
  }

  if (changed) {
    states.swap(expanded);
  }
  return changed;
}

double localWindowClearance(const std::vector<SE2State>& states,
                            size_t idx,
                            const SE2State& st,
                            const ContinuousCollisionEvaluator* svsdf,
                            const GridMap* map) {
  if (!svsdf || !map) return -kInf;
  const double sample_step = 0.5 * map->resolution();
  double clearance = svsdf->evaluate(st);
  if (idx > 0) {
    clearance = std::min(clearance,
                         transitionClearance(states[idx - 1], st, svsdf, sample_step));
  }
  if (idx + 1 < states.size()) {
    clearance = std::min(clearance,
                         transitionClearance(st, states[idx + 1], svsdf, sample_step));
  }
  return clearance;
}

struct SmallWindowQuality {
  double min_clearance = -kInf;
  double deficit_penalty = kInf;
};

SmallWindowQuality evaluateSmallWindowQuality(
    const std::vector<SE2State>& states,
    const ContinuousCollisionEvaluator* svsdf,
    const GridMap* map,
    double target_clearance) {
  SmallWindowQuality q;
  if (!svsdf || !map || states.empty()) return q;

  const double sample_step = std::max(map->resolution(), 0.05);
  q.min_clearance = kInf;
  q.deficit_penalty = 0.0;
  auto accumulate = [&](const SE2State& st) {
    const double clearance = svsdf->evaluate(st);
    q.min_clearance = std::min(q.min_clearance, clearance);
    if (clearance < target_clearance) {
      const double deficit = target_clearance - clearance;
      q.deficit_penalty += deficit * deficit;
    }
  };

  for (size_t i = 0; i < states.size(); ++i) {
    accumulate(states[i]);
    if (i + 1 >= states.size()) continue;
    const SE2State& a = states[i];
    const SE2State& b = states[i + 1];
    const double len = (b.position() - a.position()).norm();
    const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
    const int linear_steps =
        std::max(1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
    const int yaw_steps =
        std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.1)));
    const int n_steps = std::max(linear_steps, yaw_steps);
    for (int s = 1; s < n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      SE2State sample;
      sample.x = a.x + (b.x - a.x) * t;
      sample.y = a.y + (b.y - a.y) * t;
      sample.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
      accumulate(sample);
    }
  }
  return q;
}

std::vector<SE2State> reduceStatesByWorstClearance(
    const std::vector<SE2State>& states,
    const ContinuousCollisionEvaluator* svsdf,
    size_t max_keep) {
  if (!svsdf || states.size() <= max_keep || max_keep < 2) {
    return states;
  }

  std::vector<size_t> selected;
  selected.reserve(max_keep);
  selected.push_back(0);
  selected.push_back(states.size() - 1);

  struct Candidate {
    size_t idx = 0;
    double clearance = kInf;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(states.size() > 2 ? states.size() - 2 : 0);
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    candidates.push_back({i, svsdf->evaluate(states[i])});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) {
              if (std::abs(lhs.clearance - rhs.clearance) > 1e-9) {
                return lhs.clearance < rhs.clearance;
              }
              return lhs.idx < rhs.idx;
            });

  for (const auto& cand : candidates) {
    if (selected.size() >= max_keep) break;
    if (std::find(selected.begin(), selected.end(), cand.idx) == selected.end()) {
      selected.push_back(cand.idx);
    }
  }

  for (size_t i = 1; selected.size() < max_keep && i + 1 < states.size(); ++i) {
    if (std::find(selected.begin(), selected.end(), i) == selected.end()) {
      selected.push_back(i);
    }
  }

  std::sort(selected.begin(), selected.end());
  std::vector<SE2State> reduced;
  reduced.reserve(selected.size());
  for (size_t idx : selected) {
    reduced.push_back(states[idx]);
  }
  return reduced;
}

bool betterSmallWindowQuality(const SmallWindowQuality& lhs,
                              const SmallWindowQuality& rhs) {
  if (lhs.deficit_penalty + 1e-9 < rhs.deficit_penalty) return true;
  if (rhs.deficit_penalty + 1e-9 < lhs.deficit_penalty) return false;
  return lhs.min_clearance > rhs.min_clearance + 1e-6;
}

bool repairSmallSE2WindowGreedy(std::vector<SE2State>& states,
                                const ContinuousCollisionEvaluator* svsdf,
                                const GridMap* map,
                                double target_clearance) {
  if (!svsdf || !map || states.size() < 2 || states.size() > 6) return false;
  bool any_changed = false;
  for (int pass = 0; pass < 3; ++pass) {
    bool changed = false;
    SmallWindowQuality pass_quality =
        evaluateSmallWindowQuality(states, svsdf, map, target_clearance);
    if (pass_quality.min_clearance >= target_clearance) {
      return true;
    }
    for (size_t i = 1; i + 1 < states.size(); ++i) {
      SE2State best_state = states[i];
      SmallWindowQuality best_quality = pass_quality;
      SE2State seed_state = states[i];
      for (int round = 0; round < 3; ++round) {
        const double step_xy = 0.10 / static_cast<double>(1 << round);
        const double step_yaw = 0.24 / static_cast<double>(1 << round);
        bool round_improved = false;
        SE2State round_best = seed_state;
        for (int ix = -2; ix <= 2; ++ix) {
          for (int iy = -2; iy <= 2; ++iy) {
            for (int iyaw = -1; iyaw <= 1; ++iyaw) {
              SE2State trial = seed_state;
              trial.x += ix * step_xy;
              trial.y += iy * step_xy;
              trial.yaw = normalizeAngle(trial.yaw + iyaw * step_yaw);
              std::vector<SE2State> candidate = states;
              candidate[i] = trial;
              const SmallWindowQuality trial_quality =
                  evaluateSmallWindowQuality(candidate, svsdf, map, target_clearance);
              if (betterSmallWindowQuality(trial_quality, best_quality)) {
                best_quality = trial_quality;
                best_state = trial;
                round_best = trial;
                round_improved = true;
              }
              if (trial_quality.min_clearance >= target_clearance) {
                best_quality = trial_quality;
                best_state = trial;
                round_best = trial;
                round_improved = true;
              }
            }
          }
        }
        if (best_quality.min_clearance >= target_clearance) {
          break;
        }
        if (round_improved) {
          seed_state = round_best;
        }
      }

      if (betterSmallWindowQuality(best_quality, pass_quality)) {
        states[i] = best_state;
        changed = true;
        any_changed = true;
        pass_quality = best_quality;
      }
    }

    if (!changed) {
      break;
    }
  }

  return any_changed;
}

bool repairSmallSE2WindowBlockShift(std::vector<SE2State>& states,
                                    const ContinuousCollisionEvaluator* svsdf,
                                    const GridMap* map,
                                    double target_clearance) {
  if (!svsdf || !map || states.size() < 2 || states.size() > 12) return false;

  size_t first_bad = states.size();
  size_t last_bad = 0;
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    const double clearance = svsdf->evaluate(states[i]);
    if (clearance < target_clearance) {
      first_bad = std::min(first_bad, i);
      last_bad = i;
    }
  }
  if (first_bad == states.size()) {
    for (size_t i = 1; i + 1 < states.size(); ++i) {
      const double clearance = localWindowClearance(states, i, states[i], svsdf, map);
      if (clearance < target_clearance) {
        first_bad = std::min(first_bad, i);
        last_bad = i;
      }
    }
  }
  if (first_bad == states.size()) {
    return evaluateSmallWindowQuality(states, svsdf, map, target_clearance)
               .min_clearance >= target_clearance;
  }

  const double sample_step = 0.5 * map->resolution();
  const SmallWindowQuality initial_quality =
      evaluateSmallWindowQuality(states, svsdf, map, target_clearance);
  double best_clearance = initial_quality.min_clearance;
  std::vector<SE2State> best = states;

  auto tryPattern = [&](size_t shift_first,
                        size_t shift_last,
                        const std::vector<double>& weights,
                        double dx,
                        double dy,
                        double yaw_delta) {
    std::vector<SE2State> trial = states;
    for (size_t i = shift_first; i <= shift_last; ++i) {
      const double w = weights[i - shift_first];
      trial[i].x += w * dx;
      trial[i].y += w * dy;
      trial[i].yaw = normalizeAngle(trial[i].yaw + w * yaw_delta);
    }
    const SmallWindowQuality quality =
        evaluateSmallWindowQuality(trial, svsdf, map, target_clearance);
    if (quality.min_clearance > best_clearance + 1e-6) {
      best_clearance = quality.min_clearance;
      best = trial;
    }
    if (quality.min_clearance >= target_clearance) {
      states = trial;
      return true;
    }
    return false;
  };

  auto appendDirection = [](std::vector<Eigen::Vector2d>& dirs,
                            const Eigen::Vector2d& dir) {
    if (dir.norm() < 1e-9) return;
    const Eigen::Vector2d unit = dir.normalized();
    for (const auto& existing : dirs) {
      if (std::abs(existing.dot(unit)) > 0.985) return;
    }
    dirs.push_back(unit);
  };

  std::vector<std::pair<size_t, size_t>> windows;
  windows.emplace_back(first_bad, last_bad);
  if (first_bad + 1 <= last_bad) windows.emplace_back(first_bad + 1, last_bad);
  if (first_bad <= last_bad - 1) windows.emplace_back(first_bad, last_bad - 1);
  if (first_bad + 1 <= last_bad - 1) windows.emplace_back(first_bad + 1, last_bad - 1);
  const size_t ctx_first = (first_bad > 0) ? (first_bad - 1) : first_bad;
  const size_t ctx_last =
      std::min(last_bad + 1, states.size() - 1);
  if (ctx_first < first_bad || ctx_last > last_bad) {
    windows.emplace_back(ctx_first, last_bad);
    windows.emplace_back(first_bad, ctx_last);
    windows.emplace_back(ctx_first, ctx_last);
  }
  if (states.size() <= 6) {
    for (size_t l = ctx_first; l <= ctx_last; ++l) {
      for (size_t r = l; r <= ctx_last; ++r) {
        if (std::find(windows.begin(), windows.end(), std::make_pair(l, r)) == windows.end()) {
          windows.emplace_back(l, r);
        }
      }
    }
  }

  for (const auto& window : windows) {
    const size_t shift_first = std::max<size_t>(1, window.first);
    const size_t shift_last = std::min(window.second, states.size() - 2);
    if (shift_first > shift_last) {
      continue;
    }
    std::vector<double> uniform_weights(shift_last - shift_first + 1, 1.0);
    std::vector<double> tail_ramp_weights(shift_last - shift_first + 1, 1.0);
    std::vector<double> head_ramp_weights(shift_last - shift_first + 1, 1.0);
    if (uniform_weights.size() > 1) {
      for (size_t i = 0; i < uniform_weights.size(); ++i) {
        const double alpha =
            static_cast<double>(i) / static_cast<double>(uniform_weights.size() - 1);
        tail_ramp_weights[i] = 0.5 + 0.5 * alpha;
        head_ramp_weights[i] = 1.0 - 0.5 * alpha;
      }
    }

    Eigen::Vector2d window_chord = states[shift_last].position() - states[shift_first].position();
    Eigen::Vector2d anchor_dir = Eigen::Vector2d::Zero();
    if (shift_first > 0) {
      anchor_dir += states[shift_first].position() - states[shift_first - 1].position();
    }
    if (shift_last + 1 < states.size()) {
      anchor_dir += states[shift_last].position() - states[shift_last + 1].position();
    }

    std::vector<Eigen::Vector2d> dirs;
    appendDirection(dirs, Eigen::Vector2d::UnitX());
    appendDirection(dirs, -Eigen::Vector2d::UnitX());
    appendDirection(dirs, Eigen::Vector2d::UnitY());
    appendDirection(dirs, -Eigen::Vector2d::UnitY());
    appendDirection(dirs, anchor_dir);
    appendDirection(dirs, -anchor_dir);
    if (window_chord.norm() > 1e-9) {
      const Eigen::Vector2d unit = window_chord.normalized();
      appendDirection(dirs, Eigen::Vector2d(-unit.y(), unit.x()));
      appendDirection(dirs, Eigen::Vector2d(unit.y(), -unit.x()));
    }

    const std::vector<double> scales =
        (states.size() <= 5) ? std::vector<double>{0.05, 0.10, 0.20, 0.30, 0.40}
                             : std::vector<double>{0.05, 0.10, 0.20};
    for (const auto& weights : {uniform_weights, tail_ramp_weights, head_ramp_weights}) {
      for (double scale : scales) {
        for (const auto& dir : dirs) {
          if (tryPattern(shift_first, shift_last, weights,
                         scale * dir.x(), scale * dir.y(), 0.0)) {
            return true;
          }
        }
      }
    }
  }

  if (best_clearance > initial_quality.min_clearance + 1e-6) {
    states = best;
    return true;
  }
  return false;
}

bool repairMediumSE2WindowSliding(std::vector<SE2State>& states,
                                  const ContinuousCollisionEvaluator* svsdf,
                                  const GridMap* map,
                                  double target_clearance) {
  if (!svsdf || !map || states.size() <= 5 || states.size() > 12) return false;

  bool any_changed = false;
  for (int pass = 0; pass < 3; ++pass) {
    size_t worst_begin = states.size();
    SmallWindowQuality worst_quality;
    worst_quality.min_clearance = kInf;
    worst_quality.deficit_penalty = -kInf;
    for (size_t begin = 0; begin + 1 < states.size(); ++begin) {
      const size_t end = std::min(states.size() - 1, begin + 4);
      std::vector<SE2State> window(states.begin() + static_cast<long>(begin),
                                   states.begin() + static_cast<long>(end + 1));
      const SmallWindowQuality q =
          evaluateSmallWindowQuality(window, svsdf, map, target_clearance);
      if (q.min_clearance >= target_clearance) {
        continue;
      }
      if (worst_begin == states.size() ||
          q.deficit_penalty > worst_quality.deficit_penalty + 1e-9 ||
          (std::abs(q.deficit_penalty - worst_quality.deficit_penalty) <= 1e-9 &&
           q.min_clearance < worst_quality.min_clearance - 1e-6)) {
        worst_begin = begin;
        worst_quality = q;
      }
    }

    if (worst_begin == states.size()) {
      break;
    }

    const size_t worst_end = std::min(states.size() - 1, worst_begin + 4);
    std::vector<SE2State> repaired(states.begin() + static_cast<long>(worst_begin),
                                   states.begin() + static_cast<long>(worst_end + 1));
    bool ok = repairSmallSE2WindowBlockShift(repaired, svsdf, map, target_clearance);
    if (!ok) {
      ok = repairSmallSE2WindowGreedy(repaired, svsdf, map, target_clearance);
    }
    if (!ok) {
      break;
    }

    const SmallWindowQuality after =
        evaluateSmallWindowQuality(repaired, svsdf, map, target_clearance);
    if (!betterSmallWindowQuality(after, worst_quality)) {
      break;
    }

    for (size_t i = 0; i < repaired.size(); ++i) {
      states[worst_begin + i] = repaired[i];
    }
    any_changed = true;
  }

  return any_changed;
}

}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer() {}

void TrajectoryOptimizer::init(const GridMap& map, const ContinuousCollisionEvaluator& svsdf,
                               const OptimizerParams& params) {
  map_ = &map;
  svsdf_ = &svsdf;
  params_ = params;
}

// ---------------------------------------------------------------------------
// allocateTime: distribute total_time proportional to segment distances
// ---------------------------------------------------------------------------
std::vector<double> TrajectoryOptimizer::allocateTime(
    const std::vector<SE2State>& wps, double total_time) const {
  int n = static_cast<int>(wps.size()) - 1;
  std::vector<double> dists(n, 0.0);
  double total_dist = 0.0;
  for (int i = 0; i < n; ++i) {
    double dx = wps[i + 1].x - wps[i].x;
    double dy = wps[i + 1].y - wps[i].y;
    dists[i] = std::sqrt(dx * dx + dy * dy);
    if (dists[i] < 1e-9) dists[i] = 1e-9;
    total_dist += dists[i];
  }
  std::vector<double> durations(n);
  for (int i = 0; i < n; ++i) {
    durations[i] = total_time * dists[i] / total_dist;
    if (durations[i] < 0.01) durations[i] = 0.01;
  }
  return durations;
}

// ---------------------------------------------------------------------------
// fitMincoQuintic: fit 5th-order polynomial pieces (2D) through waypoints
// with C2 continuity at boundaries.
//
// For each piece k over [0, T_k]:
//   p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
//
// Boundary conditions per piece (6 equations, 6 unknowns per axis):
//   p(0)   = p_start        =>  c0 = p_start
//   p'(0)  = v_start        =>  c1 = v_start
//   p''(0) = a_start        =>  2*c2 = a_start
//   p(T)   = p_end
//   p'(T)  = v_end
//   p''(T) = a_end
//
// Interior velocities are estimated via Catmull-Rom style averaging.
// Interior accelerations are set to zero (natural spline).
// ---------------------------------------------------------------------------
void TrajectoryOptimizer::fitMincoQuintic(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& durations,
    const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
    const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
    std::vector<PolyPiece>& pieces) const {

  int N = static_cast<int>(positions.size()) - 1;
  if (N <= 0) {
    pieces.clear();
    return;
  }

  // Compute boundary velocities and accelerations at each waypoint.
  std::vector<Eigen::Vector2d> vels(N + 1);
  std::vector<Eigen::Vector2d> accs(N + 1);
  vels[0] = v0;
  vels[N] = vf;
  accs[0] = a0;
  accs[N] = af;

  // Interior velocities: Catmull-Rom estimate, clamped.
  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) dt_sum = 1e-6;
    vels[i] = (positions[i + 1] - positions[i - 1]) / dt_sum;
    // Clamp velocity magnitude
    double v_norm = vels[i].norm();
    double max_v = 2.0;  // match max_vel param
    if (v_norm > max_v) vels[i] *= max_v / v_norm;
  }

  // Interior accelerations: estimated from velocities for continuity.
  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) dt_sum = 1e-6;
    accs[i] = (vels[std::min(i + 1, N)] - vels[std::max(i - 1, 0)]) / dt_sum;
    // Clamp acceleration
    double a_norm = accs[i].norm();
    double max_a = 2.0;  // match max_acc param
    if (a_norm > max_a) accs[i] *= max_a / a_norm;
  }

  // Solve 6x6 system per piece, per axis.
  pieces.resize(N);
  for (int k = 0; k < N; ++k) {
    double T = durations[k];
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0,   0,     0,      0,      0,
         0, 1,   0,     0,      0,      0,
         0, 0,   2,     0,      0,      0,
         1, T,   T2,    T3,     T4,     T5,
         0, 1, 2*T,   3*T2,   4*T3,   5*T4,
         0, 0,   2,   6*T,   12*T2,  20*T3;

    // Use a single factorisation for both axes.
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 6>> qr = A.colPivHouseholderQr();

    PolyPiece piece;
    piece.duration = T;

    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Matrix<double, 6, 1> b;
      b(0) = positions[k](axis);
      b(1) = vels[k](axis);
      b(2) = accs[k](axis);
      b(3) = positions[k + 1](axis);
      b(4) = vels[k + 1](axis);
      b(5) = accs[k + 1](axis);

      Eigen::Matrix<double, 6, 1> c = qr.solve(b);
      for (int j = 0; j < 6; ++j) {
        double val = c(j);
        piece.coeffs(axis, j) = std::isfinite(val) ? val : 0.0;
      }
    }
    pieces[k] = piece;
  }
}

// ---------------------------------------------------------------------------
// fitYawQuintic: 1D version of fitMincoQuintic for yaw channel.
// Angle differences are normalised to [-pi, pi].
// ---------------------------------------------------------------------------
void TrajectoryOptimizer::fitYawQuintic(
    const std::vector<double>& yaws,
    const std::vector<double>& durations,
    double omega0, double omegaf,
    std::vector<YawPolyPiece>& pieces) const {

  int N = static_cast<int>(yaws.size()) - 1;
  if (N <= 0) {
    pieces.clear();
    return;
  }

  std::vector<double> omegas(N + 1, 0.0);
  std::vector<double> alphas(N + 1, 0.0);
  omegas[0] = omega0;
  omegas[N] = omegaf;

  for (int i = 1; i < N; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-12) dt_sum = 1e-12;
    double dy = normalizeAngle(yaws[i + 1] - yaws[i - 1]);
    omegas[i] = dy / dt_sum;
    alphas[i] = 0.0;
  }

  pieces.resize(N);
  for (int k = 0; k < N; ++k) {
    double T = durations[k];
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0,   0,     0,      0,      0,
         0, 1,   0,     0,      0,      0,
         0, 0,   2,     0,      0,      0,
         1, T,   T2,    T3,     T4,     T5,
         0, 1, 2*T,   3*T2,   4*T3,   5*T4,
         0, 0,   2,   6*T,   12*T2,  20*T3;

    Eigen::Matrix<double, 6, 1> b;
    b(0) = yaws[k];
    b(1) = omegas[k];
    b(2) = alphas[k];
    // Use unwrapped target so the polynomial doesn't wrap around.
    b(3) = yaws[k] + normalizeAngle(yaws[k + 1] - yaws[k]);
    b(4) = omegas[k + 1];
    b(5) = alphas[k + 1];

    Eigen::Matrix<double, 6, 1> c = A.colPivHouseholderQr().solve(b);

    YawPolyPiece yp;
    yp.duration = T;
    for (int j = 0; j < 6; ++j) {
      yp.coeffs(0, j) = c(j);
    }
    pieces[k] = yp;
  }
}

// ---------------------------------------------------------------------------
// optimizeSE2 — Eq.(3): gradient descent on interior waypoint positions and
// yaws.  Cost = smoothness + safety (SVSDF penalty) + dynamics penalty.
// ---------------------------------------------------------------------------
OptimizerResult TrajectoryOptimizer::optimizeSE2Detailed(
    const std::vector<SE2State>& waypoints, double total_time) {
  using Clock = std::chrono::steady_clock;
  const bool debug_medium_window = (waypoints.size() > 6);
  const auto debug_begin = Clock::now();
  auto debugElapsedMs = [&](const Clock::time_point& t0) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now() - t0)
        .count();
  };

  const double target_clearance = params_.safety_margin;
  const double safety_push_clearance = target_clearance + 0.03;
  const SE2State& segment_start = waypoints.front();
  const SE2State& segment_goal = waypoints.back();
  const double chain_clearance = continuousWaypointChainClearance(
      waypoints, svsdf_, map_ ? 0.5 * map_->resolution() : 0.05);
  double support_spacing = std::max(0.05, map_ ? map_->resolution() : 0.05);
  if (waypoints.size() > 12) {
    support_spacing = std::max(support_spacing, map_ ? 3.0 * map_->resolution() : 0.15);
  }
  std::vector<SE2State> wps = extractSupportStates(waypoints, support_spacing);
  int n_wps = static_cast<int>(wps.size());
  const bool tiny_window = (n_wps <= 5);
  const bool medium_window = (n_wps <= 12);
  auto buildResult = [&](const Trajectory& t, OptimizerSourceMode mode) {
    OptimizerResult result;
    result.success = !t.empty();
    result.traj = t;
    result.source_info.source_mode = mode;
    result.source_info.used_guard =
        (mode == OptimizerSourceMode::POLYLINE_GUARD ||
         mode == OptimizerSourceMode::ROTATE_TRANSLATE_GUARD);
    result.source_info.continuous_source_ok =
        (mode == OptimizerSourceMode::CONTINUOUS) && !t.empty();
    if (!t.empty()) {
      result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(t) : kInf;
      result.dynamics_ok = dynamicsFeasible(t);
      result.cost = computeCost(t, waypoints, true, true);
    }
    return result;
  };

  if (n_wps < 2) return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);

  auto buildSupportOnlyResult = [&](const std::vector<SE2State>& states) {
    const Trajectory traj = buildSupportStateContinuousTrajectory(states, total_time);
    if (!trajectoryMatchesEndpoints(traj, segment_start, segment_goal)) {
      return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);
    }
    double min_svsdf = -kInf;
    if (!trajectoryCollisionFree(traj, svsdf_, map_, &min_svsdf)) {
      return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);
    }
    OptimizerResult result;
    result.success = !traj.empty();
    result.traj = traj;
    result.source_info.source_mode = OptimizerSourceMode::CONTINUOUS;
    result.source_info.used_guard = false;
    result.source_info.continuous_source_ok = !traj.empty();
    result.min_svsdf = min_svsdf;
    result.dynamics_ok = dynamicsFeasible(traj);
    result.cost = 0.0;
    return result;
  };

  const OptimizerResult reference_support_result = buildSupportOnlyResult(waypoints);
  if (debug_medium_window) {
    std::cout << "[opt_se2] ref_support_ms=" << debugElapsedMs(debug_begin)
              << " success=" << (reference_support_result.success ? 1 : 0)
              << " min_svsdf=" << reference_support_result.min_svsdf
              << " wps=" << waypoints.size()
              << std::endl;
  }
  if (reference_support_result.success &&
      meetsClearanceTarget(reference_support_result.min_svsdf, target_clearance) &&
      trajectoryShapeAcceptable(reference_support_result.traj, waypoints)) {
    return reference_support_result;
  }

  auto buildPrimaryResult = [&](const std::vector<SE2State>& states) {
    OptimizerResult best;

    auto consider = [&](const Trajectory& cand) {
      if (cand.empty()) {
        return;
      }
      if (!trajectoryMatchesEndpoints(cand, segment_start, segment_goal)) {
        return;
      }
      double min_svsdf = -kInf;
      if (!trajectoryCollisionFree(cand, svsdf_, map_, &min_svsdf)) {
        return;
      }
      OptimizerResult result = buildResult(cand, OptimizerSourceMode::CONTINUOUS);
      result.min_svsdf = min_svsdf;
      const bool result_hits_target = meetsClearanceTarget(result.min_svsdf, target_clearance);
      const bool best_hits_target = meetsClearanceTarget(best.min_svsdf, target_clearance);
      if (!best.success ||
          (result_hits_target && !best_hits_target) ||
          ((result_hits_target == best_hits_target) &&
           (result.min_svsdf >
                best.min_svsdf +
                    (result_hits_target ? kMeaningfulTargetHitClearanceGain : 1e-6) ||
            (std::abs(result.min_svsdf - best.min_svsdf) <= 1e-6 &&
             result.cost < best.cost)))) {
        best = result;
      }
    };

    auto considerStates = [&](const std::vector<SE2State>& candidate_states) {
      if (candidate_states.size() < 2) {
        return;
      }
      consider(buildSupportStateContinuousTrajectory(candidate_states, total_time));
      consider(buildRotateTranslateTrajectory(candidate_states, total_time));
    };

    considerStates(states);
    if (best.success &&
        meetsClearanceTarget(best.min_svsdf, target_clearance) &&
        trajectoryShapeAcceptable(best.traj, waypoints)) {
      return best;
    }
    if (svsdf_ && map_ && states.size() > 2) {
      const auto compacted = compactSupportStatesForContinuousFit(
          states, svsdf_,
          std::max(0.10, 2.0 * map_->resolution()),
          0.30,
          target_clearance + 0.5 * map_->resolution());
      if (compacted.size() >= 2 && compacted.size() < states.size()) {
        considerStates(compacted);
      }
    }
    return best;
  };

  auto pickPreferred = [&](const OptimizerResult& incumbent,
                           const OptimizerResult& candidate) {
    if (!candidate.success) return incumbent;
    if (!incumbent.success) return candidate;

    const bool candidate_hits_target =
        meetsClearanceTarget(candidate.min_svsdf, target_clearance);
    const bool incumbent_hits_target =
        meetsClearanceTarget(incumbent.min_svsdf, target_clearance);
    const bool candidate_shape_ok =
        trajectoryShapeSoftAcceptable(candidate.traj, waypoints);
    const bool incumbent_shape_ok =
        trajectoryShapeSoftAcceptable(incumbent.traj, waypoints);

    if (candidate_hits_target != incumbent_hits_target) {
      return candidate_hits_target ? candidate : incumbent;
    }
    if (candidate_shape_ok != incumbent_shape_ok) {
      return candidate_shape_ok ? candidate : incumbent;
    }
    const double clearance_gain_threshold =
        (candidate_hits_target && incumbent_hits_target)
            ? kMeaningfulTargetHitClearanceGain
            : 1e-6;
    if (candidate.min_svsdf > incumbent.min_svsdf + clearance_gain_threshold) {
      return candidate;
    }
    if (incumbent.min_svsdf > candidate.min_svsdf + clearance_gain_threshold) {
      return incumbent;
    }
    if (candidate.cost + 1e-6 < incumbent.cost) {
      return candidate;
    }
    return incumbent;
  };

  OptimizerResult selected;
  OptimizerResult selected_hard_shape;
  OptimizerResult selected_low_bulge;
  auto rememberCandidate = [&](const OptimizerResult& candidate) {
    selected = pickPreferred(selected, candidate);
    if (candidate.success &&
        meetsClearanceTarget(candidate.min_svsdf, target_clearance) &&
        maxLateralBulgeToWaypoints(candidate.traj, waypoints, kShapeSampleStep) <= 0.08) {
      selected_low_bulge = pickPreferred(selected_low_bulge, candidate);
    }
    if (candidate.success &&
        meetsClearanceTarget(candidate.min_svsdf, target_clearance) &&
        trajectoryShapeAcceptable(candidate.traj, waypoints)) {
      selected_hard_shape = pickPreferred(selected_hard_shape, candidate);
    }
  };

  const OptimizerResult reference_result = buildPrimaryResult(waypoints);
  rememberCandidate(reference_result);
  if (debug_medium_window) {
    std::cout << "[opt_se2] ref_primary_ms=" << debugElapsedMs(debug_begin)
              << " success=" << (reference_result.success ? 1 : 0)
              << " min_svsdf=" << reference_result.min_svsdf
              << std::endl;
  }
  if (reference_result.success &&
      meetsClearanceTarget(reference_result.min_svsdf, target_clearance) &&
      trajectoryShapeAcceptable(reference_result.traj, waypoints)) {
    return reference_result;
  }

  std::vector<double> durations = allocateTime(wps, total_time);
  double step = params_.step_size;
  const bool need_margin_repair =
      svsdf_ && map_ && n_wps > 8 && chain_clearance < target_clearance;
  const double repair_target_clearance =
      need_margin_repair ? (target_clearance + 0.5 * map_->resolution())
                         : target_clearance;
  if (need_margin_repair && !tiny_window) {
    const double segment_length = waypointChainLength(waypoints);
    const int max_passes = (segment_length > 1.0 || n_wps >= 5) ? 18 : 10;
    for (int round = 0; round < 3; ++round) {
      const bool densified = densifyMarginViolatingEdges(
          wps, svsdf_, map_, repair_target_clearance);
      const bool changed = pushSupportStatesTowardClearance(
          wps, svsdf_, map_, repair_target_clearance, max_passes);
      OptimizerResult round_result = buildPrimaryResult(wps);
      if (round_result.success &&
          meetsClearanceTarget(round_result.min_svsdf, target_clearance)) {
        break;
      }
      if (!densified && !changed) {
        break;
      }
    }
    durations = allocateTime(wps, total_time);
    n_wps = static_cast<int>(wps.size());
  }
  if (debug_medium_window) {
    std::cout << "[opt_se2] margin_repair_ms=" << debugElapsedMs(debug_begin)
              << " support_n=" << n_wps
              << " chain_clearance=" << chain_clearance
              << std::endl;
  }
  const std::vector<SE2State> repaired_wps = wps;
  const OptimizerResult repaired_support_result = buildSupportOnlyResult(repaired_wps);
  rememberCandidate(repaired_support_result);
  if (debug_medium_window) {
    std::cout << "[opt_se2] repaired_support_ms=" << debugElapsedMs(debug_begin)
              << " success=" << (repaired_support_result.success ? 1 : 0)
              << " min_svsdf=" << repaired_support_result.min_svsdf
              << std::endl;
  }
  if (repaired_support_result.success &&
      meetsClearanceTarget(repaired_support_result.min_svsdf, target_clearance) &&
      trajectoryShapeAcceptable(repaired_support_result.traj, waypoints)) {
  }
  const OptimizerResult repaired_result = buildPrimaryResult(repaired_wps);
  rememberCandidate(repaired_result);
  if (debug_medium_window) {
    std::cout << "[opt_se2] repaired_primary_ms=" << debugElapsedMs(debug_begin)
              << " success=" << (repaired_result.success ? 1 : 0)
              << " min_svsdf=" << repaired_result.min_svsdf
              << std::endl;
  }
  if (repaired_result.success &&
      meetsClearanceTarget(repaired_result.min_svsdf, target_clearance) &&
      trajectoryShapeAcceptable(repaired_result.traj, waypoints)) {
  }

  if (svsdf_ && map_ && medium_window && !tiny_window &&
      chain_clearance < target_clearance) {
    std::vector<SE2State> medium_repaired = repaired_wps;
    if (repairMediumSE2WindowSliding(medium_repaired, svsdf_, map_, target_clearance)) {
      const OptimizerResult medium_result = buildPrimaryResult(medium_repaired);
      rememberCandidate(medium_result);
      if (debug_medium_window) {
        std::cout << "[opt_se2] medium_ms=" << debugElapsedMs(debug_begin)
                  << " support_n=" << medium_repaired.size()
                  << " success=" << (medium_result.success ? 1 : 0)
                  << " min_svsdf=" << medium_result.min_svsdf
                  << std::endl;
      }
      if (medium_result.success &&
          meetsClearanceTarget(medium_result.min_svsdf, target_clearance) &&
          trajectoryShapeAcceptable(medium_result.traj, waypoints)) {
      }
      wps = medium_repaired;
      durations = allocateTime(wps, total_time);
      n_wps = static_cast<int>(wps.size());
    }
  }

  if (svsdf_ && map_ && medium_window && !tiny_window &&
      chain_clearance < target_clearance) {
    const auto compacted = extractSupportStates(
        repaired_wps, std::max(0.30, 6.0 * map_->resolution()));
    if (compacted.size() >= 2 && compacted.size() < repaired_wps.size()) {
      std::vector<SE2State> coarse = compacted;
      if (coarse.size() > 5) {
        std::vector<SE2State> sliding_repaired = coarse;
        if (repairMediumSE2WindowSliding(sliding_repaired, svsdf_, map_, target_clearance)) {
          coarse = std::move(sliding_repaired);
        }
      }
      if (coarse.size() > 5) {
        coarse = reduceStatesByWorstClearance(coarse, svsdf_, 8);
      }
      if (coarse.size() <= 6) {
        std::vector<SE2State> compact_repaired = coarse;
        if (repairSmallSE2WindowBlockShift(compact_repaired, svsdf_, map_, target_clearance) ||
            repairSmallSE2WindowGreedy(compact_repaired, svsdf_, map_, target_clearance)) {
          const OptimizerResult compact_repaired_result = buildPrimaryResult(compact_repaired);
          rememberCandidate(compact_repaired_result);
          if (compact_repaired_result.success &&
              meetsClearanceTarget(compact_repaired_result.min_svsdf, target_clearance) &&
              trajectoryShapeAcceptable(compact_repaired_result.traj, waypoints)) {
          }
          coarse = std::move(compact_repaired);
        }
      }
      const OptimizerResult compact_result = buildPrimaryResult(coarse);
      rememberCandidate(compact_result);
      if (debug_medium_window) {
        std::cout << "[opt_se2] compact_ms=" << debugElapsedMs(debug_begin)
                  << " compact_n=" << coarse.size()
                  << " success=" << (compact_result.success ? 1 : 0)
                  << " min_svsdf=" << compact_result.min_svsdf
                  << std::endl;
      }
      if (compact_result.success &&
          meetsClearanceTarget(compact_result.min_svsdf, target_clearance) &&
          trajectoryShapeAcceptable(compact_result.traj, waypoints)) {
      }
      wps = coarse;
      durations = allocateTime(wps, total_time);
      n_wps = static_cast<int>(wps.size());
    }
  }

  if (svsdf_ && map_ && n_wps <= 12 && chain_clearance < target_clearance) {
    std::vector<SE2State> block_shifted = wps;
    if (repairSmallSE2WindowBlockShift(block_shifted, svsdf_, map_, target_clearance)) {
      const OptimizerResult block_result = buildPrimaryResult(block_shifted);
      rememberCandidate(block_result);
      if (debug_medium_window) {
        std::cout << "[opt_se2] block_shift_ms=" << debugElapsedMs(debug_begin)
                  << " support_n=" << block_shifted.size()
                  << " success=" << (block_result.success ? 1 : 0)
                  << " min_svsdf=" << block_result.min_svsdf
                  << std::endl;
      }
      if (block_result.success &&
          meetsClearanceTarget(block_result.min_svsdf, target_clearance) &&
          trajectoryShapeAcceptable(block_result.traj, waypoints)) {
      }
      wps = block_shifted;
      durations = allocateTime(wps, total_time);
      n_wps = static_cast<int>(wps.size());
    }
  }

  if (svsdf_ && map_ && n_wps <= 6 && chain_clearance < target_clearance) {
    std::vector<SE2State> locally_repaired = wps;
    if (repairSmallSE2WindowGreedy(locally_repaired, svsdf_, map_, target_clearance)) {
      const OptimizerResult local_result = buildPrimaryResult(locally_repaired);
      rememberCandidate(local_result);
      if (local_result.success &&
          meetsClearanceTarget(local_result.min_svsdf, target_clearance) &&
          trajectoryShapeAcceptable(local_result.traj, waypoints)) {
      }
      wps = locally_repaired;
      durations = allocateTime(wps, total_time);
      n_wps = static_cast<int>(wps.size());
    }
  }

  const int max_iterations =
      (n_wps <= 5) ? std::min(params_.max_iterations, 24)
                   : ((n_wps <= 12) ? std::min(params_.max_iterations, 24)
                                    : params_.max_iterations);
  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(n_wps);
    std::vector<double> yaws(n_wps);
    for (int i = 0; i < n_wps; ++i) {
      positions[i] = Eigen::Vector2d(wps[i].x, wps[i].y);
      yaws[i] = wps[i].yaw;
    }

    double total_grad_norm = 0.0;

    for (int i = 1; i < n_wps - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;

      // --- Smoothness: time-aware finite-difference (paper Eq.3 J_s) ---
      double dt_prev = durations[i - 1];
      double dt_next = durations[i];
      double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      // Acceleration approximation at waypoint i
      Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      // Gradient of ||acc||^2 w.r.t. positions[i]: -4*acc / dt^2
      // Clamp to prevent blow-up
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      double gs_norm = grad_smooth.norm();
      if (gs_norm > 100.0) grad_smooth *= 100.0 / gs_norm;
      grad_pos += params_.lambda_smooth * grad_smooth;

      // Yaw smoothness
      double yaw_dd = (normalizeAngle(yaws[i + 1] - yaws[i]) -
                       normalizeAngle(yaws[i] - yaws[i - 1])) /
                      (dt_avg * dt_avg);
      double yaw_grad = params_.lambda_smooth * (-4.0 * yaw_dd / (dt_avg * dt_avg));
      if (std::abs(yaw_grad) > 100.0) yaw_grad = (yaw_grad > 0) ? 100.0 : -100.0;
      grad_yaw += yaw_grad;

      // --- Safety: SVSDF penalty when distance < safety_margin ---
      if (svsdf_) {
        double sdf_val = svsdf_->evaluate(wps[i]);
        if (sdf_val < params_.safety_margin) {
          Eigen::Vector2d gp;
          double gy;
          svsdf_->gradient(wps[i], gp, gy);
          double pen = params_.safety_margin - sdf_val;
          // Gradient of pen^2 w.r.t. state, pushing away from obstacle.
          grad_pos -= params_.lambda_safety * 2.0 * pen * gp;
          grad_yaw -= params_.lambda_safety * 2.0 * pen * gy;
        }

        auto accumulateEdgeSafety = [&](const SE2State& a,
                                        const SE2State& b,
                                        double current_weight) {
          const double len = (b.position() - a.position()).norm();
          const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
          const double sample_step = map_ ? std::max(0.5 * map_->resolution(), 1e-3) : 0.05;
          const int linear_steps =
              std::max(1, static_cast<int>(std::ceil(len / sample_step)));
          const int yaw_steps =
              std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.1)));
          const int n_steps = std::max(linear_steps, yaw_steps);
          for (int s = 1; s < n_steps; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(n_steps);
            SE2State sample;
            sample.x = a.x + (b.x - a.x) * t;
            sample.y = a.y + (b.y - a.y) * t;
            sample.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
            const double sdf = svsdf_->evaluate(sample);
            if (sdf >= params_.safety_margin) {
              continue;
            }
            Eigen::Vector2d gp;
            double gy = 0.0;
            svsdf_->gradient(sample, gp, gy);
            const double pen = params_.safety_margin - sdf;
            const double weight =
                std::max(0.0, std::min(1.0, current_weight));
            grad_pos -= params_.lambda_safety * 2.0 * pen * weight * gp;
            grad_yaw -= params_.lambda_safety * 2.0 * pen * weight * gy;
          }
        };
        accumulateEdgeSafety(wps[i - 1], wps[i], 1.0);
        accumulateEdgeSafety(wps[i], wps[i + 1], 1.0);
      }

      // --- Dynamics: penalise velocity exceeding max_vel ---
      double dt_span = dt_prev + dt_next;
      if (dt_span < 1e-12) dt_span = 1e-12;
      Eigen::Vector2d vel_est =
          (positions[i + 1] - positions[i - 1]) / dt_span;
      double vel_norm = vel_est.norm();
      if (vel_norm > params_.max_vel) {
        double excess = vel_norm - params_.max_vel;
        // d/d(p_i) of vel_est is zero (p_i cancels), but we add a soft
        // centering penalty that pulls the waypoint toward the midpoint.
        Eigen::Vector2d mid = 0.5 * (positions[i - 1] + positions[i + 1]);
        grad_pos += params_.lambda_dynamics * excess *
                    (positions[i] - mid) / (vel_norm + 1e-12);
      }

      // --- Dynamics: penalise acceleration exceeding max_acc ---
      Eigen::Vector2d acc_est = acc_approx;  // reuse from smoothness
      double acc_norm = acc_est.norm();
      if (acc_norm > params_.max_acc) {
        double excess_a = acc_norm - params_.max_acc;
        // Push waypoint to reduce curvature
        grad_pos += params_.lambda_dynamics * excess_a *
                    (-2.0 * acc_est) / (acc_norm * dt_avg * dt_avg + 1e-12);
      }

      // Clamp total gradient to prevent instability
      double total_g = grad_pos.norm();
      if (total_g > 50.0) grad_pos *= 50.0 / total_g;
      if (std::abs(grad_yaw) > 50.0) grad_yaw = (grad_yaw > 0) ? 50.0 : -50.0;

      // Apply gradient step.
      wps[i].x -= step * grad_pos.x();
      wps[i].y -= step * grad_pos.y();
      wps[i].yaw = normalizeAngle(wps[i].yaw - step * grad_yaw);

      total_grad_norm += grad_pos.squaredNorm() + grad_yaw * grad_yaw;
    }

    // Re-allocate time after position update.
    durations = allocateTime(wps, total_time);

    if (total_grad_norm < 1e-10) break;
  }
  const OptimizerResult optimized_result = buildPrimaryResult(wps);
  rememberCandidate(optimized_result);
  if (debug_medium_window) {
    std::cout << "[opt_se2] iter_ms=" << debugElapsedMs(debug_begin)
              << " success=" << (optimized_result.success ? 1 : 0)
              << " min_svsdf=" << optimized_result.min_svsdf
              << " final_support_n=" << n_wps
              << std::endl;
  }

  auto boundedLocalRepairResult = [&](const std::vector<SE2State>& seed) {
    if (!svsdf_ || !map_ || seed.size() <= 2 || seed.size() > 6) {
      return OptimizerResult();
    }
    std::vector<SE2State> repaired = seed;
    if (!repairSmallSE2WindowGreedy(repaired, svsdf_, map_, target_clearance)) {
      return OptimizerResult();
    }
    return buildPrimaryResult(repaired);
  };

  auto smallWindowTrajectorySearchResult = [&](const std::vector<SE2State>& seed) {
    if (!svsdf_ || !map_ || seed.size() <= 2 || seed.size() > 8) {
      return OptimizerResult();
    }

    std::vector<SE2State> best_states = seed;
    OptimizerResult best = buildPrimaryResult(best_states);
    auto better = [&](const OptimizerResult& candidate,
                      const OptimizerResult& incumbent) {
      if (!candidate.success) return false;
      if (!incumbent.success) return true;
      const bool cand_hits = meetsClearanceTarget(candidate.min_svsdf, target_clearance);
      const bool inc_hits = meetsClearanceTarget(incumbent.min_svsdf, target_clearance);
      if (cand_hits != inc_hits) return cand_hits;
      if (candidate.min_svsdf > incumbent.min_svsdf + 1e-6) return true;
      if (incumbent.min_svsdf > candidate.min_svsdf + 1e-6) return false;
      return candidate.cost + 1e-6 < incumbent.cost;
    };

    for (int round = 0; round < 2; ++round) {
      const double step_xy = (round == 0) ? 0.08 : 0.04;
      const double step_yaw = (round == 0) ? 0.18 : 0.09;
      bool round_improved = false;
      std::vector<std::pair<double, size_t>> priorities;
      priorities.reserve(best_states.size());
      for (size_t i = 1; i + 1 < best_states.size(); ++i) {
        const double clearance =
            localWindowClearance(best_states, i, best_states[i], svsdf_, map_);
        priorities.emplace_back(clearance, i);
      }
      std::sort(priorities.begin(), priorities.end(),
                [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
      if (priorities.size() > 2) priorities.resize(2);

      for (const auto& priority : priorities) {
        const size_t idx = priority.second;
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();
        double grad_yaw = 0.0;
        svsdf_->gradient(best_states[idx], grad, grad_yaw);
        std::vector<Eigen::Vector2d> dirs;
        if (grad.norm() > 1e-6) {
          const Eigen::Vector2d gn = grad.normalized();
          dirs.push_back(gn);
          dirs.push_back(Eigen::Vector2d(-gn.y(), gn.x()));
          dirs.push_back(Eigen::Vector2d(gn.y(), -gn.x()));
        } else {
          dirs.push_back(Eigen::Vector2d::UnitX());
          dirs.push_back(Eigen::Vector2d::UnitY());
        }

        for (const auto& dir : dirs) {
          for (int iyaw = -1; iyaw <= 1; ++iyaw) {
            std::vector<SE2State> candidate = best_states;
            candidate[idx].x += step_xy * dir.x();
            candidate[idx].y += step_xy * dir.y();
            candidate[idx].yaw =
                normalizeAngle(candidate[idx].yaw + iyaw * step_yaw);
            const OptimizerResult candidate_result = buildPrimaryResult(candidate);
            if (!trajectoryShapeAcceptable(candidate_result.traj, waypoints)) {
              continue;
            }
            if (better(candidate_result, best)) {
              best = candidate_result;
              best_states = std::move(candidate);
              round_improved = true;
            }
            if (candidate_result.success &&
                meetsClearanceTarget(candidate_result.min_svsdf, target_clearance)) {
              return candidate_result;
            }
          }
        }
      }

      if (!round_improved) {
        break;
      }
    }
    return best;
  };

  rememberCandidate(boundedLocalRepairResult(wps));
  rememberCandidate(smallWindowTrajectorySearchResult(repaired_wps));
  rememberCandidate(smallWindowTrajectorySearchResult(wps));
  if (selected_low_bulge.success) {
    return selected_low_bulge;
  }
  if (selected_hard_shape.success) {
    return selected_hard_shape;
  }
  if (selected.success) {
    return selected;
  }
  return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);
}

Trajectory TrajectoryOptimizer::optimizeSE2(
    const std::vector<SE2State>& waypoints, double total_time) {
  return optimizeSE2Detailed(waypoints, total_time).traj;
}

// ---------------------------------------------------------------------------
// optimizeR2 — Eq.(4): gradient descent on interior waypoint positions only.
// Cost = smoothness + position residual + rotation residual.
// Yaw is derived from velocity direction after optimisation.
// ---------------------------------------------------------------------------
OptimizerResult TrajectoryOptimizer::optimizeR2Detailed(
    const std::vector<SE2State>& waypoints, double total_time) {

  const double target_clearance = params_.safety_margin;
  const double safety_push_clearance = target_clearance + 0.03;
  const SE2State& segment_start = waypoints.front();
  const SE2State& segment_goal = waypoints.back();
  const double chain_clearance = continuousWaypointChainClearance(
      waypoints, svsdf_, map_ ? 0.5 * map_->resolution() : 0.05);
  const bool low_segment_needs_buffer = chain_clearance < safety_push_clearance;
  const double pos_residual_weight =
      low_segment_needs_buffer ? 0.5 * params_.lambda_pos_residual
                               : params_.lambda_pos_residual;
  const double yaw_residual_weight =
      low_segment_needs_buffer ? 0.5 * params_.lambda_yaw_residual
                               : params_.lambda_yaw_residual;
  const double safety_weight =
      low_segment_needs_buffer ? 2.0 * params_.lambda_safety
                               : params_.lambda_safety;
  // LOW segments should only fail fast on actual collision. Using the full
  // safety margin here rejects chains whose eventual continuous fit already
  // clears the target.
  if (svsdf_ && chain_clearance < 0.0) {
    return OptimizerResult();
  }
  const double support_spacing = 0.01;
  std::vector<SE2State> wps = extractSupportStates(waypoints, support_spacing);
  int n_wps = static_cast<int>(wps.size());
  auto buildResult = [&](const Trajectory& t, OptimizerSourceMode mode) {
    OptimizerResult result;
    result.success = !t.empty();
    result.traj = t;
    result.source_info.source_mode = mode;
    result.source_info.used_guard =
        (mode == OptimizerSourceMode::POLYLINE_GUARD ||
         mode == OptimizerSourceMode::ROTATE_TRANSLATE_GUARD);
    result.source_info.continuous_source_ok =
        (mode == OptimizerSourceMode::CONTINUOUS) && !t.empty();
    if (!t.empty()) {
      result.min_svsdf = svsdf_ ? svsdf_->evaluateTrajectory(t) : kInf;
      result.dynamics_ok = dynamicsFeasible(t);
      result.cost = computeCost(t, waypoints, false, true);
    }
    return result;
  };

  if (n_wps < 2) return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);

  std::vector<SE2State> ref = wps;
  auto buildPrimaryResult = [&](const std::vector<SE2State>& states) {
    OptimizerResult best = buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);

    auto consider = [&](const Trajectory& traj, OptimizerSourceMode mode) {
      if (!trajectoryMatchesEndpoints(traj, segment_start, segment_goal)) {
        return;
      }
      double min_svsdf = -kInf;
      if (!trajectoryCollisionFree(traj, svsdf_, map_, &min_svsdf)) {
        return;
      }
      if (!meetsClearanceTarget(min_svsdf, target_clearance)) {
        return;
      }
      OptimizerResult result = buildResult(traj, mode);
      result.min_svsdf = min_svsdf;
      if (!best.success ||
          result.min_svsdf > best.min_svsdf + 1e-6 ||
          (std::abs(result.min_svsdf - best.min_svsdf) <= 1e-6 &&
           result.cost + 1e-6 < best.cost)) {
        best = result;
      }
    };

    const Trajectory continuous =
        buildSupportStateContinuousTrajectory(states, total_time);
    consider(continuous, OptimizerSourceMode::CONTINUOUS);
    if (!best.success || best.min_svsdf < safety_push_clearance) {
      consider(buildConservativePolylineTrajectory(states, total_time),
               OptimizerSourceMode::POLYLINE_GUARD);
    }
    if (!best.success || best.min_svsdf < safety_push_clearance) {
      consider(buildRotateTranslateTrajectory(states, total_time),
               OptimizerSourceMode::ROTATE_TRANSLATE_GUARD);
    }
    return best;
  };
  const OptimizerResult reference_result = buildPrimaryResult(ref);
  if (reference_result.success &&
      meetsClearanceTarget(reference_result.min_svsdf, target_clearance) &&
      trajectoryShapeAcceptable(reference_result.traj, waypoints)) {
    return reference_result;
  }

  std::vector<double> durations = allocateTime(wps, total_time);
  if (svsdf_ && map_ && n_wps > 2 && chain_clearance < target_clearance) {
    const double segment_length = waypointChainLength(waypoints);
    const int max_passes = (segment_length > 1.0 || n_wps >= 5) ? 12 : 8;
    for (int round = 0; round < 2; ++round) {
      const bool densified =
          densifyMarginViolatingEdges(wps, svsdf_, map_, target_clearance);
      const bool changed = pushSupportStatesTowardClearance(
          wps, svsdf_, map_, target_clearance, max_passes);
      for (size_t i = 1; i + 1 < wps.size(); ++i) {
        const Eigen::Vector2d vel_dir =
            wps[i + 1].position() - wps[i - 1].position();
        if (vel_dir.norm() > 1e-9) {
          wps[i].yaw = std::atan2(vel_dir.y(), vel_dir.x());
        }
      }
      const OptimizerResult repaired_result = buildPrimaryResult(wps);
      if (repaired_result.success &&
          meetsClearanceTarget(repaired_result.min_svsdf, target_clearance) &&
          trajectoryShapeAcceptable(repaired_result.traj, waypoints)) {
        return repaired_result;
      }
      if (!densified && !changed) {
        break;
      }
    }
    n_wps = static_cast<int>(wps.size());
    durations = allocateTime(wps, total_time);
  }

  double step = params_.step_size;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(n_wps);
    for (int i = 0; i < n_wps; ++i) {
      positions[i] = Eigen::Vector2d(wps[i].x, wps[i].y);
    }

    double total_grad_norm = 0.0;

    for (int i = 1; i < n_wps - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();

      double dt_prev = durations[i - 1];
      double dt_next = durations[i];
      double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      // --- Smoothness (time-aware) ---
      Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      double gs_norm = grad_smooth.norm();
      if (gs_norm > 100.0) grad_smooth *= 100.0 / gs_norm;
      grad_pos += params_.lambda_smooth * grad_smooth;

      // --- Position residual: stay near reference waypoint ---
      Eigen::Vector2d ref_pos(ref[i].x, ref[i].y);
      grad_pos += pos_residual_weight * 2.0 *
                  (positions[i] - ref_pos);

      // --- Rotation residual: heading from velocity should be smooth ---
      Eigen::Vector2d vel_dir = positions[i + 1] - positions[i - 1];
      double vel_norm = vel_dir.norm();
      if (vel_norm > 1e-9) {
        double yaw_curr = std::atan2(vel_dir.y(), vel_dir.x());

        // Heading at neighbours.
        double yaw_prev = wps[0].yaw;
        if (i >= 2) {
          Eigen::Vector2d vp = positions[i] - positions[i - 2];
          if (vp.norm() > 1e-9) yaw_prev = std::atan2(vp.y(), vp.x());
        }
        double yaw_next = wps[n_wps - 1].yaw;
        if (i + 2 < n_wps) {
          Eigen::Vector2d vn = positions[i + 2] - positions[i];
          if (vn.norm() > 1e-9) yaw_next = std::atan2(vn.y(), vn.x());
        }

        double yaw_dd = normalizeAngle(yaw_next - yaw_curr) -
                         normalizeAngle(yaw_curr - yaw_prev);

        // Gradient direction: perpendicular to velocity.
        Eigen::Vector2d perp(-vel_dir.y(), vel_dir.x());
        perp /= vel_norm;
        grad_pos += yaw_residual_weight * yaw_dd * perp;
      }

      // --- Safety: push LOW segments away from the safety margin instead of
      // only rejecting them after the fact.
      if (svsdf_) {
        const double sdf_val = svsdf_->evaluate(wps[i]);
        if (sdf_val < safety_push_clearance) {
          Eigen::Vector2d gp = Eigen::Vector2d::Zero();
          double gy = 0.0;
          svsdf_->gradient(wps[i], gp, gy);
          const double pen = safety_push_clearance - sdf_val;
          grad_pos -= safety_weight * 2.0 * pen * gp;
        }

        auto accumulateEdgeSafety = [&](const SE2State& a,
                                        const SE2State& b) {
          const double len = (b.position() - a.position()).norm();
          const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
          const double sample_step =
              map_ ? std::max(0.5 * map_->resolution(), 1e-3) : 0.05;
          const int linear_steps =
              std::max(1, static_cast<int>(std::ceil(len / sample_step)));
          const int yaw_steps =
              std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.1)));
          const int n_steps = std::max(linear_steps, yaw_steps);
          for (int s = 1; s < n_steps; ++s) {
            const double t = static_cast<double>(s) /
                             static_cast<double>(n_steps);
            SE2State sample;
            sample.x = a.x + (b.x - a.x) * t;
            sample.y = a.y + (b.y - a.y) * t;
            sample.yaw =
                normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
            const double sdf = svsdf_->evaluate(sample);
            if (sdf >= safety_push_clearance) {
              continue;
            }
            Eigen::Vector2d gp = Eigen::Vector2d::Zero();
            double gy = 0.0;
            svsdf_->gradient(sample, gp, gy);
            const double pen = safety_push_clearance - sdf;
            grad_pos -= safety_weight * 2.0 * pen * gp;
          }
        };
        accumulateEdgeSafety(wps[i - 1], wps[i]);
        accumulateEdgeSafety(wps[i], wps[i + 1]);
      }

      wps[i].x -= step * grad_pos.x();
      wps[i].y -= step * grad_pos.y();

      total_grad_norm += grad_pos.squaredNorm();
    }

    durations = allocateTime(wps, total_time);
    if (total_grad_norm < 1e-10) break;
  }

  for (int i = 1; i < n_wps - 1; ++i) {
    Eigen::Vector2d vel_dir = wps[i + 1].position() - wps[i - 1].position();
    if (vel_dir.norm() > 1e-9) {
      wps[i].yaw = std::atan2(vel_dir.y(), vel_dir.x());
    }
  }
  const OptimizerResult optimized_result = buildPrimaryResult(wps);
  if (optimized_result.success && reference_result.success) {
    const bool keep_shape =
        trajectoryShapeAcceptable(optimized_result.traj, waypoints);
    const bool optimized_hits_target =
        meetsClearanceTarget(optimized_result.min_svsdf, target_clearance);
    const bool reference_hits_target =
        meetsClearanceTarget(reference_result.min_svsdf, target_clearance);
    const double clearance_gain_threshold =
        (optimized_hits_target && reference_hits_target)
            ? kMeaningfulTargetHitClearanceGain
            : 1e-3;
    if (keep_shape &&
        optimized_result.min_svsdf >
            reference_result.min_svsdf + clearance_gain_threshold) {
      return optimized_result;
    }
    const bool no_worse_clearance =
        optimized_result.min_svsdf + 1e-6 >= reference_result.min_svsdf;
    const bool lower_cost = optimized_result.cost + 1e-6 < reference_result.cost;
    return (keep_shape && no_worse_clearance && lower_cost)
               ? optimized_result
               : reference_result;
  }
  if (optimized_result.success) return optimized_result;
  if (reference_result.success) return reference_result;
  return buildResult(Trajectory(), OptimizerSourceMode::UNKNOWN);
}

Trajectory TrajectoryOptimizer::optimizeR2(
    const std::vector<SE2State>& waypoints, double total_time) {
  return optimizeR2Detailed(waypoints, total_time).traj;
}

// ---------------------------------------------------------------------------
// stitch: collect all optimised waypoints and do ONE global MINCO fit
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::stitch(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& optimized) {
  size_t count = std::min(segments.size(), optimized.size());
  if (count == 0) return Trajectory();

  auto clearanceOf = [&](const Trajectory& traj) {
    return svsdf_ ? svsdf_->evaluateTrajectory(traj) : kInf;
  };

  auto preservedSegmentClearance = [&](size_t n_segments) {
    if (!svsdf_) return kInf;
    double min_clearance = kInf;
    for (size_t i = 0; i < n_segments; ++i) {
      if (optimized[i].empty()) return -kInf;
      min_clearance = std::min(min_clearance, clearanceOf(optimized[i]));
    }
    return min_clearance;
  };

  auto preserveByTimeScaling = [&](const Trajectory& raw, double preserved_clearance) {
    if (raw.empty()) return Trajectory();

    const std::array<double, 14> kScaleCandidates = {
        1.0, 1.25, 1.5, 2.0, 3.0, 4.0, 6.0,
        8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0};

    const Trajectory base = retimeToDynamicLimits(raw);
    for (double scale : kScaleCandidates) {
      const Trajectory candidate =
          (std::abs(scale - 1.0) < 1e-9) ? base : scaleTrajectoryTime(base, scale);
      if (candidate.empty()) continue;
      if (!dynamicsFeasible(candidate)) continue;
      if (!trajectoryBoundaryContinuityAcceptable(candidate)) continue;
      const double candidate_clearance =
          std::isfinite(preserved_clearance) ? preserved_clearance : clearanceOf(candidate);
      if (!meetsClearanceTarget(candidate_clearance, params_.safety_margin)) continue;
      return candidate;
    }
    return Trajectory();
  };

  if (count == 1) {
    return preserveByTimeScaling(optimized[0], preservedSegmentClearance(1));
  }

  // Preserve the already-optimized segment geometry, but bridge inter-segment
  // state gaps with local connectors instead of assuming exact endpoint
  // alignment.
  Trajectory concatenated;
  double stitched_clearance = preservedSegmentClearance(count);
  for (size_t i = 0; i < count; ++i) {
    if (optimized[i].empty()) return Trajectory();
    concatenated.pos_pieces.insert(concatenated.pos_pieces.end(),
                                   optimized[i].pos_pieces.begin(),
                                   optimized[i].pos_pieces.end());
    concatenated.yaw_pieces.insert(concatenated.yaw_pieces.end(),
                                   optimized[i].yaw_pieces.begin(),
                                   optimized[i].yaw_pieces.end());
    if (i + 1 >= count) {
      continue;
    }

    const SE2State seam_left = optimized[i].sample(optimized[i].totalDuration());
    const SE2State seam_right = optimized[i + 1].sample(0.0);
    const double seam_pos_gap =
        (seam_left.position() - seam_right.position()).norm();
    const double seam_yaw_gap =
        std::abs(normalizeAngle(seam_left.yaw - seam_right.yaw));
    if (seam_pos_gap < 1e-3 && seam_yaw_gap < 1e-3) {
      continue;
    }

    const double connector_time = std::max(
        0.20,
        seam_pos_gap / std::max(0.10, params_.max_vel) +
            seam_yaw_gap / std::max(0.10, params_.max_yaw_rate));
    const std::vector<SE2State> connector_wps = {seam_left, seam_right};
    const Trajectory connector =
        buildRotateTranslateTrajectory(connector_wps, connector_time);
    if (connector.empty()) {
      return Trajectory();
    }
    const double connector_clearance = clearanceOf(connector);
    if (!meetsClearanceTarget(connector_clearance, params_.safety_margin)) {
      return Trajectory();
    }
    stitched_clearance = std::min(stitched_clearance, connector_clearance);
    concatenated.pos_pieces.insert(concatenated.pos_pieces.end(),
                                   connector.pos_pieces.begin(),
                                   connector.pos_pieces.end());
    concatenated.yaw_pieces.insert(concatenated.yaw_pieces.end(),
                                   connector.yaw_pieces.begin(),
                                   connector.yaw_pieces.end());
  }

  return preserveByTimeScaling(concatenated, stitched_clearance);
}

// ---------------------------------------------------------------------------
// selectBest: prefer larger clearance, then minimum integral of acceleration squared
// ---------------------------------------------------------------------------
Trajectory TrajectoryOptimizer::selectBest(
    const std::vector<Trajectory>& candidates) {

  if (candidates.empty()) return Trajectory();

  double best_clearance = -kInf;
  double best_cost = kInf;
  size_t best_idx = 0;
  bool found = false;

  for (size_t c = 0; c < candidates.size(); ++c) {
    const Trajectory& traj = candidates[c];
    if (traj.empty()) continue;

    double min_sdf = kInf;
    if (svsdf_) {
      min_sdf = svsdf_->evaluateTrajectory(traj);
      if (min_sdf < 0.0) continue;
    }
    if (!dynamicsFeasible(traj)) continue;

    double cost = 0.0;

    for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
      const PolyPiece& pp = traj.pos_pieces[i];
      double T = pp.duration;
      // Simpson's rule with 20 sub-intervals.
      int n_sub = 20;
      double h = T / n_sub;
      for (int s = 0; s <= n_sub; ++s) {
        double t = s * h;
        Eigen::Vector2d a = pp.acceleration(t);
        double w = (s == 0 || s == n_sub) ? 1.0 :
                   (s % 2 == 1) ? 4.0 : 2.0;
        cost += w * a.squaredNorm() * h / 3.0;
      }
    }

    const bool better_clearance = (min_sdf > best_clearance + 1e-6);
    const bool same_clearance = std::abs(min_sdf - best_clearance) <= 1e-6;
    if (!found || better_clearance || (same_clearance && cost < best_cost)) {
      best_clearance = min_sdf;
      best_cost = cost;
      best_idx = c;
      found = true;
    }
  }
  if (!found) return Trajectory();
  return candidates[best_idx];
}

// ---------------------------------------------------------------------------
// computeCost: evaluate total cost for a trajectory
// ---------------------------------------------------------------------------
double TrajectoryOptimizer::computeCost(
    const Trajectory& traj, const std::vector<SE2State>& ref_wps,
    bool include_svsdf, bool include_residual) {

  if (traj.empty()) return kInf;

  double cost = 0.0;
  double total_dur = traj.totalDuration();
  double dt_sample = 0.05;
  int n_samples = std::max(1, static_cast<int>(total_dur / dt_sample));
  double dt = total_dur / n_samples;

  for (int s = 0; s <= n_samples; ++s) {
    double t = std::min(static_cast<double>(s) * dt, total_dur);

    // Locate piece.
    double acc_time = 0.0;
    for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
      double T_i = traj.pos_pieces[i].duration;
      if (t <= acc_time + T_i || i + 1 == traj.pos_pieces.size()) {
        double lt = t - acc_time;

        // Smoothness: acceleration squared.
        Eigen::Vector2d a = traj.pos_pieces[i].acceleration(lt);
        cost += params_.lambda_smooth * a.squaredNorm() * dt;

        // Safety (SVSDF).
        if (include_svsdf && svsdf_) {
          SE2State st = traj.sample(t);
          double sdf_val = svsdf_->evaluate(st);
          if (sdf_val < params_.safety_margin) {
            double pen = params_.safety_margin - sdf_val;
            cost += params_.lambda_safety * pen * pen * dt;
          }
        }
        break;
      }
      acc_time += T_i;
    }
  }

  // Position and yaw residual costs.
  if (include_residual && !ref_wps.empty()) {
    int n_ref = static_cast<int>(ref_wps.size());
    double ref_dt = (n_ref > 1) ? total_dur / (n_ref - 1) : 0.0;
    for (int i = 0; i < n_ref; ++i) {
      double t_ref = i * ref_dt;
      SE2State sampled = traj.sample(t_ref);
      double dx = sampled.x - ref_wps[i].x;
      double dy = sampled.y - ref_wps[i].y;
      cost += params_.lambda_pos_residual * (dx * dx + dy * dy);

      double dyaw = normalizeAngle(sampled.yaw - ref_wps[i].yaw);
      cost += params_.lambda_yaw_residual * dyaw * dyaw;
    }
  }

  // Time cost.
  cost += params_.lambda_time * total_dur;

  return cost;
}

Trajectory TrajectoryOptimizer::scaleTrajectoryTime(const Trajectory& traj,
                                                    double scale) const {
  if (traj.empty()) return traj;
  if (scale <= 1e-9) return traj;
  if (std::abs(scale - 1.0) < 1e-9) return traj;

  Trajectory scaled = traj;
  for (auto& pp : scaled.pos_pieces) {
    double scale_pow = scale;
    for (int i = 1; i < 6; ++i) {
      pp.coeffs.col(i) /= scale_pow;
      scale_pow *= scale;
    }
    pp.duration *= scale;
  }

  for (auto& yp : scaled.yaw_pieces) {
    double scale_pow = scale;
    for (int i = 1; i < 6; ++i) {
      yp.coeffs(0, i) /= scale_pow;
      scale_pow *= scale;
    }
    yp.duration *= scale;
  }

  return scaled;
}

Trajectory TrajectoryOptimizer::retimeToDynamicLimits(const Trajectory& traj) const {
  if (traj.empty()) return traj;

  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;
  checkDynamics(traj, &max_vel, &max_acc, &max_yaw_rate);

  double vel_scale = 1.0;
  double acc_scale = 1.0;
  double yaw_scale = 1.0;
  if (params_.max_vel > 1e-9) {
    vel_scale = max_vel / params_.max_vel;
  }
  if (params_.max_acc > 1e-9) {
    acc_scale = std::sqrt(std::max(0.0, max_acc) / params_.max_acc);
  }
  if (params_.max_yaw_rate > 1e-9) {
    yaw_scale = max_yaw_rate / params_.max_yaw_rate;
  }

  double scale = std::max(vel_scale, std::max(acc_scale, yaw_scale));
  scale = std::max(0.75, scale);
  scale *= 1.02;

  return scaleTrajectoryTime(traj, scale);
}

bool TrajectoryOptimizer::dynamicsFeasible(const Trajectory& traj,
                                           double* max_vel,
                                           double* max_acc,
                                           double* max_yaw_rate) const {
  return checkDynamics(traj, max_vel, max_acc, max_yaw_rate);
}

// ---------------------------------------------------------------------------
// checkDynamics: verify max velocity, acceleration, and yaw-rate constraints
// ---------------------------------------------------------------------------
bool TrajectoryOptimizer::checkDynamics(const Trajectory& traj,
                                        double* out_max_vel,
                                        double* out_max_acc,
                                        double* out_max_yaw_rate) const {
  double dt_sample = 0.02;
  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;

  for (size_t i = 0; i < traj.pos_pieces.size(); ++i) {
    const PolyPiece& pp = traj.pos_pieces[i];
    double T = pp.duration;
    int n_samples = std::max(1, static_cast<int>(T / dt_sample));
    double dt = T / n_samples;

    for (int s = 0; s <= n_samples; ++s) {
      double t = s * dt;
      max_vel = std::max(max_vel, pp.velocity(t).norm());
      max_acc = std::max(max_acc, pp.acceleration(t).norm());
    }
  }

  for (size_t i = 0; i < traj.yaw_pieces.size(); ++i) {
    const YawPolyPiece& yp = traj.yaw_pieces[i];
    double T = yp.duration;
    int n_samples = std::max(1, static_cast<int>(T / dt_sample));
    double dt = T / n_samples;

    for (int s = 0; s <= n_samples; ++s) {
      double t = s * dt;
      max_yaw_rate = std::max(max_yaw_rate, std::abs(yp.velocity(t)));
    }
  }

  if (out_max_vel) *out_max_vel = max_vel;
  if (out_max_acc) *out_max_acc = max_acc;
  if (out_max_yaw_rate) *out_max_yaw_rate = max_yaw_rate;

  return max_vel <= params_.max_vel * 1.10 &&
         max_acc <= params_.max_acc * 1.10 &&
         max_yaw_rate <= params_.max_yaw_rate * 1.10;
}

Trajectory TrajectoryOptimizer::buildConservativePolylineTrajectory(
    const std::vector<SE2State>& waypoints,
    double total_time) const {
  if (waypoints.size() < 2) return Trajectory();

  std::vector<double> durations = allocateTime(waypoints, total_time);
  if (durations.size() + 1 != waypoints.size()) return Trajectory();

  Trajectory traj;
  traj.pos_pieces.reserve(durations.size());
  traj.yaw_pieces.reserve(durations.size());

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double T = std::max(0.05, durations[i]);
    const Eigen::Vector2d delta = b.position() - a.position();

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = a.position();
    pp.coeffs.col(1) = delta / T;
    pp.coeffs.col(2).setZero();
    pp.coeffs.col(3).setZero();
    pp.coeffs.col(4).setZero();
    pp.coeffs.col(5).setZero();
    traj.pos_pieces.push_back(pp);

    const double yaw_delta = normalizeAngle(b.yaw - a.yaw);
    YawPolyPiece yp;
    yp.duration = T;
    yp.coeffs(0, 0) = a.yaw;
    yp.coeffs(0, 1) = yaw_delta / T;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 0.0;
    yp.coeffs(0, 4) = 0.0;
    yp.coeffs(0, 5) = 0.0;
    traj.yaw_pieces.push_back(yp);
  }

  return retimeToDynamicLimits(traj);
}

Trajectory TrajectoryOptimizer::buildSupportStateContinuousTrajectory(
    const std::vector<SE2State>& support_states,
    double total_time) const {
  if (support_states.size() < 2) return Trajectory();

  std::vector<double> durations = allocateTime(support_states, total_time);
  if (durations.size() + 1 != support_states.size()) return Trajectory();

  Trajectory traj;
  traj.pos_pieces.reserve(durations.size());
  traj.yaw_pieces.reserve(durations.size());

  for (size_t i = 0; i + 1 < support_states.size(); ++i) {
    const SE2State& a = support_states[i];
    const SE2State& b = support_states[i + 1];
    const double T = std::max(0.05, durations[i]);
    const Eigen::Vector2d delta = b.position() - a.position();

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = a.position();
    pp.coeffs.col(1) = delta / T;
    pp.coeffs.col(2).setZero();
    pp.coeffs.col(3).setZero();
    pp.coeffs.col(4).setZero();
    pp.coeffs.col(5).setZero();
    traj.pos_pieces.push_back(pp);

    const double yaw_delta = normalizeAngle(b.yaw - a.yaw);
    YawPolyPiece yp;
    yp.duration = T;
    yp.coeffs(0, 0) = a.yaw;
    yp.coeffs(0, 1) = yaw_delta / T;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 0.0;
    yp.coeffs(0, 4) = 0.0;
    yp.coeffs(0, 5) = 0.0;
    traj.yaw_pieces.push_back(yp);
  }

  return retimeToDynamicLimits(traj);
}

Trajectory TrajectoryOptimizer::buildRotateTranslateTrajectory(
    const std::vector<SE2State>& waypoints,
    double total_time) const {
  if (waypoints.size() < 2) return Trajectory();

  struct Primitive {
    Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p1 = Eigen::Vector2d::Zero();
    double yaw0 = 0.0;
    double yaw1 = 0.0;
    double duration = 0.1;
    bool translate = false;
  };

  std::vector<Primitive> prims;
  prims.reserve(waypoints.size() * 3);
  const double pos_dt = std::max(0.10, params_.max_vel);
  const double yaw_dt = std::max(0.10, params_.max_yaw_rate);
  const double linear_sample = map_ ? 0.5 * map_->resolution() : 0.05;
  const double yaw_sample = 0.1;

  double current_yaw = waypoints.front().yaw;
  double nominal_total = 0.0;

  auto appendRotation = [&](const Eigen::Vector2d& pos, double from_yaw, double to_yaw) {
    const double delta = std::abs(normalizeAngle(to_yaw - from_yaw));
    if (delta < 1e-6) return;
    Primitive prim;
    prim.p0 = pos;
    prim.p1 = pos;
    prim.yaw0 = from_yaw;
    prim.yaw1 = to_yaw;
    prim.duration = std::max(0.05, delta / yaw_dt);
    prim.translate = false;
    nominal_total += prim.duration;
    prims.push_back(prim);
  };

  auto appendTranslation = [&](const Eigen::Vector2d& from,
                               const Eigen::Vector2d& to,
                               double yaw) {
    const double dist = (to - from).norm();
    if (dist < 1e-9) return;
    Primitive prim;
    prim.p0 = from;
    prim.p1 = to;
    prim.yaw0 = yaw;
    prim.yaw1 = yaw;
    prim.duration = std::max(0.05, dist / pos_dt);
    prim.translate = true;
    nominal_total += prim.duration;
    prims.push_back(prim);
  };

  auto bestTranslationYaw = [&](const SE2State& a,
                                const SE2State& b,
                                double from_yaw,
                                double& out_yaw) {
    double best_clearance = -kInf;
    double best_yaw = a.yaw;
    std::vector<double> candidates = {
        a.yaw,
        b.yaw,
        from_yaw,
        std::atan2((b.y - a.y), (b.x - a.x))
    };
    constexpr int kYawSamples = 36;
    for (int i = 0; i < kYawSamples; ++i) {
      candidates.push_back(-M_PI + (2.0 * M_PI * static_cast<double>(i)) /
                                       static_cast<double>(kYawSamples));
    }

    for (double cand_yaw : candidates) {
      const double rot_in = inPlaceRotationClearance(
          a.position(), from_yaw, cand_yaw, svsdf_, yaw_sample);
      if (rot_in < 0.0) continue;
      const double trans = constantYawTranslationClearance(
          a, b, cand_yaw, svsdf_, linear_sample);
      if (trans < 0.0) continue;
      const double rot_out = inPlaceRotationClearance(
          b.position(), cand_yaw, b.yaw, svsdf_, yaw_sample);
      if (rot_out < 0.0) continue;

      const double overall = std::min(rot_in, std::min(trans, rot_out));
      if (overall > best_clearance) {
        best_clearance = overall;
        best_yaw = cand_yaw;
      }
    }

    if (best_clearance < 0.0) {
      return false;
    }
    out_yaw = best_yaw;
    return true;
  };

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];

    double trans_yaw = current_yaw;
    if (!bestTranslationYaw(a, b, current_yaw, trans_yaw)) {
      return Trajectory();
    }
    appendRotation(a.position(), current_yaw, trans_yaw);
    appendTranslation(a.position(), b.position(), trans_yaw);
    appendRotation(b.position(), trans_yaw, b.yaw);
    current_yaw = b.yaw;
  }

  if (prims.empty()) return Trajectory();

  const double scale = (nominal_total > 1e-6) ? total_time / nominal_total : 1.0;
  Trajectory traj;
  traj.pos_pieces.reserve(prims.size());
  traj.yaw_pieces.reserve(prims.size());

  for (const Primitive& prim : prims) {
    const double T = std::max(0.05, prim.duration * scale);

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = prim.p0;
    pp.coeffs.col(1).setZero();
    pp.coeffs.col(2).setZero();
    if (prim.translate) {
      const Eigen::Vector2d delta = prim.p1 - prim.p0;
      pp.coeffs.col(3) = delta * (10.0 / (T * T * T));
      pp.coeffs.col(4) = delta * (-15.0 / (T * T * T * T));
      pp.coeffs.col(5) = delta * (6.0 / (T * T * T * T * T));
    } else {
      pp.coeffs.col(3).setZero();
      pp.coeffs.col(4).setZero();
      pp.coeffs.col(5).setZero();
    }
    traj.pos_pieces.push_back(pp);

    YawPolyPiece yp;
    yp.duration = T;
    const double yaw_delta = normalizeAngle(prim.yaw1 - prim.yaw0);
    yp.coeffs(0, 0) = prim.yaw0;
    yp.coeffs(0, 1) = 0.0;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 10.0 * yaw_delta / (T * T * T);
    yp.coeffs(0, 4) = -15.0 * yaw_delta / (T * T * T * T);
    yp.coeffs(0, 5) = 6.0 * yaw_delta / (T * T * T * T * T);
    traj.yaw_pieces.push_back(yp);
  }

  traj = retimeToDynamicLimits(traj);
  if (svsdf_ && !traj.empty() && svsdf_->evaluateTrajectory(traj) < 0.0) {
    return Trajectory();
  }
  return traj;
}

}  // namespace esv_planner
