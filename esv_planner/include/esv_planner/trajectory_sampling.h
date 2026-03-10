#pragma once

#include <esv_planner/common.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace esv_planner {

inline std::vector<SE2State> sampleTrajectoryUniformTime(
    const Trajectory& traj, double time_step) {
  std::vector<SE2State> states;
  if (traj.empty()) return states;

  const double total = traj.totalDuration();
  const double dt = std::max(1e-3, time_step);
  const int n_steps =
      std::max(1, static_cast<int>(std::ceil(total / dt)));
  states.reserve(static_cast<size_t>(n_steps) + 1);
  for (int i = 0; i <= n_steps; ++i) {
    const double t = (i == n_steps) ? total : std::min(total, i * dt);
    states.push_back(traj.sample(t));
  }
  return states;
}

inline std::vector<SE2State> resampleStatesByArcLength(
    const std::vector<SE2State>& dense_states, double spatial_step) {
  std::vector<SE2State> samples;
  if (dense_states.empty()) return samples;

  const double ds = std::max(1e-4, spatial_step);
  samples.reserve(dense_states.size());
  samples.push_back(dense_states.front());

  double carried = 0.0;
  SE2State prev = dense_states.front();
  for (size_t i = 1; i < dense_states.size(); ++i) {
    const SE2State curr = dense_states[i];
    Eigen::Vector2d seg = curr.position() - prev.position();
    double seg_len = seg.norm();
    if (seg_len < 1e-9) {
      prev = curr;
      continue;
    }

    while (carried + seg_len >= ds) {
      const double remain = ds - carried;
      const double ratio = remain / seg_len;
      SE2State st;
      st.x = prev.x + (curr.x - prev.x) * ratio;
      st.y = prev.y + (curr.y - prev.y) * ratio;
      st.yaw = normalizeAngle(
          prev.yaw + normalizeAngle(curr.yaw - prev.yaw) * ratio);
      samples.push_back(st);
      prev = st;
      seg = curr.position() - prev.position();
      seg_len = seg.norm();
      if (seg_len < 1e-9) {
        carried = 0.0;
        break;
      }
      carried = 0.0;
    }

    carried += seg_len;
    prev = curr;
  }

  const auto& last_dense = dense_states.back();
  if ((last_dense.position() - samples.back().position()).norm() > 1e-6 ||
      std::abs(normalizeAngle(last_dense.yaw - samples.back().yaw)) > 1e-4) {
    samples.push_back(last_dense);
  }
  return samples;
}

inline std::vector<SE2State> sampleTrajectoryByArcLength(
    const Trajectory& traj,
    double spatial_step,
    double dense_time_step = 0.02) {
  return resampleStatesByArcLength(
      sampleTrajectoryUniformTime(traj, dense_time_step), spatial_step);
}

}  // namespace esv_planner
