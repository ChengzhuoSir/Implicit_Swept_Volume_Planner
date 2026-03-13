#include "esv_planner/path_sparsifier.h"

#include <algorithm>
#include <cmath>

namespace esv_planner {

void PathSparsifier::init(const GridMap& map,
                          double line_clearance,
                          double max_segment_length,
                          int max_support_points) {
  map_ = &map;
  line_clearance_ = line_clearance;
  max_segment_length_ = max_segment_length;
  max_support_points_ = std::max(2, max_support_points);
}

std::vector<SE2State> PathSparsifier::sparsify(const std::vector<Eigen::Vector2d>& dense_path,
                                               const SE2State& start,
                                               const SE2State& goal) const {
  std::vector<SE2State> sparse;
  if (!map_ || dense_path.size() < 2) {
    return sparse;
  }

  std::vector<Eigen::Vector2d> key_points;
  key_points.push_back(start.position());
  size_t anchor = 0;
  while (anchor + 1 < dense_path.size()) {
    size_t best = anchor + 1;
    for (size_t candidate = best; candidate < dense_path.size(); ++candidate) {
      const double seg_len = (dense_path[candidate] - dense_path[anchor]).norm();
      if (seg_len > max_segment_length_) {
        break;
      }
      if (segmentSafe(dense_path[anchor], dense_path[candidate])) {
        best = candidate;
      }
    }
    key_points.push_back(dense_path[best]);
    anchor = best;
  }

  if ((key_points.back() - goal.position()).norm() > 1e-6) {
    key_points.back() = goal.position();
  }

  if (static_cast<int>(key_points.size()) > max_support_points_) {
    std::vector<Eigen::Vector2d> capped;
    capped.reserve(static_cast<size_t>(max_support_points_));
    const double last_index = static_cast<double>(key_points.size() - 1);
    const double denom = static_cast<double>(max_support_points_ - 1);
    for (int i = 0; i < max_support_points_; ++i) {
      const size_t idx = static_cast<size_t>(
          std::lround(last_index * static_cast<double>(i) / denom));
      capped.push_back(key_points[std::min(idx, key_points.size() - 1)]);
    }
    key_points.swap(capped);
  }

  sparse.reserve(key_points.size());
  for (size_t i = 0; i < key_points.size(); ++i) {
    sparse.push_back(SE2State(key_points[i].x(), key_points[i].y(), 0.0));
  }
  assignYaw(&sparse, start, goal);
  return sparse;
}

bool PathSparsifier::segmentSafe(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  const double distance = (b - a).norm();
  const double step = std::max(0.25 * map_->resolution(), 0.02);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector2d p = a + ratio * (b - a);
    if (map_->getEsdf(p.x(), p.y()) <= line_clearance_) {
      return false;
    }
  }
  return true;
}

void PathSparsifier::assignYaw(std::vector<SE2State>* states,
                               const SE2State& start,
                               const SE2State& goal) const {
  if (!states || states->empty()) {
    return;
  }
  (*states)[0].yaw = start.yaw;
  for (size_t i = 1; i + 1 < states->size(); ++i) {
    const Eigen::Vector2d delta =
        (*states)[i + 1].position() - (*states)[i - 1].position();
    if (delta.norm() > 1e-6) {
      (*states)[i].yaw = std::atan2(delta.y(), delta.x());
    } else {
      (*states)[i].yaw = (*states)[i - 1].yaw;
    }
  }
  (*states)[states->size() - 1].yaw = goal.yaw;
}

}  // namespace esv_planner
