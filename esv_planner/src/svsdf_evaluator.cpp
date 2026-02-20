#include "esv_planner/svsdf_evaluator.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace esv_planner {

SvsdfEvaluator::SvsdfEvaluator() {}

void SvsdfEvaluator::init(const GridMap& map, const FootprintModel& footprint) {
  map_ = &map;
  footprint_ = &footprint;
}

// Ray-casting point-in-polygon test.
// Returns true if point (px, py) lies inside the polygon defined by verts.
static bool pointInPolygon(double px, double py,
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

double SvsdfEvaluator::evaluate(const SE2State& state) const {
  auto rotated = footprint_->rotatedVertices(state.yaw);
  const double res = map_->resolution();
  const int n = static_cast<int>(rotated.size());

  double min_dist = kInf;

  // Helper: query ESDF at a body-frame offset point
  auto queryPoint = [&](double lx, double ly) {
    double wx = state.x + lx;
    double wy = state.y + ly;
    double d = map_->getEsdf(wx, wy);
    if (d < min_dist) min_dist = d;
  };

  // --- 1. Densely sample the polygon boundary at resolution-level spacing ---
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    const Eigen::Vector2d& p0 = rotated[i];
    const Eigen::Vector2d& p1 = rotated[j];
    Eigen::Vector2d edge = p1 - p0;
    double edge_len = edge.norm();
    if (edge_len < 1e-12) {
      queryPoint(p0.x(), p0.y());
      continue;
    }
    // Number of samples along this edge (at least the two endpoints)
    int num_samples = std::max(2, static_cast<int>(std::ceil(edge_len / res)) + 1);
    for (int s = 0; s < num_samples; ++s) {
      double t = static_cast<double>(s) / static_cast<double>(num_samples - 1);
      Eigen::Vector2d pt = p0 + t * edge;
      queryPoint(pt.x(), pt.y());
    }
  }

  // --- 2. Sample interior points on a grid within the polygon's bounding box ---
  double interior_step = 2.0 * res;

  // Compute axis-aligned bounding box of the rotated polygon (body frame)
  double bb_min_x = std::numeric_limits<double>::max();
  double bb_max_x = std::numeric_limits<double>::lowest();
  double bb_min_y = std::numeric_limits<double>::max();
  double bb_max_y = std::numeric_limits<double>::lowest();
  for (int i = 0; i < n; ++i) {
    bb_min_x = std::min(bb_min_x, rotated[i].x());
    bb_max_x = std::max(bb_max_x, rotated[i].x());
    bb_min_y = std::min(bb_min_y, rotated[i].y());
    bb_max_y = std::max(bb_max_y, rotated[i].y());
  }

  // Iterate over interior grid points and check if inside polygon via ray casting
  for (double gx = bb_min_x + interior_step * 0.5; gx <= bb_max_x; gx += interior_step) {
    for (double gy = bb_min_y + interior_step * 0.5; gy <= bb_max_y; gy += interior_step) {
      if (pointInPolygon(gx, gy, rotated)) {
        queryPoint(gx, gy);
      }
    }
  }

  // --- 3. Always sample the centroid (robot center) ---
  queryPoint(0.0, 0.0);

  return min_dist;
}

double SvsdfEvaluator::evaluateTrajectory(const Trajectory& traj, double dt) const {
  double min_dist = kInf;
  double total = traj.totalDuration();

  // Collect time samples at regular dt intervals
  std::vector<double> times;
  times.reserve(static_cast<size_t>(total / dt) + 2);
  for (double t = 0.0; t <= total; t += dt) {
    times.push_back(t);
  }
  // Ensure the final time is included
  if (times.empty() || times.back() < total - 1e-9) {
    times.push_back(total);
  }

  // Evaluate at each sample and also at midpoints between consecutive samples
  // to catch fast-moving collisions
  for (size_t i = 0; i < times.size(); ++i) {
    SE2State st = traj.sample(times[i]);
    double d = evaluate(st);
    min_dist = std::min(min_dist, d);

    if (i + 1 < times.size()) {
      double t_mid = 0.5 * (times[i] + times[i + 1]);
      SE2State st_mid = traj.sample(t_mid);
      double d_mid = evaluate(st_mid);
      min_dist = std::min(min_dist, d_mid);
    }
  }

  return min_dist;
}

void SvsdfEvaluator::gradient(const SE2State& state,
                               Eigen::Vector2d& grad_pos, double& grad_yaw) const {
  double eps_pos = map_->resolution() * 0.25;
  double eps_yaw = 0.005;

  // Central finite differences for position gradient
  SE2State sx_plus = state;  sx_plus.x += eps_pos;
  SE2State sx_minus = state; sx_minus.x -= eps_pos;
  grad_pos.x() = (evaluate(sx_plus) - evaluate(sx_minus)) / (2.0 * eps_pos);

  SE2State sy_plus = state;  sy_plus.y += eps_pos;
  SE2State sy_minus = state; sy_minus.y -= eps_pos;
  grad_pos.y() = (evaluate(sy_plus) - evaluate(sy_minus)) / (2.0 * eps_pos);

  // Central finite difference for yaw gradient
  SE2State syaw_plus = state;  syaw_plus.yaw += eps_yaw;
  SE2State syaw_minus = state; syaw_minus.yaw -= eps_yaw;
  grad_yaw = (evaluate(syaw_plus) - evaluate(syaw_minus)) / (2.0 * eps_yaw);
}

}  // namespace esv_planner
