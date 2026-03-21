#include "esv_planner/core/math_utils.h"
#include <cmath>

namespace esv_planner {
namespace math {

Eigen::Vector2d EsdfAscentDirection(const GridMap& map, const Eigen::Vector2d& world, double eps) {
  const double ex_p = map.getEsdf(world.x() + eps, world.y());
  const double ex_n = map.getEsdf(world.x() - eps, world.y());
  const double ey_p = map.getEsdf(world.x(), world.y() + eps);
  const double ey_n = map.getEsdf(world.x(), world.y() - eps);
  Eigen::Vector2d gradient(ex_p - ex_n, ey_p - ey_n);
  if (gradient.norm() > 1e-9) {
    return gradient.normalized();
  }

  const double base = map.getEsdf(world.x(), world.y());
  Eigen::Vector2d best = Eigen::Vector2d::Zero();
  double best_gain = 0.0;
  for (int k = 0; k < 16; ++k) {
    const double theta = kTwoPi * static_cast<double>(k) / 16.0;
    const Eigen::Vector2d direction(std::cos(theta), std::sin(theta));
    for (double scale : {1.0, 2.0, 3.0}) {
      const double value =
          map.getEsdf(world.x() + scale * eps * direction.x(),
                      world.y() + scale * eps * direction.y());
      const double gain = value - base;
      if (gain > best_gain) {
        best_gain = gain;
        best = direction;
      }
    }
  }
  return best;
}

bool PushPointFromObstacle(const GridMap& map, Eigen::Vector2d& pos, 
                           double target_clearance, double step_size, int max_iters) {
  const double h = map.resolution();
  for (int iter = 0; iter < max_iters; ++iter) {
    const double esdf = map.getEsdf(pos.x(), pos.y());
    if (std::isfinite(esdf) && esdf >= target_clearance) {
      return true;
    }

    Eigen::Vector2d gradient = EsdfAscentDirection(map, pos, h);

    if (gradient.norm() > 1e-6) {
      pos += step_size * gradient;
    } else {
      bool found = false;
      for (double radius = h; radius <= 3.0; radius += h) {
        double best_esdf = -1.0;
        Eigen::Vector2d best_step = Eigen::Vector2d::Zero();
        for (int a = 0; a < 16; ++a) {
          const double angle = a * M_PI / 8.0;
          Eigen::Vector2d step(radius * std::cos(angle), radius * std::sin(angle));
          const double e = map.getEsdf(pos.x() + step.x(), pos.y() + step.y());
          if (std::isfinite(e) && e > best_esdf) {
            best_esdf = e;
            best_step = step;
          }
        }
        if (best_esdf >= target_clearance) {
          pos += best_step;
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
    }
  }
  
  const double final_esdf = map.getEsdf(pos.x(), pos.y());
  return std::isfinite(final_esdf) && final_esdf >= target_clearance;
}

} // namespace math
} // namespace esv_planner