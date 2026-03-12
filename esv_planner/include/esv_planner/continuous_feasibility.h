#pragma once

#include <esv_planner/common.h>
#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/continuous_feasibility_checker.h>
#include <esv_planner/unified_continuous_evaluator.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/collision_checker.h>

#include <cmath>
#include <memory>

namespace esv_planner {

class GridContinuousFeasibilityChecker final : public ContinuousFeasibilityChecker {
public:
  GridContinuousFeasibilityChecker(const GridMap& map,
                                   const CollisionChecker& checker,
                                   double discretization_step)
      : map_(&map), checker_(&checker), disc_step_(discretization_step) {
    evaluator_.initGridEsdf(map, checker.footprint());
  }

  GridContinuousFeasibilityChecker(
      std::shared_ptr<const ContinuousCollisionEvaluator> evaluator,
      double required_clearance,
      double discretization_step)
      : external_evaluator_(std::move(evaluator)),
        required_clearance_(required_clearance),
        disc_step_(discretization_step) {}

  double requiredClearance() const override {
    if (map_ != nullptr) {
      return std::max(required_clearance_, map_->resolution());
    }
    return required_clearance_;
  }

  double stateClearance(const SE2State& state) const override {
    if (external_evaluator_) {
      return external_evaluator_->evaluate(state);
    }
    return evaluator_.evaluate(state);
  }

  double transitionClearance(const SE2State& from,
                             const SE2State& to) const override {
    const Eigen::Vector2d delta = to.position() - from.position();
    const double len = delta.norm();
    const double yaw_delta = std::abs(normalizeAngle(to.yaw - from.yaw));
    const double resolution = map_ != nullptr ? map_->resolution() : disc_step_;
    const int coarse_steps = std::max(
        1, static_cast<int>(std::ceil(len / std::max(disc_step_, resolution))));
    const int yaw_steps = std::max(
        1, static_cast<int>(std::ceil(yaw_delta / 0.10)));
    const int n_steps = std::max(coarse_steps, yaw_steps);
    double min_clearance = kInf;
    for (int i = 0; i <= n_steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n_steps);
      SE2State sample;
      sample.x = from.x + delta.x() * t;
      sample.y = from.y + delta.y() * t;
      sample.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
      min_clearance = std::min(min_clearance, stateClearance(sample));
    }
    return min_clearance;
  }

  double segmentClearance(
      const std::vector<SE2State>& waypoints) const override {
    if (waypoints.size() < 2) return -kInf;

    auto fineTransitionClearance = [&](const SE2State& from,
                                       const SE2State& to) {
      const Eigen::Vector2d delta = to.position() - from.position();
      const double len = delta.norm();
      const double yaw_delta = std::abs(normalizeAngle(to.yaw - from.yaw));
      const double resolution = map_ != nullptr ? map_->resolution() : disc_step_;
      const double sample_step = std::max(0.5 * resolution, 1e-3);
      const int linear_steps = std::max(
          1, static_cast<int>(std::ceil(len / sample_step)));
      const int yaw_steps = std::max(
          1, static_cast<int>(std::ceil(yaw_delta / 0.10)));
      const int n_steps = std::max(linear_steps, yaw_steps);
      double min_clearance = kInf;
      for (int i = 0; i <= n_steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n_steps);
        SE2State sample;
        sample.x = from.x + delta.x() * t;
        sample.y = from.y + delta.y() * t;
        sample.yaw =
            normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
        min_clearance = std::min(min_clearance, stateClearance(sample));
      }
      return min_clearance;
    };

    double min_clearance = kInf;
    for (const auto& state : waypoints) {
      min_clearance = std::min(min_clearance, stateClearance(state));
    }
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      min_clearance = std::min(
          min_clearance, fineTransitionClearance(waypoints[i], waypoints[i + 1]));
    }
    return min_clearance;
  }

  bool safeYaw(SE2State& state, double desired_yaw) const override {
    if (checker_ == nullptr) {
      state.yaw = desired_yaw;
      return true;
    }

    const int bins = checker_->numYawBins();
    const auto wrap_bin = [bins](int bin) {
      int wrapped = bin % bins;
      return wrapped < 0 ? wrapped + bins : wrapped;
    };

    int desired_bin = checker_->binFromYaw(desired_yaw);
    if (checker_->isYawSafe(state.x, state.y, desired_bin)) {
      state.yaw = checker_->yawFromBin(desired_bin);
      return true;
    }

    for (int offset = 1; offset <= bins / 2; ++offset) {
      for (int sign : {-1, 1}) {
        const int bin = wrap_bin(desired_bin + sign * offset);
        if (checker_->isYawSafe(state.x, state.y, bin)) {
          state.yaw = checker_->yawFromBin(bin);
          return true;
        }
      }
    }

    return false;
  }

private:
  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  std::shared_ptr<const ContinuousCollisionEvaluator> external_evaluator_;
  UnifiedContinuousEvaluator evaluator_;
  double required_clearance_ = 0.10;
  double disc_step_ = 0.15;
};

}  // namespace esv_planner
