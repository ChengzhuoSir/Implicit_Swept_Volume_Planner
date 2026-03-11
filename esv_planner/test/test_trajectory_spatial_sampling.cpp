#include <esv_planner/common.h>
#include <esv_planner/trajectory_sampling.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace esv_planner;

namespace {

PolyPiece makeLinearPiece(const Eigen::Vector2d& start,
                          const Eigen::Vector2d& end,
                          double duration) {
  PolyPiece piece;
  piece.duration = duration;
  piece.coeffs.col(0) = start;
  piece.coeffs.col(1) = (end - start) / duration;
  return piece;
}

YawPolyPiece makeConstantYawPiece(double yaw, double duration) {
  YawPolyPiece piece;
  piece.duration = duration;
  piece.coeffs(0, 0) = yaw;
  return piece;
}

std::vector<double> consecutiveDistances(const std::vector<SE2State>& states) {
  std::vector<double> dists;
  for (size_t i = 1; i < states.size(); ++i) {
    dists.push_back((states[i].position() - states[i - 1].position()).norm());
  }
  return dists;
}

double maxAbsDeviation(const std::vector<double>& values, double target) {
  double worst = 0.0;
  for (double v : values) {
    worst = std::max(worst, std::abs(v - target));
  }
  return worst;
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  Trajectory traj;
  traj.pos_pieces.push_back(
      makeLinearPiece(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0), 1.0));
  traj.pos_pieces.push_back(
      makeLinearPiece(Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(2.0, 0.0), 4.0));
  traj.yaw_pieces.push_back(makeConstantYawPiece(0.0, 1.0));
  traj.yaw_pieces.push_back(makeConstantYawPiece(0.0, 4.0));

  const double spatial_step = 0.2;
  const auto states = sampleTrajectoryByArcLength(traj, spatial_step);
  if (states.size() < 10) {
    std::cerr << "[test] FAIL: too few samples: " << states.size() << "\n";
    return 1;
  }

  const auto dists = consecutiveDistances(states);
  if (dists.empty()) {
    std::cerr << "[test] FAIL: empty spacing set\n";
    return 1;
  }

  const double max_dev = maxAbsDeviation(dists, spatial_step);
  const double final_x = states.back().x;
  std::cout << "[test] sample_count=" << states.size()
            << " max_spacing_deviation=" << max_dev
            << " final_x=" << final_x << "\n";

  if (std::abs(final_x - 2.0) > 1e-6) {
    std::cerr << "[test] FAIL: final sample mismatch\n";
    return 1;
  }
  if (max_dev > 0.03) {
    std::cerr << "[test] FAIL: spacing deviation too large\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
