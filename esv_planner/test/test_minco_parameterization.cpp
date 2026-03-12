#include <esv_planner/minco_parameterization.h>

#include <cmath>
#include <iostream>

namespace {

bool nearVec(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double tol) {
  return (a - b).norm() <= tol;
}

}  // namespace

int main() {
  using namespace esv_planner;

  const std::vector<SE2State> support_states = {
      {0.0, 0.0, 0.0},
      {1.0, 0.3, 0.15},
      {2.1, 0.8, 0.25},
      {3.0, 1.1, 0.35},
  };
  const std::vector<double> durations = {0.8, 1.0, 1.2};

  const auto vars =
      MincoParameterization::packSe2DecisionVariables(support_states, durations);

  const int expected_dim =
      static_cast<int>(3 * (support_states.size() - 2) + durations.size());
  if (vars.values.size() != expected_dim) {
    std::cerr << "unexpected_dim=" << vars.values.size()
              << " expected_dim=" << expected_dim << std::endl;
    return 1;
  }

  if (vars.piece_count != static_cast<int>(durations.size())) {
    std::cerr << "unexpected_piece_count=" << vars.piece_count << std::endl;
    return 1;
  }

  std::vector<SE2State> unpacked_states;
  std::vector<double> unpacked_durations;
  if (!MincoParameterization::unpackSe2DecisionVariables(
          vars, support_states.front(), support_states.back(), &unpacked_states,
          &unpacked_durations)) {
    std::cerr << "unpack_failed" << std::endl;
    return 1;
  }

  if (unpacked_states.size() != support_states.size()) {
    std::cerr << "unexpected_state_count=" << unpacked_states.size() << std::endl;
    return 1;
  }

  if (unpacked_durations.size() != durations.size()) {
    std::cerr << "unexpected_duration_count=" << unpacked_durations.size()
              << std::endl;
    return 1;
  }

  const Trajectory traj = MincoParameterization::buildSe2Trajectory(
      vars, support_states.front(), support_states.back());
  if (traj.empty()) {
    std::cerr << "trajectory_empty" << std::endl;
    return 1;
  }

  if (traj.pos_pieces.size() != durations.size() ||
      traj.yaw_pieces.size() != durations.size()) {
    std::cerr << "unexpected_piece_vector_sizes" << std::endl;
    return 1;
  }

  for (size_t i = 0; i + 1 < traj.pos_pieces.size(); ++i) {
    const auto& left_pos = traj.pos_pieces[i];
    const auto& right_pos = traj.pos_pieces[i + 1];
    const auto& left_yaw = traj.yaw_pieces[i];
    const auto& right_yaw = traj.yaw_pieces[i + 1];

    if (!nearVec(left_pos.evaluate(left_pos.duration), right_pos.evaluate(0.0),
                 1e-6)) {
      std::cerr << "position_c0_break piece=" << i << std::endl;
      return 1;
    }
    if (!nearVec(left_pos.velocity(left_pos.duration), right_pos.velocity(0.0),
                 1e-6)) {
      std::cerr << "position_c1_break piece=" << i << std::endl;
      return 1;
    }
    if (!nearVec(left_pos.acceleration(left_pos.duration),
                 right_pos.acceleration(0.0), 1e-5)) {
      std::cerr << "position_c2_break piece=" << i << std::endl;
      return 1;
    }

    const double yaw_gap =
        std::abs(normalizeAngle(left_yaw.evaluate(left_yaw.duration) -
                                right_yaw.evaluate(0.0)));
    if (yaw_gap > 1e-6) {
      std::cerr << "yaw_c0_break piece=" << i
                << " left=" << left_yaw.evaluate(left_yaw.duration)
                << " right=" << right_yaw.evaluate(0.0)
                << " gap=" << yaw_gap
                << std::endl;
      return 1;
    }

    const double yaw_rate_gap = std::abs(left_yaw.velocity(left_yaw.duration) -
                                         right_yaw.velocity(0.0));
    if (yaw_rate_gap > 1e-5) {
      std::cerr << "yaw_c1_break piece=" << i << " gap=" << yaw_rate_gap
                << std::endl;
      return 1;
    }
  }

  std::cout << "piece_count=" << vars.piece_count
            << " variable_dim=" << vars.values.size() << std::endl;
  return 0;
}
