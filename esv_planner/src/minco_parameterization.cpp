#include <esv_planner/minco_parameterization.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace esv_planner {

namespace {

std::vector<double> unwrapYawSequence(const std::vector<SE2State>& states) {
  std::vector<double> yaws;
  yaws.reserve(states.size());
  if (states.empty()) return yaws;

  yaws.push_back(states.front().yaw);
  for (size_t i = 1; i < states.size(); ++i) {
    const double delta = normalizeAngle(states[i].yaw - states[i - 1].yaw);
    yaws.push_back(yaws.back() + delta);
  }
  return yaws;
}

}  // namespace

MincoDecisionVariables MincoParameterization::packSe2DecisionVariables(
    const std::vector<SE2State>& support_states,
    const std::vector<double>& durations) {
  MincoDecisionVariables vars;
  if (support_states.size() < 2 || durations.size() + 1 != support_states.size()) {
    return vars;
  }

  vars.piece_count = static_cast<int>(durations.size());
  const int interior_count = static_cast<int>(support_states.size()) - 2;
  vars.values = Eigen::VectorXd::Zero(3 * interior_count + vars.piece_count);

  int cursor = 0;
  for (int i = 1; i + 1 < static_cast<int>(support_states.size()); ++i) {
    vars.values(cursor++) = support_states[i].x;
    vars.values(cursor++) = support_states[i].y;
    vars.values(cursor++) = support_states[i].yaw;
  }

  for (double duration : durations) {
    vars.values(cursor++) = std::log(std::max(1e-3, duration));
  }

  return vars;
}

bool MincoParameterization::unpackSe2DecisionVariables(
    const MincoDecisionVariables& vars,
    const SE2State& start,
    const SE2State& goal,
    std::vector<SE2State>* support_states,
    std::vector<double>* durations) {
  if (!support_states || !durations || vars.piece_count <= 0) {
    return false;
  }

  const int expected_dim = 4 * vars.piece_count - 3;
  if (vars.values.size() != expected_dim) {
    return false;
  }

  const int interior_count = vars.piece_count - 1;
  support_states->clear();
  support_states->reserve(static_cast<size_t>(vars.piece_count + 1));
  support_states->push_back(start);

  int cursor = 0;
  for (int i = 0; i < interior_count; ++i) {
    SE2State state;
    state.x = vars.values(cursor++);
    state.y = vars.values(cursor++);
    state.yaw = vars.values(cursor++);
    state.normalizeYaw();
    support_states->push_back(state);
  }
  support_states->push_back(goal);

  durations->clear();
  durations->reserve(static_cast<size_t>(vars.piece_count));
  for (int i = 0; i < vars.piece_count; ++i) {
    durations->push_back(std::max(1e-3, std::exp(vars.values(cursor++))));
  }

  return true;
}

Trajectory MincoParameterization::buildSe2Trajectory(
    const MincoDecisionVariables& vars,
    const SE2State& start,
    const SE2State& goal) {
  std::vector<SE2State> support_states;
  std::vector<double> durations;
  if (!unpackSe2DecisionVariables(vars, start, goal, &support_states, &durations)) {
    return {};
  }

  std::vector<Eigen::Vector2d> positions;
  positions.reserve(support_states.size());
  for (const auto& state : support_states) {
    positions.push_back(state.position());
  }

  const std::vector<double> yaws = unwrapYawSequence(support_states);
  const Eigen::Vector2d first_delta = positions[1] - positions[0];
  const Eigen::Vector2d last_delta =
      positions.back() - positions[positions.size() - 2];
  const double first_dt = std::max(0.05, durations.front());
  const double last_dt = std::max(0.05, durations.back());
  const Eigen::Vector2d v0 = first_delta / first_dt;
  const Eigen::Vector2d vf = last_delta / last_dt;
  const Eigen::Vector2d a0 = Eigen::Vector2d::Zero();
  const Eigen::Vector2d af = Eigen::Vector2d::Zero();
  const double omega0 = normalizeAngle(yaws[1] - yaws[0]) / first_dt;
  const double omegaf =
      normalizeAngle(yaws.back() - yaws[yaws.size() - 2]) / last_dt;

  Trajectory traj;
  fitPositionQuintic(positions, durations, v0, vf, a0, af, &traj.pos_pieces);
  fitYawQuintic(yaws, durations, omega0, omegaf, &traj.yaw_pieces);
  return traj;
}

void MincoParameterization::fitPositionQuintic(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& durations,
    const Eigen::Vector2d& v0,
    const Eigen::Vector2d& vf,
    const Eigen::Vector2d& a0,
    const Eigen::Vector2d& af,
    std::vector<PolyPiece>* pieces) {
  if (!pieces) return;
  const int piece_count = static_cast<int>(positions.size()) - 1;
  if (piece_count <= 0 || durations.size() != static_cast<size_t>(piece_count)) {
    pieces->clear();
    return;
  }

  std::vector<Eigen::Vector2d> vels(static_cast<size_t>(piece_count + 1),
                                    Eigen::Vector2d::Zero());
  std::vector<Eigen::Vector2d> accs(static_cast<size_t>(piece_count + 1),
                                    Eigen::Vector2d::Zero());
  vels.front() = v0;
  vels.back() = vf;
  accs.front() = a0;
  accs.back() = af;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[static_cast<size_t>(i - 1)] +
                    durations[static_cast<size_t>(i)];
    dt_sum = std::max(1e-6, dt_sum);
    vels[static_cast<size_t>(i)] =
        (positions[static_cast<size_t>(i + 1)] -
         positions[static_cast<size_t>(i - 1)]) /
        dt_sum;
  }

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[static_cast<size_t>(i - 1)] +
                    durations[static_cast<size_t>(i)];
    dt_sum = std::max(1e-6, dt_sum);
    accs[static_cast<size_t>(i)] =
        (vels[static_cast<size_t>(std::min(i + 1, piece_count))] -
         vels[static_cast<size_t>(std::max(i - 1, 0))]) /
        dt_sum;
  }

  pieces->resize(static_cast<size_t>(piece_count));
  for (int k = 0; k < piece_count; ++k) {
    const double T = durations[static_cast<size_t>(k)];
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, T, T2, T3,
        T4, T5, 0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4, 0, 0, 2, 6 * T,
        12 * T2, 20 * T3;

    const auto qr = A.colPivHouseholderQr();
    PolyPiece piece;
    piece.duration = T;
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Matrix<double, 6, 1> b;
      b(0) = positions[static_cast<size_t>(k)](axis);
      b(1) = vels[static_cast<size_t>(k)](axis);
      b(2) = accs[static_cast<size_t>(k)](axis);
      b(3) = positions[static_cast<size_t>(k + 1)](axis);
      b(4) = vels[static_cast<size_t>(k + 1)](axis);
      b(5) = accs[static_cast<size_t>(k + 1)](axis);
      const auto c = qr.solve(b);
      for (int j = 0; j < 6; ++j) {
        piece.coeffs(axis, j) = std::isfinite(c(j)) ? c(j) : 0.0;
      }
    }
    (*pieces)[static_cast<size_t>(k)] = piece;
  }
}

void MincoParameterization::fitYawQuintic(
    const std::vector<double>& yaws,
    const std::vector<double>& durations,
    double omega0,
    double omegaf,
    std::vector<YawPolyPiece>* pieces) {
  if (!pieces) return;
  const int piece_count = static_cast<int>(yaws.size()) - 1;
  if (piece_count <= 0 || durations.size() != static_cast<size_t>(piece_count)) {
    pieces->clear();
    return;
  }

  std::vector<double> omegas(static_cast<size_t>(piece_count + 1), 0.0);
  std::vector<double> alphas(static_cast<size_t>(piece_count + 1), 0.0);
  omegas.front() = omega0;
  omegas.back() = omegaf;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[static_cast<size_t>(i - 1)] +
                    durations[static_cast<size_t>(i)];
    dt_sum = std::max(1e-6, dt_sum);
    omegas[static_cast<size_t>(i)] =
        normalizeAngle(yaws[static_cast<size_t>(i + 1)] -
                       yaws[static_cast<size_t>(i - 1)]) /
        dt_sum;
  }

  pieces->resize(static_cast<size_t>(piece_count));
  for (int k = 0; k < piece_count; ++k) {
    const double T = durations[static_cast<size_t>(k)];
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    YawPolyPiece piece;
    piece.duration = T;
    piece.coeffs(0, 0) = yaws[static_cast<size_t>(k)];
    piece.coeffs(0, 1) = omegas[static_cast<size_t>(k)];
    piece.coeffs(0, 2) = 0.5 * alphas[static_cast<size_t>(k)];

    Eigen::Matrix3d A;
    A << T3, T4, T5,
         3.0 * T2, 4.0 * T3, 5.0 * T4,
         6.0 * T, 12.0 * T2, 20.0 * T3;
    Eigen::Vector3d rhs;
    rhs(0) = yaws[static_cast<size_t>(k + 1)] -
             (piece.coeffs(0, 0) + piece.coeffs(0, 1) * T +
              piece.coeffs(0, 2) * T2);
    rhs(1) = omegas[static_cast<size_t>(k + 1)] -
             (piece.coeffs(0, 1) + 2.0 * piece.coeffs(0, 2) * T);
    rhs(2) = alphas[static_cast<size_t>(k + 1)] -
             (2.0 * piece.coeffs(0, 2));

    const Eigen::Vector3d tail = A.colPivHouseholderQr().solve(rhs);
    piece.coeffs(0, 3) = std::isfinite(tail(0)) ? tail(0) : 0.0;
    piece.coeffs(0, 4) = std::isfinite(tail(1)) ? tail(1) : 0.0;
    piece.coeffs(0, 5) = std::isfinite(tail(2)) ? tail(2) : 0.0;
    (*pieces)[static_cast<size_t>(k)] = piece;
  }
}

}  // namespace esv_planner
