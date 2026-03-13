#include "esv_planner/paper_backend.h"

#include <algorithm>
#include <cmath>

namespace esv_planner {

void PaperBackend::init(const GridMap& map,
                        const FootprintModel& footprint,
                        const SvsdfEvaluator& svsdf,
                        const PaperBackendParams& params) {
  map_ = &map;
  footprint_ = &footprint;
  svsdf_ = &svsdf;
  params_ = params;

  TrueSvsdfPenaltyParams penalty_params;
  penalty_params.safety_margin = params_.safety_margin;
  penalty_params.obstacle_weight = params_.obstacle_weight;
  penalty_params.obstacle_cutoff_distance = params_.obstacle_cutoff_distance;
  penalty_params.sample_time_step = params_.sample_time_step;
  penalty_.init(footprint, penalty_params);
}

PaperBackendResult PaperBackend::optimize(
    const std::vector<SE2State>& reference_support,
    const std::vector<Eigen::Vector2d>& local_obstacles) const {
  PaperBackendResult result;
  result.support_states = reference_support;
  if (!map_ || !footprint_ || !svsdf_ || reference_support.size() < 2) {
    return result;
  }

  Eigen::VectorXd vars = packInteriorPositions(reference_support);
  CostEvaluation current = evaluate(vars, reference_support, local_obstacles);
  if (vars.size() == 0) {
    result.success = !current.trajectory.empty() && current.min_clearance >= 0.0;
    result.trajectory = current.trajectory;
    result.support_states = current.states;
    result.final_cost = current.cost;
    result.min_clearance = current.min_clearance;
    return result;
  }

  const LmbmSolver::EvaluateFn objective =
      [this, &reference_support, &local_obstacles](
          const Eigen::VectorXd& x,
          Eigen::VectorXd* grad) -> double {
        const CostEvaluation eval = evaluate(x, reference_support, local_obstacles);
        if (grad) {
          *grad = eval.gradient;
        }
        return eval.cost;
      };
  const LmbmSolver::Result solve_result =
      solver_.optimize(params_.lmbm, objective, &vars);

  current = evaluate(vars, reference_support, local_obstacles);
  result.iterations = solve_result.iterations;
  result.final_cost = solve_result.final_cost;
  result.trajectory = current.trajectory;
  result.support_states = current.states;
  result.min_clearance = current.min_clearance;
  result.success = !current.trajectory.empty() && current.min_clearance >= 0.0;
  return result;
}

std::vector<double> PaperBackend::allocateDurations(
    const std::vector<SE2State>& states) const {
  std::vector<double> durations;
  if (states.size() < 2) {
    return durations;
  }

  durations.reserve(states.size() - 1);
  for (size_t i = 1; i < states.size(); ++i) {
    const double distance =
        (states[i].position() - states[i - 1].position()).norm();
    durations.push_back(std::max(
        params_.min_piece_duration,
        distance / std::max(0.1, params_.nominal_speed)));
  }
  return durations;
}

PaperBackend::CostEvaluation PaperBackend::evaluate(
    const Eigen::VectorXd& vars,
    const std::vector<SE2State>& reference_support,
    const std::vector<Eigen::Vector2d>& local_obstacles) const {
  CostEvaluation evaluation;
  evaluation.states = unpackStates(vars, reference_support);
  evaluation.gradient = Eigen::VectorXd::Zero(vars.size());
  if (evaluation.states.size() != reference_support.size()) {
    return evaluation;
  }

  std::vector<Eigen::Vector2d> state_grad(
      evaluation.states.size(), Eigen::Vector2d::Zero());
  double cost = 0.0;

  for (size_t i = 1; i + 1 < evaluation.states.size(); ++i) {
    const Eigen::Vector2d delta =
        evaluation.states[i].position() - reference_support[i].position();
    cost += params_.anchor_weight * delta.squaredNorm();
    state_grad[i] += 2.0 * params_.anchor_weight * delta;
  }

  for (size_t i = 1; i + 1 < evaluation.states.size(); ++i) {
    const Eigen::Vector2d smooth =
        evaluation.states[i + 1].position() -
        2.0 * evaluation.states[i].position() +
        evaluation.states[i - 1].position();
    cost += params_.smooth_weight * smooth.squaredNorm();
    state_grad[i - 1] += 2.0 * params_.smooth_weight * smooth;
    state_grad[i] += -4.0 * params_.smooth_weight * smooth;
    state_grad[i + 1] += 2.0 * params_.smooth_weight * smooth;
  }

  const std::vector<double> durations = allocateDurations(evaluation.states);
  const Eigen::VectorXd se2_vars =
      MincoParameterization::packSe2StateVariables(evaluation.states);
  evaluation.trajectory = MincoParameterization::buildSe2TrajectoryFixedTime(
      se2_vars, durations, evaluation.states.front(), evaluation.states.back());
  if (evaluation.trajectory.empty()) {
    return evaluation;
  }

  const TrueSvsdfPenalty::Evaluation penalty_eval =
      penalty_.evaluate(evaluation.trajectory,
                        static_cast<int>(evaluation.states.size()),
                        local_obstacles);
  cost += penalty_eval.cost;
  evaluation.min_clearance = penalty_eval.min_clearance;

  for (size_t i = 1; i + 1 < evaluation.states.size(); ++i) {
    evaluation.gradient(2 * static_cast<Eigen::Index>(i - 1)) =
        state_grad[i].x() + penalty_eval.gradient(2 * static_cast<Eigen::Index>(i - 1));
    evaluation.gradient(2 * static_cast<Eigen::Index>(i - 1) + 1) =
        state_grad[i].y() + penalty_eval.gradient(2 * static_cast<Eigen::Index>(i - 1) + 1);
  }

  evaluation.cost = cost;
  return evaluation;
}

std::vector<SE2State> PaperBackend::unpackStates(
    const Eigen::VectorXd& vars,
    const std::vector<SE2State>& reference_support) const {
  std::vector<SE2State> states = reference_support;
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    states[i].x = vars(2 * static_cast<Eigen::Index>(i - 1));
    states[i].y = vars(2 * static_cast<Eigen::Index>(i - 1) + 1);
  }

  if (!states.empty()) {
    states.front().yaw = reference_support.front().yaw;
    states.back().yaw = reference_support.back().yaw;
  }
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    const Eigen::Vector2d delta =
        states[i + 1].position() - states[i - 1].position();
    states[i].yaw =
        delta.norm() > 1e-6 ? std::atan2(delta.y(), delta.x()) : states[i - 1].yaw;
  }
  return states;
}

Eigen::VectorXd PaperBackend::packInteriorPositions(
    const std::vector<SE2State>& states) const {
  if (states.size() < 3) {
    return Eigen::VectorXd();
  }

  Eigen::VectorXd vars = Eigen::VectorXd::Zero(
      2 * static_cast<Eigen::Index>(states.size() - 2));
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    vars(2 * static_cast<Eigen::Index>(i - 1)) = states[i].x;
    vars(2 * static_cast<Eigen::Index>(i - 1) + 1) = states[i].y;
  }
  return vars;
}

}  // namespace esv_planner
