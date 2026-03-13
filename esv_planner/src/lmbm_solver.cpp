#include "esv_planner/lmbm_solver.h"

#include "lmbm/lmbm.h"

#include <vector>

namespace esv_planner {

namespace {

struct LmbmContext {
  LmbmSolver::EvaluateFn evaluate;
  int iterations = 0;
};

double evaluateThunk(void* instance,
                     const double* x,
                     double* g,
                     const int n) {
  LmbmContext& context = *static_cast<LmbmContext*>(instance);
  Eigen::Map<const Eigen::VectorXd> vars(x, n);
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);
  const double cost = context.evaluate(vars, &grad);
  Eigen::Map<Eigen::VectorXd>(g, n) = grad;
  return cost;
}

int progressThunk(void* instance,
                  const double*,
                  const int k) {
  LmbmContext& context = *static_cast<LmbmContext*>(instance);
  context.iterations = k;
  return 0;
}

}  // namespace

LmbmSolver::Result LmbmSolver::optimize(const LmbmSolverParams& params,
                                        const EvaluateFn& evaluate,
                                        Eigen::VectorXd* variables) const {
  Result result;
  if (!variables || !evaluate || variables->size() == 0) {
    return result;
  }

  LmbmContext context;
  context.evaluate = evaluate;

  std::vector<double> dense_vars(static_cast<size_t>(variables->size()));
  for (Eigen::Index i = 0; i < variables->size(); ++i) {
    dense_vars[static_cast<size_t>(i)] = (*variables)(i);
  }

  lmbm::lmbm_parameter_t native_params;
  native_params.timeout = params.timeout;
  native_params.bundle_size = params.bundle_size;
  native_params.ini_corrections = params.ini_corrections;
  native_params.max_corrections = params.max_corrections;
  native_params.max_iterations = params.max_iterations;
  native_params.max_evaluations = params.max_evaluations;
  native_params.past = params.past;
  native_params.verbose = params.verbose;
  native_params.update_method = params.update_method;
  native_params.scaling_strategy = params.scaling_strategy;
  native_params.delta_past = params.delta_past;
  native_params.f_rel_eps = params.f_rel_eps;
  native_params.terminate_param1 = params.terminate_param1;
  native_params.terminate_param2 = params.terminate_param2;
  native_params.distance_measure = params.distance_measure;
  native_params.sufficient_dec = params.sufficient_dec;
  native_params.max_stepsize = params.max_stepsize;

  double final_cost = 0.0;
  result.code = lmbm::lmbm_optimize(
      static_cast<int>(dense_vars.size()),
      dense_vars.data(),
      &final_cost,
      evaluateThunk,
      &context,
      progressThunk,
      &native_params);
  result.final_cost = final_cost;
  result.iterations = context.iterations;

  for (Eigen::Index i = 0; i < variables->size(); ++i) {
    (*variables)(i) = dense_vars[static_cast<size_t>(i)];
  }

  return result;
}

}  // namespace esv_planner
