#pragma once

#include <Eigen/Core>

#include <functional>

namespace esv_planner {

struct LmbmSolverParams {
  int max_iterations = 60;
  int max_evaluations = 400;
  float timeout = 30.0f;
  int bundle_size = 2;
  int ini_corrections = 7;
  int max_corrections = 15;
  int past = 5;
  int verbose = -1;
  int update_method = 0;
  int scaling_strategy = 0;
  double delta_past = 1.0e-8;
  double f_rel_eps = 1.0e+4;
  double terminate_param1 = 1.0e-6;
  double terminate_param2 = 1.0e-6;
  double distance_measure = 0.5;
  double sufficient_dec = 1.0e-4;
  double max_stepsize = 1.5;
};

class LmbmSolver {
public:
  typedef std::function<double(const Eigen::VectorXd&, Eigen::VectorXd*)> EvaluateFn;

  struct Result {
    int code = -1;
    int iterations = 0;
    double final_cost = 0.0;
  };

  Result optimize(const LmbmSolverParams& params,
                  const EvaluateFn& evaluate,
                  Eigen::VectorXd* variables) const;
};

}  // namespace esv_planner
