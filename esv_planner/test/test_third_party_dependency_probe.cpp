#include <Eigen/Core>
#include <LBFGS.h>
#include <igl/winding_number.h>

#include <cmath>
#include <iostream>

namespace {

struct QuadraticObjective {
  double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    grad.resize(2);
    grad(0) = 2.0 * (x(0) - 1.0);
    grad(1) = 2.0 * (x(1) + 2.0);
    return std::pow(x(0) - 1.0, 2) + std::pow(x(1) + 2.0, 2);
  }
};

}  // namespace

int main() {
  Eigen::Matrix<double, 4, 2> V;
  V << 0.0, 0.0,
       1.0, 0.0,
       1.0, 1.0,
       0.0, 1.0;
  Eigen::Matrix<int, 4, 2> F;
  F << 0, 1,
       1, 2,
       2, 3,
       3, 0;

  Eigen::RowVector2d inside(0.5, 0.5);
  Eigen::RowVector2d outside(1.5, 0.5);
  const double w_inside = igl::winding_number(V, F, inside);
  const double w_outside = igl::winding_number(V, F, outside);
  std::cout << "[test] winding_inside=" << w_inside
            << " winding_outside=" << w_outside << "\n";
  if (!(w_inside > 0.5) || !(std::abs(w_outside) < 1e-6)) {
    std::cerr << "[test] FAIL: libigl winding number probe mismatch\n";
    return 1;
  }

  LBFGSpp::LBFGSParam<double> param;
  param.max_iterations = 32;
  param.epsilon = 1e-9;
  LBFGSpp::LBFGSSolver<double> solver(param);
  QuadraticObjective objective;
  Eigen::VectorXd x(2);
  x << 10.0, -10.0;
  double fx = 0.0;
  const int iters = solver.minimize(objective, x, fx);
  std::cout << "[test] lbfgs_iters=" << iters
            << " solution=(" << x(0) << "," << x(1) << ")"
            << " fx=" << fx << "\n";
  if ((x - (Eigen::Vector2d() << 1.0, -2.0).finished()).norm() > 1e-4 || fx > 1e-8) {
    std::cerr << "[test] FAIL: LBFGS++ probe mismatch\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
