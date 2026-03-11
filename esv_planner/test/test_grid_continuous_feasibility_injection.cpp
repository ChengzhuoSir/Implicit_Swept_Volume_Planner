#include <ros/ros.h>

#include <esv_planner/continuous_collision_evaluator.h>
#include <esv_planner/continuous_feasibility.h>

#include <iostream>
#include <memory>

using namespace esv_planner;

namespace {

class FakeContinuousCollisionEvaluator final : public ContinuousCollisionEvaluator {
public:
  double evaluate(const SE2State& state) const override {
    return state.x < 1.0 ? 0.25 : -0.2;
  }

  double evaluateTrajectory(const Trajectory& traj) const override {
    if (traj.empty()) return -kInf;
    return std::min(evaluate(traj.sample(0.0)),
                    evaluate(traj.sample(traj.totalDuration())));
  }

  void gradient(const SE2State& state,
                Eigen::Vector2d& grad_pos,
                double& grad_yaw) const override {
    (void)state;
    grad_pos.setZero();
    grad_yaw = 0.0;
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_grid_continuous_feasibility_injection",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  auto evaluator = std::make_shared<FakeContinuousCollisionEvaluator>();
  GridContinuousFeasibilityChecker checker(evaluator, 0.1, 0.05);

  const SE2State safe_a(0.2, 0.0, 0.0);
  const SE2State safe_b(0.8, 0.0, 0.0);
  const SE2State unsafe_c(1.2, 0.0, 0.0);

  const double ca = checker.stateClearance(safe_a);
  const double cb = checker.stateClearance(unsafe_c);
  std::cout << "[test] state_clearance_safe=" << ca
            << " state_clearance_unsafe=" << cb << "\n";
  if (std::abs(ca - 0.25) > 1e-9 || std::abs(cb + 0.2) > 1e-9) {
    std::cerr << "[test] FAIL: injected collision evaluator must drive state clearance\n";
    return 1;
  }

  const double safe_transition = checker.transitionClearance(safe_a, safe_b);
  const double unsafe_transition = checker.transitionClearance(safe_a, unsafe_c);
  std::cout << "[test] transition_safe=" << safe_transition
            << " transition_unsafe=" << unsafe_transition << "\n";
  if (safe_transition < 0.2) {
    std::cerr << "[test] FAIL: safe transition should remain safe under injected evaluator\n";
    return 1;
  }
  if (unsafe_transition > -0.1) {
    std::cerr << "[test] FAIL: unsafe transition should inherit injected evaluator violation\n";
    return 1;
  }

  SE2State yaw_state(0.2, 0.0, 0.0);
  if (!checker.safeYaw(yaw_state, 0.75) || std::abs(yaw_state.yaw - 0.75) > 1e-9) {
    std::cerr << "[test] FAIL: injected feasibility checker should use identity safeYaw assignment\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
