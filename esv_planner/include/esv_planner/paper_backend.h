#pragma once

#include <esv_planner/common.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/lmbm_solver.h>
#include <esv_planner/minco_parameterization.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/true_svsdf_penalty.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct PaperBackendParams {
  int max_iterations = 40;
  double nominal_speed = 0.9;
  double min_piece_duration = 0.35;
  double anchor_weight = 2.0;
  double smooth_weight = 10.0;
  double obstacle_weight = 30.0;
  double safety_margin = 0.05;
  double obstacle_cutoff_distance = 1.0;
  double sample_time_step = 0.05;
  LmbmSolverParams lmbm;
};

struct PaperBackendResult {
  bool success = false;
  Trajectory trajectory;
  std::vector<SE2State> support_states;
  int iterations = 0;
  double final_cost = kInf;
  double min_clearance = -kInf;
};

class PaperBackend {
public:
  PaperBackend() = default;

  void init(const GridMap& map,
            const FootprintModel& footprint,
            const SvsdfEvaluator& svsdf,
            const PaperBackendParams& params);

  PaperBackendResult optimize(
      const std::vector<SE2State>& reference_support,
      const std::vector<Eigen::Vector2d>& local_obstacles) const;

private:
  struct CostEvaluation {
    double cost = kInf;
    Eigen::VectorXd gradient;
    std::vector<SE2State> states;
    Trajectory trajectory;
    double min_clearance = kInf;
  };

  std::vector<double> allocateDurations(
      const std::vector<SE2State>& states) const;

  CostEvaluation evaluate(
      const Eigen::VectorXd& vars,
      const std::vector<SE2State>& reference_support,
      const std::vector<Eigen::Vector2d>& local_obstacles) const;

  std::vector<SE2State> unpackStates(
      const Eigen::VectorXd& vars,
      const std::vector<SE2State>& reference_support) const;

  Eigen::VectorXd packInteriorPositions(
      const std::vector<SE2State>& states) const;

  const GridMap* map_ = nullptr;
  const FootprintModel* footprint_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  PaperBackendParams params_;
  TrueSvsdfPenalty penalty_;
  LmbmSolver solver_;
};

}  // namespace esv_planner
