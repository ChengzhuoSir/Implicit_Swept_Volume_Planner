#pragma once

#include <esv_planner/common.h>
#include <esv_planner/footprint_model.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

struct TrueSvsdfPenaltyParams {
  double safety_margin = 0.05;
  double obstacle_weight = 50.0;
  double obstacle_cutoff_distance = 1.0;
  double sample_time_step = 0.05;
};

class TrueSvsdfPenalty {
public:
  struct Evaluation {
    double cost = 0.0;
    double min_clearance = kInf;
    Eigen::VectorXd gradient;
  };

  TrueSvsdfPenalty() = default;

  void init(const FootprintModel& footprint,
            const TrueSvsdfPenaltyParams& params);

  Evaluation evaluate(const Trajectory& trajectory,
                      int support_count,
                      const std::vector<Eigen::Vector2d>& local_obstacles) const;

private:
  struct SweptSample {
    SE2State state;
    int segment_index = 0;
    double alpha = 0.0;
  };

  std::vector<SweptSample> buildSamples(const Trajectory& trajectory) const;
  void addGradientContribution(int support_index,
                               const Eigen::Vector2d& gradient,
                               double weight,
                               Eigen::VectorXd* out_gradient) const;

  const FootprintModel* footprint_ = nullptr;
  TrueSvsdfPenaltyParams params_;
};

}  // namespace esv_planner
