#pragma once

#include <esv_planner/common.h>

#include <Eigen/Dense>

#include <vector>

namespace esv_planner {

struct MincoDecisionVariables {
  Eigen::VectorXd values;
  int piece_count = 0;
};

class MincoParameterization {
public:
  static MincoDecisionVariables packSe2DecisionVariables(
      const std::vector<SE2State>& support_states,
      const std::vector<double>& durations);

  static bool unpackSe2DecisionVariables(
      const MincoDecisionVariables& vars,
      const SE2State& start,
      const SE2State& goal,
      std::vector<SE2State>* support_states,
      std::vector<double>* durations);

  static Trajectory buildSe2Trajectory(const MincoDecisionVariables& vars,
                                       const SE2State& start,
                                       const SE2State& goal);

private:
  static void fitPositionQuintic(const std::vector<Eigen::Vector2d>& positions,
                                 const std::vector<double>& durations,
                                 const Eigen::Vector2d& v0,
                                 const Eigen::Vector2d& vf,
                                 const Eigen::Vector2d& a0,
                                 const Eigen::Vector2d& af,
                                 std::vector<PolyPiece>* pieces);

  static void fitYawQuintic(const std::vector<double>& yaws,
                            const std::vector<double>& durations,
                            double omega0,
                            double omegaf,
                            std::vector<YawPolyPiece>* pieces);
};

}  // namespace esv_planner
