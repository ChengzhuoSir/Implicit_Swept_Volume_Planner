#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Core>

#include "esv_planner/collision_checker.h"
#include "esv_planner/common.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/hybrid_astar.h"
#include "esv_planner/se2_sequence_generator.h"
#include "esv_planner/se2_svsdf_solver.h"
#include "esv_planner/svsdf_evaluator.h"
#include "esv_planner/topology_planner.h"
#include "esv_planner/trajectory_optimizer.h"

namespace esv_planner {

struct SvsdfPlanResult {
  bool success = false;
  nav_msgs::Path coarse_path;
  nav_msgs::Path trajectory;
  PlanningStats stats;
};

class SvsdfRuntime {
 public:
  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool UpdateMap(const nav_msgs::OccupancyGrid& map);
  SvsdfPlanResult Plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

  bool map_ready() const { return map_ready_; }

 private:
  struct CandidateResult {
    Trajectory trajectory;
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    int support_points = 0;
    int high_risk_segments = 0;
    int escalated_segments = 0;
    bool degraded = false;
  };

  void LoadParameters(ros::NodeHandle& pnh);
  nav_msgs::Path MakePath(const std::vector<SE2State>& states) const;
  nav_msgs::Path MakePathFromTopo(const TopoPath& path) const;
  nav_msgs::Path MakeTrajectoryPath(const Trajectory& traj) const;
  double EstimateSegmentTime(const std::vector<SE2State>& waypoints) const;
  CandidateResult EvaluateCandidate(const std::vector<MotionSegment>& segments);
  void PushSegmentWaypointsFromObstacles(std::vector<MotionSegment>* segments) const;
  bool SolveStrictSegment(const std::vector<SE2State>& waypoints,
                          Trajectory* trajectory);
  bool OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                              double seg_time, Trajectory* trajectory);
  bool TryRecoverBottleneck(const std::vector<MotionSegment>& segments,
                            const std::vector<Trajectory>& segment_trajectories,
                            const Trajectory& current_traj,
                            CandidateResult* result);
  void MaybeImproveTailTrajectory(const std::vector<MotionSegment>& segments,
                                  const std::vector<Trajectory>& segment_trajectories,
                                  CandidateResult* result);
  bool IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                            double* max_vel, double* max_acc) const;

  GridMap grid_map_;
  FootprintModel footprint_;
  CollisionChecker collision_checker_;
  SE2SequenceGenerator se2_generator_;
  HybridAStarPlanner hybrid_astar_;
  TopologyPlanner topology_planner_;
  SvsdfEvaluator svsdf_evaluator_;
  TrajectoryOptimizer optimizer_;
  Se2SvsdfSolver strict_solver_;

  OptimizerParams optimizer_params_;
  HybridAStarParams hybrid_astar_params_;

  int topology_num_samples_ = 520;
  int topology_knn_ = 18;
  int topology_max_paths_ = 14;
  double se2_disc_step_ = 0.15;
  int yaw_bins_ = 18;
  int max_push_attempts_ = 5;

  nav_msgs::OccupancyGrid latest_map_;
  bool initialized_ = false;
  bool map_ready_ = false;
};

}  // namespace esv_planner
