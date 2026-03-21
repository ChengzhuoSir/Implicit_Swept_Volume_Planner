#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <unordered_map>

#include "esv_planner/env/collision_checker.h"
#include "esv_planner/core/common.h"
#include "esv_planner/env/footprint_model.h"
#include "esv_planner/env/grid_map.h"
#include "esv_planner/global_search/hybrid_astar.h"
#include "esv_planner/optimization/midend/se2_sequence_generator.h"
#include "esv_planner/optimization/backend/se2_svsdf_solver.h"
#include "esv_planner/optimization/evaluator/svsdf_evaluator.h"
#include "esv_planner/global_search/topology_planner.h"
#include "esv_planner/optimization/backend/trajectory_optimizer.h"

namespace esv_planner {

struct SvsdfPlanResult {
  bool success = false;
  nav_msgs::Path coarse_path;
  nav_msgs::Path trajectory;
  Trajectory raw_trajectory; // Keep the raw continuous trajectory for marker generation
  PlanningStats stats;
};

class SvsdfRuntime {
 public:
  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool UpdateMap(const nav_msgs::OccupancyGrid& map);
  SvsdfPlanResult Plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

  bool map_ready() const { return map_ready_; }
  const FootprintModel& footprint() const { return footprint_; }

  struct CandidateResult {
    Trajectory trajectory;
    std::vector<MotionSegment> segments;
    std::vector<Trajectory> segment_trajectories;
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    int support_points = 0;
    int high_risk_segments = 0;
    int escalated_segments = 0;
    double segment_solve_time = 0.0;
    double full_feasibility_time = 0.0;
    double tail_refine_time = 0.0;
    double recovery_time = 0.0;
    int tail_attempts = 0;
    int tail_pruned = 0;
    bool degraded = false;
    bool budget_exhausted = false;
  };

 private:
  friend class RecoveryManager;

  void LoadParameters(ros::NodeHandle& pnh);
  double EstimateSegmentTime(const std::vector<SE2State>& waypoints) const;
  CandidateResult EvaluateCandidate(const std::vector<MotionSegment>& segments,
                                  const ros::WallTime& deadline);
  void PushSegmentWaypointsFromObstacles(std::vector<MotionSegment>* segments) const;
  bool SolveStrictSegment(const std::vector<SE2State>& waypoints,
                          Trajectory* trajectory);
  bool OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                              double seg_time, Trajectory* trajectory);
  bool TryRecoverBottleneck(const std::vector<MotionSegment>& segments,
                            const std::vector<Trajectory>& segment_trajectories,
                            const Trajectory& current_traj,
                            const ros::WallTime& deadline,
                            CandidateResult* result);
  void MaybeImproveTailTrajectory(const std::vector<MotionSegment>& segments,
                                  const std::vector<Trajectory>& segment_trajectories,
                                  const ros::WallTime& deadline,
                                  CandidateResult* result);
  bool IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                            double* max_vel, double* max_acc) const;

  // Segment solve cache
  using SegmentCacheKey = std::pair<size_t, bool>; // (seg_index, is_strict)
  struct SegmentCacheHash {
    size_t operator()(const SegmentCacheKey& k) const {
      return std::hash<size_t>()(k.first) ^ (std::hash<bool>()(k.second) << 16);
    }
  };
  using SegmentCache = std::unordered_map<SegmentCacheKey, Trajectory, SegmentCacheHash>;

  bool SolveStrictCached(size_t idx, const std::vector<SE2State>& wp,
                         Trajectory* traj, SegmentCache& cache);
  bool OptimizeLowRiskCached(size_t idx, const std::vector<SE2State>& wp,
                             double t, Trajectory* traj, SegmentCache& cache);

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
