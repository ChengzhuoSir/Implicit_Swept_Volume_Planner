#pragma once

#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>

#include "isweep_planner/core/common.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/optimization/evaluator/svsdf_evaluator.h"

namespace isweep_planner {

struct ClearanceProbe {
  bool valid = false;
  double time = 0.0;
  double clearance = kInf;
  SE2State state;
};

bool DeadlineExpired(const ros::WallTime& deadline);
double RemainingBudgetSec(const ros::WallTime& deadline);

double WaypointPathLength(const std::vector<SE2State>& waypoints);

void AppendUniqueWaypoints(const std::vector<SE2State>& source,
                           std::vector<SE2State>* target);

std::vector<SE2State> BuildLinearWaypoints(const SE2State& start, const SE2State& goal,
                                           double step);

Trajectory ConcatenateTrajectories(const std::vector<Trajectory>& segment_trajectories,
                                   size_t prefix_count, const Trajectory& suffix);

Trajectory ConcatenateTrajectories(const std::vector<Trajectory>& segment_trajectories,
                                   size_t prefix_count, size_t suffix_begin,
                                   const Trajectory& middle);

void AppendTrajectoryPieces(const Trajectory& source, Trajectory* target);

double ApproximateTrajectoryLength(const Trajectory& traj);

double TrajectorySmoothnessCost(const Trajectory& traj);

int CountHighRiskSegments(const std::vector<MotionSegment>& segments, size_t end_index);

int CountHighRiskSegments(const std::vector<MotionSegment>& segments, size_t begin_index,
                          size_t end_index);

int CountSupportPoints(const std::vector<MotionSegment>& segments, size_t begin_index,
                       size_t end_index);
                       
int CountSupportPoints(const std::vector<MotionSegment>& segments);

ClearanceProbe FindWorstClearanceSample(const Trajectory& traj,
                                        const SvsdfEvaluator& evaluator);

size_t SegmentIndexFromTime(const std::vector<Trajectory>& segment_trajectories, double time);

std::pair<double, double> SegmentTimeWindow(
    const std::vector<Trajectory>& segment_trajectories, size_t begin_index,
    size_t end_index);

std::vector<Eigen::Vector2d> SampleLowClearanceCorridor(
    const Trajectory& traj, const SvsdfEvaluator& evaluator, double begin_time,
    double end_time, double clearance_threshold, double min_spacing);

double MinDistanceToCenters(const Eigen::Vector2d& point,
                            const std::vector<Eigen::Vector2d>& centers);

bool CorridorPositionAllowed(const GridMap& map, const CollisionChecker& checker,
                             const Eigen::Vector2d& point, double required_clearance,
                             const std::vector<Eigen::Vector2d>& blocked_centers,
                             double blocked_radius, int min_safe_yaw_count);

bool CorridorLineFree(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                      const GridMap& map, const CollisionChecker& checker,
                      double required_clearance,
                      const std::vector<Eigen::Vector2d>& blocked_centers,
                      double blocked_radius, int min_safe_yaw_count);

TopoPath BuildTopoPathFromCorridor(
    const std::vector<Eigen::Vector2d>& dense_path, const GridMap& map,
    const CollisionChecker& checker, double required_clearance,
    const std::vector<Eigen::Vector2d>& blocked_centers, double blocked_radius,
    int min_safe_yaw_count);

std::vector<Eigen::Vector2d> LoadFootprint(ros::NodeHandle& pnh);

SE2State ToSe2State(const Eigen::Vector3d& state);

std::vector<SE2State> SparsifyStrictWaypoints(const std::vector<SE2State>& dense,
                                             const GridMap& map);

}  // namespace isweep_planner