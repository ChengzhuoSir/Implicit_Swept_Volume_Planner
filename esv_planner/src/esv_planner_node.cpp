#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <sstream>

#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/collision_checker.h"
#include "esv_planner/topology_planner.h"
#include "esv_planner/se2_sequence_generator.h"
#include "esv_planner/hybrid_astar.h"
#include "esv_planner/planner_trigger_state.h"
#include "esv_planner/svsdf_evaluator.h"
#include "esv_planner/trajectory_optimizer.h"

namespace esv_planner {

class EsvPlannerNode {
public:
  EsvPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadParams();
    setupPubSub();
    ROS_INFO("ESV Planner node initialized.");
  }

private:
  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  // Publishers
  ros::Publisher traj_pub_;
  ros::Publisher traj_astar_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher time_pub_;

  // Components
  GridMap grid_map_;
  FootprintModel footprint_;
  CollisionChecker collision_checker_;
  TopologyPlanner topology_planner_;
  SE2SequenceGenerator se2_generator_;
  HybridAStarPlanner hybrid_astar_;
  SvsdfEvaluator svsdf_evaluator_;
  TrajectoryOptimizer optimizer_;
  OptimizerParams opt_params_;
  HybridAStarParams hybrid_params_;

  // State
  SE2State start_;
  SE2State goal_;
  PlannerTriggerState trigger_state_;
  std::vector<MotionSegment> last_segments_;

  // Parameters
  int num_samples_, knn_, max_paths_;
  double disc_step_;
  int yaw_bins_;
  int max_push_;

  void loadParams() {
    // Footprint
    std::vector<double> fp_flat;
    pnh_.param("footprint", fp_flat, std::vector<double>());
    std::vector<Eigen::Vector2d> fp_verts;
    for (size_t i = 0; i + 1 < fp_flat.size(); i += 2) {
      fp_verts.emplace_back(fp_flat[i], fp_flat[i + 1]);
    }
    if (fp_verts.empty()) {
      // Default T-shape
      fp_verts = {{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
                  {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}};
    }
    footprint_.setPolygon(fp_verts);

    double insc_r;
    pnh_.param("inscribed_radius", insc_r, 0.1);
    footprint_.setInscribedRadius(insc_r);

    // Topology
    pnh_.param("topology/num_samples", num_samples_, 520);
    pnh_.param("topology/knn", knn_, 18);
    pnh_.param("topology/max_paths", max_paths_, 14);

    // SE2
    pnh_.param("se2/discretization_step", disc_step_, 0.15);
    pnh_.param("se2/yaw_bins", yaw_bins_, 18);
    pnh_.param("se2/max_push_attempts", max_push_, 5);

    // Optimizer
    pnh_.param("optimizer/max_iterations", opt_params_.max_iterations, 100);
    pnh_.param("optimizer/lambda_smooth", opt_params_.lambda_smooth, 1.0);
    pnh_.param("optimizer/lambda_time", opt_params_.lambda_time, 1.0);
    pnh_.param("optimizer/lambda_safety", opt_params_.lambda_safety, 10.0);
    pnh_.param("optimizer/lambda_dynamics", opt_params_.lambda_dynamics, 1.0);
    pnh_.param("optimizer/max_vel", opt_params_.max_vel, 1.0);
    pnh_.param("optimizer/max_acc", opt_params_.max_acc, 2.0);
    pnh_.param("optimizer/max_yaw_rate", opt_params_.max_yaw_rate, 1.5);
    pnh_.param("optimizer/lambda_pos_residual", opt_params_.lambda_pos_residual, 5.0);
    pnh_.param("optimizer/lambda_yaw_residual", opt_params_.lambda_yaw_residual, 2.0);
    pnh_.param("optimizer/safety_margin", opt_params_.safety_margin, 0.05);
    pnh_.param("optimizer/step_size", opt_params_.step_size, 0.005);

    // Hybrid A*
    pnh_.param("hybrid_astar/step_size", hybrid_params_.step_size, 0.35);
    pnh_.param("hybrid_astar/wheel_base", hybrid_params_.wheel_base, 0.8);
    pnh_.param("hybrid_astar/max_steer", hybrid_params_.max_steer, 0.6);
    pnh_.param("hybrid_astar/steer_samples", hybrid_params_.steer_samples, 5);
    pnh_.param("hybrid_astar/goal_tolerance_pos", hybrid_params_.goal_tolerance_pos, 0.35);
    pnh_.param("hybrid_astar/goal_tolerance_yaw", hybrid_params_.goal_tolerance_yaw, 0.4);
    pnh_.param("hybrid_astar/reverse_penalty", hybrid_params_.reverse_penalty, 2.0);
    pnh_.param("hybrid_astar/steer_penalty", hybrid_params_.steer_penalty, 0.2);
    pnh_.param("hybrid_astar/steer_change_penalty", hybrid_params_.steer_change_penalty, 0.2);
    pnh_.param("hybrid_astar/switch_penalty", hybrid_params_.switch_penalty, 2.0);
    pnh_.param("hybrid_astar/max_expansions", hybrid_params_.max_expansions, 50000);
    pnh_.param("hybrid_astar/fast_pass_max_expansions", hybrid_params_.fast_pass_max_expansions, 8000);
    pnh_.param("hybrid_astar/fast_pass_steer_samples", hybrid_params_.fast_pass_steer_samples, 3);
    pnh_.param("hybrid_astar/pose_recovery_max_radius", hybrid_params_.pose_recovery_max_radius, 0.6);
    pnh_.param("hybrid_astar/pose_recovery_radial_step", hybrid_params_.pose_recovery_radial_step, 0.05);
    pnh_.param("hybrid_astar/pose_recovery_angular_samples", hybrid_params_.pose_recovery_angular_samples, 24);

    // Default start/goal
    double sx, sy, syaw, gx, gy, gyaw;
    pnh_.param("start_pose/x", sx, 1.0);
    pnh_.param("start_pose/y", sy, 1.0);
    pnh_.param("start_pose/yaw", syaw, 0.0);
    pnh_.param("goal_pose/x", gx, 8.0);
    pnh_.param("goal_pose/y", gy, 8.0);
    pnh_.param("goal_pose/yaw", gyaw, 1.57);
    start_ = SE2State(sx, sy, syaw);
    goal_ = SE2State(gx, gy, gyaw);
    trigger_state_.setDefaultStartGoalAvailable();
  }

  void setupPubSub() {
    map_sub_ = nh_.subscribe("/map", 1, &EsvPlannerNode::mapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &EsvPlannerNode::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EsvPlannerNode::goalCallback, this);

    traj_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory", 1);
    traj_astar_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory_astar", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/esv_planner/markers", 1);
    time_pub_ = nh_.advertise<std_msgs::Float64>("/esv_planner/solve_time", 1);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map: %dx%d, res=%.3f", msg->info.width, msg->info.height, msg->info.resolution);
    grid_map_.fromOccupancyGrid(msg);

    // Initialize components
    collision_checker_.init(grid_map_, footprint_, yaw_bins_);
    topology_planner_.init(grid_map_, collision_checker_, num_samples_, knn_, max_paths_,
                           footprint_.inscribedRadius());
    se2_generator_.init(grid_map_, collision_checker_, disc_step_, max_push_);
    hybrid_astar_.init(grid_map_, collision_checker_, hybrid_params_);
    svsdf_evaluator_.init(grid_map_, footprint_);
    optimizer_.init(grid_map_, svsdf_evaluator_, opt_params_);

    ROS_INFO("Map processed. ESDF computed. Robot kernels generated.");

    if (trigger_state_.onMapReceived()) {
      tryPlan();
    }
  }

  void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_.x = msg->pose.pose.position.x;
    start_.y = msg->pose.pose.position.y;
    start_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Start: (%.2f, %.2f, %.2f)", start_.x, start_.y, start_.yaw);
    trigger_state_.onStartUpdated();
    ROS_INFO("Start updated. Waiting for goal selection to plan.");
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = tf2::getYaw(msg->pose.orientation);
    ROS_INFO("Goal: (%.2f, %.2f, %.2f)", goal_.x, goal_.y, goal_.yaw);
    if (trigger_state_.onGoalUpdated()) {
      tryPlan();
    }
  }

  void tryPlan() {
    if (!trigger_state_.canPlan()) return;

    ROS_INFO("=== ESV Planner: Starting planning ===");
    ros::Time t0 = ros::Time::now();
    std::vector<SE2State> hybrid_path;

    // Always try A* baseline, independent of ESV pipeline success.
    bool hybrid_ok = hybrid_astar_.plan(start_, goal_, hybrid_path);
    if (!hybrid_ok || hybrid_path.empty()) {
      ROS_WARN("Hybrid A* failed or produced empty path. start=(%.2f, %.2f, %.2f), "
               "goal=(%.2f, %.2f, %.2f)",
               start_.x, start_.y, start_.yaw, goal_.x, goal_.y, goal_.yaw);
    } else {
      ROS_INFO("Hybrid A* path size: %zu", hybrid_path.size());
      publishHybridPath(hybrid_path, traj_astar_pub_);
    }

    // Stage 1: Topology path generation
    topology_planner_.buildRoadmap(start_.position(), goal_.position());
    auto topo_paths = topology_planner_.searchPaths();
    topology_planner_.shortenPaths(topo_paths);
    ROS_INFO("Stage 1: Found %zu topological paths", topo_paths.size());

    if (topo_paths.empty()) {
      const double start_esdf = grid_map_.getEsdf(start_.x, start_.y);
      const double goal_esdf = grid_map_.getEsdf(goal_.x, goal_.y);
      const bool start_free = collision_checker_.isFree(start_);
      const bool goal_free = collision_checker_.isFree(goal_);
      ROS_WARN("No topological paths found! nodes=%zu start_degree=%d goal_degree=%d "
               "start_esdf=%.3f goal_esdf=%.3f start_free=%d goal_free=%d",
               topology_planner_.numNodes(),
               topology_planner_.startDegree(),
               topology_planner_.goalDegree(),
               start_esdf, goal_esdf,
               start_free ? 1 : 0,
               goal_free ? 1 : 0);
      return;
    }

    std::vector<Trajectory> candidates;
    std::vector<std::vector<MotionSegment>> all_segments;

    auto segmentLength = [](const std::vector<SE2State>& wps) {
      if (wps.size() < 2) return 0.0;
      double len = 0.0;
      for (size_t i = 1; i < wps.size(); ++i) {
        double dx = wps[i].x - wps[i - 1].x;
        double dy = wps[i].y - wps[i - 1].y;
        len += std::sqrt(dx * dx + dy * dy);
      }
      return len;
    };

    struct ValidationReport {
      bool has_traj = false;
      bool collision_free = false;
      bool margin_ok = false;
      bool dynamics_ok = false;
      bool accepted_by_current_gate = false;
      double min_svsdf = 0.0;
      double max_vel = 0.0;
      double max_acc = 0.0;
      double max_yaw_rate = 0.0;
    };

    auto inspectTrajectory = [&](const Trajectory& traj) {
      ValidationReport report;
      report.has_traj = !traj.empty();
      if (!report.has_traj) {
        return report;
      }

      report.min_svsdf = svsdf_evaluator_.evaluateTrajectory(traj, 0.05);
      report.dynamics_ok = optimizer_.dynamicsFeasible(
          traj, &report.max_vel, &report.max_acc, &report.max_yaw_rate);
      report.collision_free = (report.min_svsdf >= 0.0);
      report.margin_ok = (report.min_svsdf >= opt_params_.safety_margin);
      report.accepted_by_current_gate =
          (report.min_svsdf >= -opt_params_.safety_margin) && report.dynamics_ok;
      return report;
    };

    auto formatValidation = [&](const ValidationReport& report) {
      std::ostringstream oss;
      if (!report.has_traj) {
        oss << "traj=empty";
        return oss.str();
      }
      oss << "min_svsdf=" << report.min_svsdf
          << " collision_free=" << (report.collision_free ? 1 : 0)
          << " margin_ok=" << (report.margin_ok ? 1 : 0)
          << " dynamics_ok=" << (report.dynamics_ok ? 1 : 0)
          << " vmax=" << report.max_vel
          << " amax=" << report.max_acc
          << " yaw_rate_max=" << report.max_yaw_rate;
      return oss.str();
    };

    auto validateTrajectory = [&](const Trajectory& traj,
                                  double* out_min_svsdf,
                                  double* out_max_vel,
                                  double* out_max_acc,
                                  ValidationReport* out_report) {
      ValidationReport report = inspectTrajectory(traj);
      if (out_min_svsdf) *out_min_svsdf = report.min_svsdf;
      if (out_max_vel) *out_max_vel = report.max_vel;
      if (out_max_acc) *out_max_acc = report.max_acc;
      if (out_report) *out_report = report;
      return report.accepted_by_current_gate;
    };

    for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
      // Stage 2: SE(2) motion sequence
      auto segments = se2_generator_.generate(topo_paths[pi], start_, goal_);
      size_t high_count = 0;
      size_t low_count = 0;
      for (const auto& seg : segments) {
        if (seg.risk == RiskLevel::HIGH) ++high_count;
        else ++low_count;
      }
      ROS_INFO("  Path %zu: %zu segments (high=%zu low=%zu)", pi, segments.size(),
               high_count, low_count);
      if (segments.empty()) {
        ROS_WARN("  Path %zu discarded: no segments generated", pi);
        continue;
      }

      // Stage 3: Trajectory optimization
      std::vector<Trajectory> seg_trajs(segments.size());
      std::vector<double> seg_times(segments.size(), 0.5);
      for (size_t si = 0; si < segments.size(); ++si) {
        double seg_len = segmentLength(segments[si].waypoints);
        double est_time = seg_len / std::max(0.10, opt_params_.max_vel);
        seg_times[si] = std::max(0.5, est_time);
      }

      bool path_valid = true;
      // First: high-risk SE(2) segments must be feasible, otherwise discard path.
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk != RiskLevel::HIGH) continue;
        Trajectory seg_traj = optimizer_.optimizeSE2(segments[si].waypoints, seg_times[si]);
        if (seg_traj.empty()) {
          ROS_WARN("  Path %zu discarded: high-risk segment %zu optimizeSE2 returned empty "
                   "(wps=%zu est_time=%.3f)", pi, si, segments[si].waypoints.size(), seg_times[si]);
          path_valid = false;
          break;
        }
        ValidationReport seg_report = inspectTrajectory(seg_traj);
        if (seg_report.min_svsdf < -opt_params_.safety_margin) {
          ROS_WARN("  Path %zu discarded: high-risk segment %zu infeasible: %s",
                   pi, si, formatValidation(seg_report).c_str());
          path_valid = false;
          break;
        }
        seg_trajs[si] = seg_traj;
      }
      if (!path_valid) {
        ROS_WARN("  Path %zu discarded: high-risk SE(2) segment infeasible", pi);
        continue;
      }

      // Then optimize low-risk segments in R2.
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk == RiskLevel::HIGH) continue;
        Trajectory seg_traj = optimizer_.optimizeR2(segments[si].waypoints, seg_times[si]);
        if (seg_traj.empty()) {
          ROS_WARN("  Path %zu discarded: low-risk segment %zu optimizeR2 returned empty "
                   "(wps=%zu est_time=%.3f)", pi, si, segments[si].waypoints.size(), seg_times[si]);
          path_valid = false;
          break;
        }
        seg_trajs[si] = seg_traj;
      }
      if (!path_valid) {
        ROS_WARN("  Path %zu discarded: low-risk optimization failed", pi);
        continue;
      }

      Trajectory full = optimizer_.stitch(segments, seg_trajs);
      if (full.empty()) {
        ROS_WARN("  Path %zu discarded: stitch failed", pi);
        continue;
      }

      double min_svsdf = 0.0, max_vel = 0.0, max_acc = 0.0;
      ValidationReport full_report;
      bool valid = validateTrajectory(full, &min_svsdf, &max_vel, &max_acc, &full_report);
      if (!valid) {
        ROS_WARN("  Path %zu full validation failed: %s", pi,
                 formatValidation(full_report).c_str());
        // Paper-aligned fallback: if full trajectory is unsafe, upgrade R2 to SE(2).
        bool fallback_ok = true;
        for (size_t si = 0; si < segments.size(); ++si) {
          if (segments[si].risk == RiskLevel::HIGH) continue;
          Trajectory reopt = optimizer_.optimizeSE2(segments[si].waypoints, seg_times[si]);
          if (reopt.empty()) {
            ROS_WARN("  Path %zu fallback failed: low-risk segment %zu optimizeSE2 returned empty",
                     pi, si);
            fallback_ok = false;
            break;
          }
          ValidationReport fallback_seg_report = inspectTrajectory(reopt);
          if (fallback_seg_report.min_svsdf < -opt_params_.safety_margin) {
            ROS_WARN("  Path %zu fallback failed: upgraded segment %zu infeasible: %s",
                     pi, si, formatValidation(fallback_seg_report).c_str());
            fallback_ok = false;
            break;
          }
          seg_trajs[si] = reopt;
        }

        if (fallback_ok) {
          full = optimizer_.stitch(segments, seg_trajs);
          valid = validateTrajectory(full, &min_svsdf, &max_vel, &max_acc, &full_report);
          if (!valid) {
            ROS_WARN("  Path %zu fallback validation still failed: %s", pi,
                     formatValidation(full_report).c_str());
          }
        }
      }

      if (valid) {
        ROS_INFO("  Path %zu accepted: %s", pi, formatValidation(full_report).c_str());
        if (!full_report.collision_free) {
          ROS_ERROR("  Path %zu accepted even though it penetrates obstacles "
                    "(min_svsdf=%.3f). Current hard gate only rejects "
                    "min_svsdf < -safety_margin (%.3f).",
                    pi, full_report.min_svsdf, opt_params_.safety_margin);
        } else if (!full_report.margin_ok) {
          ROS_WARN("  Path %zu accepted but clearance is below desired safety_margin "
                   "(min_svsdf=%.3f < %.3f).",
                   pi, full_report.min_svsdf, opt_params_.safety_margin);
        }
        candidates.push_back(full);
        all_segments.push_back(segments);
      } else {
        ROS_WARN("  Path %zu discarded after validation/fallback", pi);
      }
    }

    if (candidates.empty()) {
      ROS_WARN("No valid trajectories generated!");
      return;
    }

    Trajectory best = optimizer_.selectBest(candidates);
    if (best.empty()) {
      ROS_WARN("No valid trajectory selected!");
      return;
    }

    // Find which candidate was selected and store its segments
    last_segments_.clear();
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].pos_pieces.size() == best.pos_pieces.size() &&
          std::abs(candidates[i].totalDuration() - best.totalDuration()) < 1e-6) {
        last_segments_ = all_segments[i];
        break;
      }
    }
    if (last_segments_.empty() && !all_segments.empty()) {
      last_segments_ = all_segments[0];
    }

    ros::Time t1 = ros::Time::now();
    double solve_time = (t1 - t0).toSec();
    ROS_INFO("=== Planning done: %.3f s, %zu candidates ===", solve_time, candidates.size());

    // Publish
    publishTrajectory(best, traj_pub_);
    publishHybridPath(hybrid_path, traj_astar_pub_);
    publishMarkers(topo_paths, best, last_segments_, hybrid_path);

    std_msgs::Float64 time_msg;
    time_msg.data = solve_time;
    time_pub_.publish(time_msg);
  }

  void publishTrajectory(const Trajectory& traj, const ros::Publisher& pub) {
    if (traj.empty()) return;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    double total = traj.totalDuration();
    double dt = 0.05;
    for (double t = 0.0; t <= total; t += dt) {
      SE2State st = traj.sample(t);
      geometry_msgs::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = st.x;
      ps.pose.position.y = st.y;
      ps.pose.orientation.z = std::sin(st.yaw * 0.5);
      ps.pose.orientation.w = std::cos(st.yaw * 0.5);
      path_msg.poses.push_back(ps);
    }

    pub.publish(path_msg);
  }

  void publishHybridPath(const std::vector<SE2State>& path, const ros::Publisher& pub) {
    if (path.empty()) return;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    for (const auto& st : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = st.x;
      ps.pose.position.y = st.y;
      ps.pose.orientation.z = std::sin(st.yaw * 0.5);
      ps.pose.orientation.w = std::cos(st.yaw * 0.5);
      path_msg.poses.push_back(ps);
    }
    pub.publish(path_msg);
  }

  visualization_msgs::Marker makeFootprintMarker(
      int& id, const std::string& ns, double x, double y, double yaw,
      float r, float g, float b, float a, double scale_x) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id++;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = scale_x;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;

    auto verts = footprint_.rotatedVertices(yaw);
    for (const auto& v : verts) {
      geometry_msgs::Point p;
      p.x = v.x() + x;
      p.y = v.y() + y;
      p.z = 0.15;
      m.points.push_back(p);
    }
    // Close polygon
    if (!verts.empty()) {
      geometry_msgs::Point p;
      p.x = verts[0].x() + x;
      p.y = verts[0].y() + y;
      p.z = 0.15;
      m.points.push_back(p);
    }
    return m;
  }

  void publishMarkers(const std::vector<TopoPath>& topo_paths,
                       const Trajectory& best,
                       const std::vector<MotionSegment>& segments,
                       const std::vector<SE2State>& hybrid_path) {
    visualization_msgs::MarkerArray ma;
    int id = 0;

    // Topological paths
    for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "topo_paths";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.03;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 1.0;
      m.color.a = 0.6;

      for (const auto& wp : topo_paths[pi].waypoints) {
        geometry_msgs::Point p;
        p.x = wp.pos.x();
        p.y = wp.pos.y();
        p.z = 0.05;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // Best trajectory
    {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "best_traj";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      double total = best.totalDuration();
      for (double t = 0.0; t <= total; t += 0.05) {
        SE2State st = best.sample(t);
        geometry_msgs::Point p;
        p.x = st.x;
        p.y = st.y;
        p.z = 0.1;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // A* path
    if (!hybrid_path.empty()) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "traj_astar";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.04;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 0.95;
      for (const auto& st : hybrid_path) {
        geometry_msgs::Point p;
        p.x = st.x;
        p.y = st.y;
        p.z = 0.1;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // 1) Robot footprint polygons along best trajectory (ns="footprint")
    {
      double total = best.totalDuration();
      for (double t = 0.0; t <= total; t += 0.3) {
        SE2State st = best.sample(t);
        ma.markers.push_back(
            makeFootprintMarker(id, "footprint", st.x, st.y, st.yaw,
                                0.0f, 0.8f, 0.0f, 0.5f, 0.02));
      }
    }

    // 2) SE(2) vs R2 segment coloring (ns="segments")
    for (const auto& seg : segments) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "segments";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.04;
      if (seg.risk == RiskLevel::HIGH) {
        m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 0.8f;
      } else {
        m.color.r = 0.2f; m.color.g = 0.2f; m.color.b = 1.0f; m.color.a = 0.8f;
      }
      for (const auto& wp : seg.waypoints) {
        geometry_msgs::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.12;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // 3) Start and goal footprint (ns="start_goal")
    ma.markers.push_back(
        makeFootprintMarker(id, "start_goal", start_.x, start_.y, start_.yaw,
                            0.0f, 1.0f, 1.0f, 1.0f, 0.03));
    ma.markers.push_back(
        makeFootprintMarker(id, "start_goal", goal_.x, goal_.y, goal_.yaw,
                            1.0f, 0.0f, 1.0f, 1.0f, 0.03));

    // 4) Waypoint markers (ns="waypoints")
    {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "waypoints";
      m.id = id++;
      m.type = visualization_msgs::Marker::SPHERE_LIST;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.05;
      m.scale.y = 0.05;
      m.scale.z = 0.05;

      for (const auto& seg : segments) {
        std_msgs::ColorRGBA c;
        if (seg.risk == RiskLevel::HIGH) {
          c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 1.0f;
        } else {
          c.r = 0.0f; c.g = 0.0f; c.b = 1.0f; c.a = 1.0f;
        }
        for (const auto& wp : seg.waypoints) {
          geometry_msgs::Point p;
          p.x = wp.x;
          p.y = wp.y;
          p.z = 0.2;
          m.points.push_back(p);
          m.colors.push_back(c);
        }
      }
      ma.markers.push_back(m);
    }

    marker_pub_.publish(ma);
  }
};

}  // namespace esv_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "esv_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  esv_planner::EsvPlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
