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

#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/collision_checker.h"
#include "esv_planner/topology_planner.h"
#include "esv_planner/se2_sequence_generator.h"
#include "esv_planner/svsdf_evaluator.h"
#include "esv_planner/trajectory_sampling.h"
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
  ros::Publisher marker_pub_;
  ros::Publisher time_pub_;

  // Components
  GridMap grid_map_;
  FootprintModel footprint_;
  CollisionChecker collision_checker_;
  TopologyPlanner topology_planner_;
  SE2SequenceGenerator se2_generator_;
  SvsdfEvaluator svsdf_evaluator_;
  TrajectoryOptimizer optimizer_;
  OptimizerParams opt_params_;

  // State
  SE2State start_;
  SE2State goal_;
  bool map_ready_ = false;
  bool start_ready_ = false;
  bool goal_ready_ = false;
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
    start_ready_ = true;
    goal_ready_ = false;
  }

  void setupPubSub() {
    map_sub_ = nh_.subscribe("/map", 1, &EsvPlannerNode::mapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &EsvPlannerNode::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EsvPlannerNode::goalCallback, this);

    traj_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/esv_planner/markers", 1);
    time_pub_ = nh_.advertise<std_msgs::Float64>("/esv_planner/solve_time", 1);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map: %dx%d, res=%.3f", msg->info.width, msg->info.height, msg->info.resolution);
    grid_map_.fromOccupancyGrid(msg);

    collision_checker_.init(grid_map_, footprint_, yaw_bins_);
    svsdf_evaluator_.initGeometryBodyFrame(grid_map_.geometryMap(), footprint_);
    topology_planner_.init(grid_map_, collision_checker_, svsdf_evaluator_,
                           num_samples_, knn_, max_paths_, footprint_.inscribedRadius());
    se2_generator_.init(grid_map_, collision_checker_, svsdf_evaluator_, disc_step_, max_push_);
    optimizer_.init(grid_map_, svsdf_evaluator_, opt_params_);

    map_ready_ = true;
    ROS_INFO("Map processed. ESDF computed. Robot kernels generated.");

    if (start_ready_ && goal_ready_) tryPlan();
  }

  void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_.x = msg->pose.pose.position.x;
    start_.y = msg->pose.pose.position.y;
    start_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    start_ready_ = true;
    ROS_INFO("Start: (%.2f, %.2f, %.2f)", start_.x, start_.y, start_.yaw);
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = tf2::getYaw(msg->pose.orientation);
    goal_ready_ = true;
    ROS_INFO("Goal: (%.2f, %.2f, %.2f)", goal_.x, goal_.y, goal_.yaw);

    if (map_ready_ && start_ready_) tryPlan();
  }

  void tryPlan() {
    ROS_INFO("=== ESV Planner: Starting planning ===");
    ros::Time t0 = ros::Time::now();

    // Stage 1: Topology path generation
    topology_planner_.buildRoadmap(start_.position(), goal_.position());
    auto topo_paths = topology_planner_.searchPaths();
    topology_planner_.shortenPaths(topo_paths);
    ROS_INFO("Stage 1: Found %zu topological paths", topo_paths.size());

    if (topo_paths.empty()) {
      ROS_WARN("No topological paths found!");
      publishEmpty(topo_paths, t0);
      return;
    }

    std::vector<Trajectory> candidates;
    std::vector<std::vector<MotionSegment>> all_segments;

    for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
      // Stage 2: SE(2) motion sequence generation
      auto segments = se2_generator_.generate(topo_paths[pi], start_, goal_);
      if (segments.empty()) continue;

      // Stage 3: Trajectory optimization
      std::vector<Trajectory> seg_trajs(segments.size());
      bool path_valid = true;

      // First: optimize high-risk segments with SE(2) planner
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk != RiskLevel::HIGH) continue;
        double seg_len = waypointChainLength(segments[si].waypoints);
        double est_time = std::max(0.5, seg_len / std::max(0.10, opt_params_.max_vel));
        auto result = optimizer_.optimizeSE2(segments[si].waypoints, est_time);
        if (!result.success || result.traj.empty()) { path_valid = false; break; }
        seg_trajs[si] = result.traj;
      }
      if (!path_valid) continue;

      // Then: optimize low-risk segments with R² planner
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk == RiskLevel::HIGH) continue;
        double seg_len = waypointChainLength(segments[si].waypoints);
        double est_time = std::max(0.5, seg_len / std::max(0.10, opt_params_.max_vel));
        auto result = optimizer_.optimizeR2(segments[si].waypoints, est_time);
        if (!result.success || result.traj.empty()) { path_valid = false; break; }
        seg_trajs[si] = result.traj;
      }
      if (!path_valid) continue;

      Trajectory full = optimizer_.stitch(segments, seg_trajs);
      if (full.empty()) continue;

      double min_svsdf = svsdf_evaluator_.evaluateTrajectory(full);
      if (min_svsdf >= 0.0) {
        candidates.push_back(full);
        all_segments.push_back(segments);
        ROS_INFO("  Path %zu accepted: min_svsdf=%.3f", pi, min_svsdf);
        if (min_svsdf >= opt_params_.safety_margin) break;  // early stop
      }
    }

    if (candidates.empty()) {
      ROS_WARN("No valid trajectories generated!");
      publishEmpty(topo_paths, t0);
      return;
    }

    Trajectory best = optimizer_.selectBest(candidates);
    last_segments_ = all_segments.empty() ? std::vector<MotionSegment>() : all_segments[0];

    ros::Time t1 = ros::Time::now();
    double solve_time = (t1 - t0).toSec();
    ROS_INFO("=== Planning done: %.3f s, %zu candidates ===", solve_time, candidates.size());

    publishTrajectory(best);
    publishMarkers(topo_paths, best, last_segments_);
    std_msgs::Float64 time_msg;
    time_msg.data = solve_time;
    time_pub_.publish(time_msg);
  }

  void publishEmpty(const std::vector<TopoPath>& topo_paths, ros::Time t0) {
    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = "map";
    traj_pub_.publish(empty_path);
    publishMarkers(topo_paths, Trajectory(), {});
    std_msgs::Float64 time_msg;
    time_msg.data = (ros::Time::now() - t0).toSec();
    time_pub_.publish(time_msg);
  }

  void publishTrajectory(const Trajectory& traj) {
    if (traj.empty()) return;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    const auto samples = sampleTrajectoryByArcLength(traj, 0.05, 0.02);
    for (const auto& st : samples) {
      geometry_msgs::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = st.x;
      ps.pose.position.y = st.y;
      ps.pose.orientation.z = std::sin(st.yaw * 0.5);
      ps.pose.orientation.w = std::cos(st.yaw * 0.5);
      path_msg.poses.push_back(ps);
    }
    traj_pub_.publish(path_msg);
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
                       const std::vector<MotionSegment>& segments) {
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
      m.color.r = 0.5; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 0.6;
      for (const auto& wp : topo_paths[pi].waypoints) {
        geometry_msgs::Point p;
        p.x = wp.pos.x(); p.y = wp.pos.y(); p.z = 0.05;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // Best trajectory
    if (!best.empty()) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "best_traj";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
      const auto samples = sampleTrajectoryByArcLength(best, 0.05, 0.02);
      for (const auto& st : samples) {
        geometry_msgs::Point p;
        p.x = st.x; p.y = st.y; p.z = 0.1;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);

      // Footprint along trajectory
      const auto fp_samples = sampleTrajectoryByArcLength(best, 0.45, 0.02);
      for (const auto& st : fp_samples) {
        ma.markers.push_back(
            makeFootprintMarker(id, "footprint", st.x, st.y, st.yaw,
                                0.0f, 0.8f, 0.0f, 0.5f, 0.02));
      }
    }

    // SE(2) vs R² segment coloring
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
        p.x = wp.x; p.y = wp.y; p.z = 0.12;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }

    // Start and goal footprint
    ma.markers.push_back(
        makeFootprintMarker(id, "start_goal", start_.x, start_.y, start_.yaw,
                            0.0f, 1.0f, 1.0f, 1.0f, 0.03));
    ma.markers.push_back(
        makeFootprintMarker(id, "start_goal", goal_.x, goal_.y, goal_.yaw,
                            1.0f, 0.0f, 1.0f, 1.0f, 0.03));

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
