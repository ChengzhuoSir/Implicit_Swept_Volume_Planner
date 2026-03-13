#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "esv_planner/common.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/paper_planner.h"
#include "esv_planner/svsdf_evaluator.h"
#include "esv_planner/trajectory_sampling.h"

namespace esv_planner {

class EsvPlannerNode {
public:
  EsvPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    loadParams();
    setupPubSub();
    ROS_INFO("ESV Planner node initialized.");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  ros::Publisher traj_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher time_pub_;
  ros::Publisher stats_pub_;

  GridMap grid_map_;
  FootprintModel footprint_;
  SvsdfEvaluator svsdf_evaluator_;
  PaperPlanner planner_;
  PaperPlannerParams planner_params_;

  SE2State start_;
  SE2State goal_;
  bool map_ready_ = false;
  bool start_ready_ = false;
  bool goal_ready_ = false;
  bool use_start_goal_params_ = true;

  void loadParams() {
    pnh_.param("use_start_goal_params", use_start_goal_params_, true);

    std::vector<double> fp_flat;
    pnh_.param("footprint", fp_flat, std::vector<double>());
    std::vector<Eigen::Vector2d> fp_verts;
    for (size_t i = 0; i + 1 < fp_flat.size(); i += 2) {
      fp_verts.push_back(Eigen::Vector2d(fp_flat[i], fp_flat[i + 1]));
    }
    if (fp_verts.empty()) {
      fp_verts.push_back(Eigen::Vector2d(0.25, 0.3));
      fp_verts.push_back(Eigen::Vector2d(0.25, -0.3));
      fp_verts.push_back(Eigen::Vector2d(-0.25, -0.3));
      fp_verts.push_back(Eigen::Vector2d(-0.25, -0.1));
      fp_verts.push_back(Eigen::Vector2d(-0.6, -0.1));
      fp_verts.push_back(Eigen::Vector2d(-0.6, 0.1));
      fp_verts.push_back(Eigen::Vector2d(-0.25, 0.1));
      fp_verts.push_back(Eigen::Vector2d(-0.25, 0.3));
    }
    footprint_.setPolygon(fp_verts);

    double inscribed_radius = 0.1;
    pnh_.param("inscribed_radius", inscribed_radius, 0.1);
    footprint_.setInscribedRadius(inscribed_radius);

    pnh_.param("paper/front_end_clearance",
               planner_params_.front_end_clearance,
               footprint_.circumscribedRadius() + 0.01);
    pnh_.param("paper/line_of_sight_clearance",
               planner_params_.line_of_sight_clearance,
               footprint_.circumscribedRadius() * 0.35);
    pnh_.param("paper/local_aabb_half_extent",
               planner_params_.local_aabb_half_extent,
               footprint_.circumscribedRadius() + 0.5);
    pnh_.param("paper/max_segment_length",
               planner_params_.max_segment_length, 1.1);
    pnh_.param("paper/max_support_points",
               planner_params_.max_support_points, 24);
    pnh_.param("paper/backend/max_iterations",
               planner_params_.backend.max_iterations, 40);
    pnh_.param("paper/backend/nominal_speed",
               planner_params_.backend.nominal_speed, 0.9);
    pnh_.param("paper/backend/min_piece_duration",
               planner_params_.backend.min_piece_duration, 0.35);
    pnh_.param("paper/backend/anchor_weight",
               planner_params_.backend.anchor_weight, 4.0);
    pnh_.param("paper/backend/smooth_weight",
               planner_params_.backend.smooth_weight, 4.0);
    pnh_.param("paper/backend/obstacle_weight",
               planner_params_.backend.obstacle_weight, 50.0);
    pnh_.param("paper/backend/safety_margin",
               planner_params_.backend.safety_margin, 0.03);
    pnh_.param("paper/backend/obstacle_cutoff_distance",
               planner_params_.backend.obstacle_cutoff_distance, 1.0);
    pnh_.param("paper/backend/sample_time_step",
               planner_params_.backend.sample_time_step, 0.04);
    pnh_.param("paper/backend/lmbm_max_iterations",
               planner_params_.backend.lmbm.max_iterations, 45);
    pnh_.param("paper/backend/lmbm_max_evaluations",
               planner_params_.backend.lmbm.max_evaluations, 250);
    pnh_.param("paper/backend/lmbm_timeout",
               planner_params_.backend.lmbm.timeout, 30.0f);
    pnh_.param("paper/backend/lmbm_bundle_size",
               planner_params_.backend.lmbm.bundle_size, 2);
    pnh_.param("paper/backend/lmbm_ini_corrections",
               planner_params_.backend.lmbm.ini_corrections, 7);
    pnh_.param("paper/backend/lmbm_max_corrections",
               planner_params_.backend.lmbm.max_corrections, 15);

    double sx = 1.0;
    double sy = 1.0;
    double syaw = 0.0;
    double gx = 8.0;
    double gy = 8.0;
    double gyaw = 0.0;
    pnh_.param("start_pose/x", sx, sx);
    pnh_.param("start_pose/y", sy, sy);
    pnh_.param("start_pose/yaw", syaw, syaw);
    pnh_.param("goal_pose/x", gx, gx);
    pnh_.param("goal_pose/y", gy, gy);
    pnh_.param("goal_pose/yaw", gyaw, gyaw);
    start_ = SE2State(sx, sy, syaw);
    goal_ = SE2State(gx, gy, gyaw);
    start_ready_ = use_start_goal_params_;
    goal_ready_ = use_start_goal_params_;

    if (use_start_goal_params_) {
      ROS_INFO("Configured startup start: (%.2f, %.2f, %.2f)",
               start_.x, start_.y, start_.yaw);
      ROS_INFO("Configured startup goal: (%.2f, %.2f, %.2f)",
               goal_.x, goal_.y, goal_.yaw);
    } else {
      ROS_INFO("Startup start/goal parameters disabled; waiting for RViz /initialpose and /move_base_simple/goal.");
    }
  }

  void setupPubSub() {
    map_sub_ = nh_.subscribe("/map", 1, &EsvPlannerNode::mapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &EsvPlannerNode::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EsvPlannerNode::goalCallback, this);

    traj_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/esv_planner/markers", 1, true);
    time_pub_ = nh_.advertise<std_msgs::Float64>("/esv_planner/solve_time", 1, true);
    stats_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/esv_planner/planning_stats", 1, true);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map: %dx%d, res=%.3f",
             msg->info.width, msg->info.height, msg->info.resolution);
    grid_map_.fromOccupancyGrid(msg);
    svsdf_evaluator_.initGeometryBodyFrame(grid_map_.geometryMap(), footprint_);
    planner_.init(grid_map_, footprint_, svsdf_evaluator_, planner_params_);
    map_ready_ = true;
    ROS_INFO("Map processed. Paper-aligned planner initialized.");

    if (start_ready_ && goal_ready_) {
      tryPlan();
    }
  }

  void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_.x = msg->pose.pose.position.x;
    start_.y = msg->pose.pose.position.y;
    start_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    start_ready_ = true;
    ROS_INFO("Start: (%.2f, %.2f, %.2f)", start_.x, start_.y, start_.yaw);
    if (map_ready_ && goal_ready_) {
      tryPlan();
    }
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = tf2::getYaw(msg->pose.orientation);
    goal_ready_ = true;
    ROS_INFO("Goal: (%.2f, %.2f, %.2f)", goal_.x, goal_.y, goal_.yaw);
    if (map_ready_ && start_ready_) {
      tryPlan();
    }
  }

  void tryPlan() {
    ROS_INFO("=== ESV Planner: Starting paper-aligned planning ===");
    const PaperPlannerResult result = planner_.plan(start_, goal_);

    publishTrajectory(result.trajectory);
    publishMarkers(result);
    publishStats(result.stats);

    std_msgs::Float64 time_msg;
    time_msg.data = result.stats.total_solve_time;
    time_pub_.publish(time_msg);

    if (result.success) {
      ROS_INFO("=== Planning done: %.3f s, coarse=%d support=%d local_obs=%d min_clearance=%.3f ===",
               result.stats.total_solve_time,
               result.stats.coarse_path_points,
               result.stats.support_points,
               result.stats.local_obstacle_points,
               result.stats.min_clearance);
    } else {
      ROS_WARN("Planning failed after %.3f s (coarse=%d support=%d local_obs=%d min_clearance=%.3f)",
               result.stats.total_solve_time,
               result.stats.coarse_path_points,
               result.stats.support_points,
               result.stats.local_obstacle_points,
               result.stats.min_clearance);
    }
  }

  void publishTrajectory(const Trajectory& traj) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    if (!traj.empty()) {
      const std::vector<SE2State> samples = sampleTrajectoryByArcLength(traj, 0.05, 0.02);
      for (size_t i = 0; i < samples.size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = samples[i].x;
        pose.pose.position.y = samples[i].y;
        pose.pose.orientation.z = std::sin(samples[i].yaw * 0.5);
        pose.pose.orientation.w = std::cos(samples[i].yaw * 0.5);
        path_msg.poses.push_back(pose);
      }
    }
    traj_pub_.publish(path_msg);
  }

  void publishStats(const PlanningStats& stats) {
    std_msgs::Float64MultiArray msg;
    msg.data = stats.asVector();
    stats_pub_.publish(msg);
  }

  visualization_msgs::Marker makeLineMarker(
      int id,
      const std::string& ns,
      float r,
      float g,
      float b,
      float a,
      double width) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  }

  visualization_msgs::Marker makeFootprintMarker(
      int id,
      const std::string& ns,
      const SE2State& state,
      float r,
      float g,
      float b) const {
    visualization_msgs::Marker marker = makeLineMarker(id, ns, r, g, b, 0.9f, 0.03);
    const std::vector<Eigen::Vector2d> verts = footprint_.rotatedVertices(state.yaw);
    for (size_t i = 0; i < verts.size(); ++i) {
      geometry_msgs::Point point;
      point.x = verts[i].x() + state.x;
      point.y = verts[i].y() + state.y;
      point.z = 0.15;
      marker.points.push_back(point);
    }
    if (!verts.empty()) {
      geometry_msgs::Point point;
      point.x = verts[0].x() + state.x;
      point.y = verts[0].y() + state.y;
      point.z = 0.15;
      marker.points.push_back(point);
    }
    return marker;
  }

  void publishMarkers(const PaperPlannerResult& result) {
    visualization_msgs::MarkerArray markers;
    int id = 0;

    visualization_msgs::Marker coarse =
        makeLineMarker(id++, "coarse_path", 0.3f, 0.5f, 1.0f, 0.8f, 0.03);
    for (size_t i = 0; i < result.coarse_path.size(); ++i) {
      geometry_msgs::Point point;
      point.x = result.coarse_path[i].x();
      point.y = result.coarse_path[i].y();
      point.z = 0.05;
      coarse.points.push_back(point);
    }
    markers.markers.push_back(coarse);

    visualization_msgs::Marker support =
        makeLineMarker(id++, "support_path", 1.0f, 0.6f, 0.0f, 0.9f, 0.05);
    for (size_t i = 0; i < result.support_states.size(); ++i) {
      geometry_msgs::Point point;
      point.x = result.support_states[i].x;
      point.y = result.support_states[i].y;
      point.z = 0.08;
      support.points.push_back(point);
    }
    markers.markers.push_back(support);

    visualization_msgs::Marker local_obs;
    local_obs.header.frame_id = "map";
    local_obs.header.stamp = ros::Time::now();
    local_obs.ns = "local_obstacles";
    local_obs.id = id++;
    local_obs.type = visualization_msgs::Marker::POINTS;
    local_obs.action = visualization_msgs::Marker::ADD;
    local_obs.scale.x = 0.04;
    local_obs.scale.y = 0.04;
    local_obs.color.r = 1.0f;
    local_obs.color.g = 0.1f;
    local_obs.color.b = 0.1f;
    local_obs.color.a = 0.6f;
    for (size_t i = 0; i < result.local_obstacles.size(); ++i) {
      geometry_msgs::Point point;
      point.x = result.local_obstacles[i].x();
      point.y = result.local_obstacles[i].y();
      local_obs.points.push_back(point);
    }
    markers.markers.push_back(local_obs);

    if (!result.trajectory.empty()) {
      visualization_msgs::Marker traj =
          makeLineMarker(id++, "best_traj", 0.0f, 1.0f, 0.0f, 1.0f, 0.05);
      const std::vector<SE2State> traj_samples =
          sampleTrajectoryByArcLength(result.trajectory, 0.05, 0.02);
      for (size_t i = 0; i < traj_samples.size(); ++i) {
        geometry_msgs::Point point;
        point.x = traj_samples[i].x;
        point.y = traj_samples[i].y;
        point.z = 0.1;
        traj.points.push_back(point);
      }
      markers.markers.push_back(traj);

      const std::vector<SE2State> fp_samples =
          sampleTrajectoryByArcLength(result.trajectory, 0.45, 0.02);
      for (size_t i = 0; i < fp_samples.size(); ++i) {
        markers.markers.push_back(
            makeFootprintMarker(id++, "footprint", fp_samples[i], 0.0f, 0.8f, 0.0f));
      }
    }

    markers.markers.push_back(
        makeFootprintMarker(id++, "start_goal", start_, 0.0f, 1.0f, 1.0f));
    markers.markers.push_back(
        makeFootprintMarker(id++, "start_goal", goal_, 1.0f, 0.0f, 1.0f));

    marker_pub_.publish(markers);
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
