#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <string>

#include "esv_planner/svsdf_runtime.h"

namespace esv_planner {

class EsvPlannerNode {
 public:
  EsvPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    LoadParams();
    runtime_.Initialize(nh_, pnh_);
    SetupPubSub();
    ROS_INFO("ESV Planner coarse-to-fine runtime initialized.");
  }

 private:
  void LoadParams() {
    pnh_.param("use_start_goal_params", use_start_goal_params_, true);
    start_ = LoadPoseParam("start_pose");
    goal_ = LoadPoseParam("goal_pose");
    start_ready_ = use_start_goal_params_;
    goal_ready_ = use_start_goal_params_;
    if (use_start_goal_params_) {
      ROS_INFO("Using parameter start/goal: start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f)",
               start_.x(), start_.y(), start_.z(), goal_.x(), goal_.y(), goal_.z());
    }
  }

  Eigen::Vector3d LoadPoseParam(const std::string& key) {
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();
    pnh_.param(key + "/x", pose.x(), 0.0);
    pnh_.param(key + "/y", pose.y(), 0.0);
    pnh_.param(key + "/yaw", pose.z(), 0.0);
    return pose;
  }

  void SetupPubSub() {
    map_sub_ = nh_.subscribe("/map", 1, &EsvPlannerNode::MapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &EsvPlannerNode::StartCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EsvPlannerNode::GoalCallback, this);

    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory", 1, true);
    coarse_path_pub_ = nh_.advertise<nav_msgs::Path>("/esv_planner/trajectory_astar", 1, true);
    time_pub_ = nh_.advertise<std_msgs::Float64>("/esv_planner/planning_time", 1, true);
    stats_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/esv_planner/planning_stats", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/esv_planner/markers", 1, true);

    if (!use_start_goal_params_) {
      ROS_INFO("Waiting for RViz /initialpose and /move_base_simple/goal.");
    }
  }

  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!runtime_.UpdateMap(*msg)) {
      ROS_ERROR("Failed to update planner map.");
      return;
    }
    map_ready_ = true;
    ROS_INFO("Map received: %u x %u @ %.3f m", msg->info.width, msg->info.height, msg->info.resolution);
    TryPlan();
  }

  void StartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_.x() = msg->pose.pose.position.x;
    start_.y() = msg->pose.pose.position.y;
    start_.z() = tf2::getYaw(msg->pose.pose.orientation);
    start_ready_ = true;
    use_start_goal_params_ = false;
    ROS_INFO("Start: (%.2f, %.2f, %.2f)", start_.x(), start_.y(), start_.z());
    TryPlan();
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x() = msg->pose.position.x;
    goal_.y() = msg->pose.position.y;
    goal_.z() = tf2::getYaw(msg->pose.orientation);
    goal_ready_ = true;
    use_start_goal_params_ = false;
    ROS_INFO("Goal: (%.2f, %.2f, %.2f)", goal_.x(), goal_.y(), goal_.z());
    TryPlan();
  }

  void TryPlan() {
    if (!map_ready_ || !start_ready_ || !goal_ready_ || planning_) {
      return;
    }
    planning_ = true;
    RunPlanning();
    planning_ = false;
  }

  void RunPlanning() {
    ROS_INFO("=== ESV Planner: Starting coarse-to-fine planning ===");
    const SvsdfPlanResult result = runtime_.Plan(start_, goal_);

    coarse_path_pub_.publish(result.coarse_path);
    trajectory_pub_.publish(result.trajectory);
    PublishStats(result.stats);
    PublishPlanningTime(result.stats.total_solve_time);
    PublishEmptyMarkers();

    if (result.success) {
      ROS_INFO("Planning done: %.3f s, coarse=%d support=%d local_obs=%d min_clearance=%.3f",
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

  void PublishPlanningTime(double seconds) {
    std_msgs::Float64 msg;
    msg.data = seconds;
    time_pub_.publish(msg);
  }

  void PublishStats(const PlanningStats& stats) {
    std_msgs::Float64MultiArray msg;
    msg.data = stats.asVector();
    stats_pub_.publish(msg);
  }

  void PublishEmptyMarkers() {
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    marker_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher coarse_path_pub_;
  ros::Publisher time_pub_;
  ros::Publisher stats_pub_;
  ros::Publisher marker_pub_;

  SvsdfRuntime runtime_;
  Eigen::Vector3d start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  bool use_start_goal_params_ = true;
  bool map_ready_ = false;
  bool start_ready_ = false;
  bool goal_ready_ = false;
  bool planning_ = false;
};

}  // namespace esv_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "esv_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  esv_planner::EsvPlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
