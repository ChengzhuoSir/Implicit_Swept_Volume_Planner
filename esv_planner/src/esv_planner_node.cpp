#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <cmath>
#include <string>

#include "esv_planner/svsdf_runtime.h"

namespace esv_planner {

class EsvPlannerNode {
 public:
  EsvPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    LoadParams();
    runtime_.Initialize(nh_, pnh_);
    ROS_INFO("ESV Planner coarse-to-fine runtime initialized.");
    WarmupInitialMap();
    SetupPubSub();
    TryPlan();
  }

 private:
  void LoadParams() {
    pnh_.param("use_start_goal_params", use_start_goal_params_, true);
    pnh_.param("startup_wait_for_map", startup_wait_for_map_, true);
    pnh_.param("startup_map_wait_timeout", startup_map_wait_timeout_sec_, 0.0);
    pnh_.param("startup_map_wait_poll", startup_map_wait_poll_sec_, 1.0);
    if (startup_map_wait_poll_sec_ < 0.1) {
      startup_map_wait_poll_sec_ = 0.1;
    }

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

  bool SameMap(const nav_msgs::OccupancyGrid& lhs,
               const nav_msgs::OccupancyGrid& rhs) const {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width &&
           lhs.info.height == rhs.info.height &&
           std::abs(lhs.info.resolution - rhs.info.resolution) < 1e-9 &&
           std::abs(lhs.info.origin.position.x - rhs.info.origin.position.x) < 1e-9 &&
           std::abs(lhs.info.origin.position.y - rhs.info.origin.position.y) < 1e-9 &&
           std::abs(lhs.info.origin.position.z - rhs.info.origin.position.z) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.x - rhs.info.origin.orientation.x) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.y - rhs.info.origin.orientation.y) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.z - rhs.info.origin.orientation.z) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.w - rhs.info.origin.orientation.w) < 1e-9 &&
           lhs.data == rhs.data;
  }

  void WarmupInitialMap() {
    if (!startup_wait_for_map_) {
      ROS_INFO("Startup warmup: disabled; planner will wait for /map through subscriber callbacks.");
      return;
    }

    if (startup_map_wait_timeout_sec_ > 0.0) {
      ROS_INFO("Startup warmup: waiting up to %.1fs for the first /map before enabling planning.",
               startup_map_wait_timeout_sec_);
    } else {
      ROS_INFO("Startup warmup: waiting for the first /map before enabling planning.");
    }

    const ros::WallTime wait_start = ros::WallTime::now();
    while (ros::ok()) {
      const double elapsed = (ros::WallTime::now() - wait_start).toSec();
      double poll_timeout_sec = startup_map_wait_poll_sec_;
      if (startup_map_wait_timeout_sec_ > 0.0) {
        const double remaining = startup_map_wait_timeout_sec_ - elapsed;
        if (remaining <= 1e-6) {
          ROS_WARN("Startup warmup: timed out after %.3fs waiting for /map; falling back to normal subscriber-driven map updates.",
                   elapsed);
          return;
        }
        if (remaining < poll_timeout_sec) {
          poll_timeout_sec = remaining;
        }
      }

      nav_msgs::OccupancyGrid::ConstPtr msg =
          ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
              "/map", nh_, ros::Duration(poll_timeout_sec));
      if (!msg) {
        ROS_INFO("Startup warmup: still waiting for /map after %.1fs",
                 (ros::WallTime::now() - wait_start).toSec());
        continue;
      }

      const double wait_time = (ros::WallTime::now() - wait_start).toSec();
      ROS_INFO("Startup warmup: received initial /map after %.3fs (%u x %u @ %.3f m); updating runtime and warming topology cache.",
               wait_time, msg->info.width, msg->info.height, msg->info.resolution);

      const ros::WallTime update_start = ros::WallTime::now();
      if (!runtime_.UpdateMap(*msg)) {
        ROS_ERROR("Startup warmup: failed to update planner state from the initial /map; continuing to wait for another map message.");
        continue;
      }

      map_ready_ = true;
      startup_map_ = *msg;
      skip_duplicate_startup_map_ = true;
      const double update_time = (ros::WallTime::now() - update_start).toSec();
      ROS_INFO("Startup warmup: initial /map integrated in %.3fs. Map preload complete; planner is ready%s",
               update_time,
               use_start_goal_params_ ? " to plan with parameter start/goal."
                                      : " for RViz start/goal input.");
      return;
    }

    ROS_WARN("Startup warmup: ROS shutdown requested before the first /map arrived.");
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
    if (skip_duplicate_startup_map_ && SameMap(*msg, startup_map_)) {
      skip_duplicate_startup_map_ = false;
      ROS_INFO("Map callback: skipping duplicate startup /map already preloaded during warmup.");
      return;
    }
    skip_duplicate_startup_map_ = false;

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
  bool startup_wait_for_map_ = true;
  double startup_map_wait_timeout_sec_ = 0.0;
  double startup_map_wait_poll_sec_ = 1.0;
  bool skip_duplicate_startup_map_ = false;
  nav_msgs::OccupancyGrid startup_map_;
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
