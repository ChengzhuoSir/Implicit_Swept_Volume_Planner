#include "esv_planner/ros_interface/ros_visualizer.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace esv_planner {

nav_msgs::Path RosVisualizer::MakePath(const std::vector<SE2State>& states) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for (size_t i = 0; i < states.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = states[i].x;
    pose.pose.position.y = states[i].y;
    pose.pose.orientation.z = std::sin(states[i].yaw * 0.5);
    pose.pose.orientation.w = std::cos(states[i].yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::Path RosVisualizer::MakePathFromTopo(const TopoPath& path_in) {
  std::vector<SE2State> states;
  states.reserve(path_in.waypoints.size());
  for (size_t i = 0; i < path_in.waypoints.size(); ++i) {
    const TopoWaypoint& wp = path_in.waypoints[i];
    states.push_back(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  }
  return MakePath(states);
}

nav_msgs::Path RosVisualizer::MakeTrajectoryPath(const Trajectory& traj) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  if (traj.empty()) {
    ROS_WARN("MakeTrajectoryPath: traj is empty!");
    return path;
  }

  const double total = traj.totalDuration();
  ROS_INFO("MakeTrajectoryPath: pieces=%zu total_duration=%.4f", traj.pos_pieces.size(), total);
  if (!traj.pos_pieces.empty()) {
    SE2State s0 = traj.sample(0.0);
    SE2State sf = traj.sample(std::max(0.0, total - 0.01));
    ROS_INFO("MakeTrajectoryPath: start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)",
             s0.x, s0.y, s0.yaw, sf.x, sf.y, sf.yaw);
  }

  const double min_sample_dt = 0.05;
  const double max_sample_dt = 0.40;
  const size_t min_pose_count = 1000;
  const size_t max_pose_count = 2500;
  const size_t target_pose_count =
      std::max(min_pose_count,
               std::min(max_pose_count,
                        traj.pos_pieces.size() * static_cast<size_t>(2)));
  const double sample_dt =
      total > 1e-9
          ? std::min(max_sample_dt,
                     std::max(min_sample_dt,
                              total / static_cast<double>(target_pose_count)))
          : min_sample_dt;
  const size_t estimated_pose_count =
      total > 1e-9 ? static_cast<size_t>(std::ceil(total / sample_dt)) + 1 : 1;
  path.poses.reserve(estimated_pose_count);
  ROS_INFO("MakeTrajectoryPath: sample_dt=%.3f target_poses=%zu estimated_poses=%zu",
           sample_dt, target_pose_count, estimated_pose_count);

  for (double t = 0.0; t < total; t += sample_dt) {
    const SE2State state = traj.sample(t);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.z = std::sin(state.yaw * 0.5);
    pose.pose.orientation.w = std::cos(state.yaw * 0.5);
    path.poses.push_back(pose);
  }

  const SE2State final_state = traj.sample(total);
  geometry_msgs::PoseStamped final_pose;
  final_pose.header = path.header;
  final_pose.pose.position.x = final_state.x;
  final_pose.pose.position.y = final_state.y;
  final_pose.pose.orientation.z = std::sin(final_state.yaw * 0.5);
  final_pose.pose.orientation.w = std::cos(final_state.yaw * 0.5);
  path.poses.push_back(final_pose);
  return path;
}

visualization_msgs::MarkerArray RosVisualizer::MakeFootprintMarkers(
    const Trajectory& traj, 
    const FootprintModel& footprint, 
    const std::string& frame_id,
    double dt) {
    
  visualization_msgs::MarkerArray msg;
  
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(delete_marker);
  
  if (traj.empty()) {
    return msg;
  }

  const double total = traj.totalDuration();
  int marker_id = 0;
  
  for (double t = 0.0; t <= total + 1e-9; t += dt) {
    const SE2State state = traj.sample(std::min(t, total));
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "footprint";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.02; 
    marker.color.a = 0.6;  
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    const std::vector<Eigen::Vector2d>& local_poly = footprint.vertices();
    for (const Eigen::Vector2d& pt : local_poly) {
      double wx = state.x + pt.x() * std::cos(state.yaw) - pt.y() * std::sin(state.yaw);
      double wy = state.y + pt.x() * std::sin(state.yaw) + pt.y() * std::cos(state.yaw);
      geometry_msgs::Point p;
      p.x = wx;
      p.y = wy;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    if (!local_poly.empty()) {
      const Eigen::Vector2d& pt = local_poly.front();
      double wx = state.x + pt.x() * std::cos(state.yaw) - pt.y() * std::sin(state.yaw);
      double wy = state.y + pt.x() * std::sin(state.yaw) + pt.y() * std::cos(state.yaw);
      geometry_msgs::Point p;
      p.x = wx;
      p.y = wy;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    msg.markers.push_back(marker);
    }

    return msg;
    }

    }  // namespace esv_planner
