#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <Eigen/Core>

#include "esv_planner/common.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/occupancy_grid_bridge.h"
#include "map_manager/PCSmap_manager.h"
#include "planner_algorithm/front_end_Astar.hpp"
#include "swept_volume/sw_manager.hpp"
#include "utils/config.hpp"

namespace esv_planner {

class StrictFrontend {
 public:
  StrictFrontend();

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                  const FootprintModel& footprint);
  void InitializeForTesting(const FootprintModel& footprint);
  bool UpdateMap(const nav_msgs::OccupancyGrid& map);
  std::vector<TopoPath> Search(const SE2State& start, const SE2State& goal);

 private:
  Config BuildDefaultConfig() const;
  void LoadConfigOverrides(ros::NodeHandle& pnh);
  void ConfigureModules(bool enable_ros_io);
  bool ResetSearchMap(const nav_msgs::OccupancyGrid& map);
  bool BuildCroppedMap(const SE2State& start, const SE2State& goal,
                       nav_msgs::OccupancyGrid* cropped) const;
  TopoPath MakeTopoPath(const std::vector<Eigen::Vector3d>& path) const;
  TopoPath ShortcutPath(const std::vector<Eigen::Vector3d>& path) const;
  bool TransitionCollisionFree(const Eigen::Vector3d& from,
                               const Eigen::Vector3d& to) const;
  bool SearchSingle(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                    std::vector<Eigen::Vector3d>* path);

  Config config_;
  PCSmapManager::Ptr pcs_map_manager_;
  SweptVolumeManager::Ptr swept_volume_manager_;
  std::shared_ptr<AstarPathSearcher> astar_;
  nav_msgs::OccupancyGrid latest_map_;

  bool initialized_ = false;
  bool ready_ = false;
};

}  // namespace esv_planner
