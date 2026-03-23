#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <memory>

#include "isweep_planner/core/common.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/ros_interface/occupancy_grid_bridge.h"
#include "isweep_planner/env/map_manager/PCSmap_manager.h"
#include "isweep_planner/optimization/svsdf_backend/planner_algorithm/back_end_optimizer.hpp"
#include "isweep_planner/optimization/svsdf_backend/planner_algorithm/mid_end.hpp"
#include "isweep_planner/env/swept_volume/sw_manager.hpp"
#include "isweep_planner/optimization/svsdf_backend/utils/config.hpp"

namespace isweep_planner {

class Se2SvsdfSolver {
 public:
  Se2SvsdfSolver();

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                  const FootprintModel& footprint);
  void InitializeForTesting(const FootprintModel& footprint);
  bool UpdateMap(const nav_msgs::OccupancyGrid& map);
  bool Solve(const std::vector<SE2State>& segment, Trajectory* trajectory,
             bool preserve_shape = false);
  void setGridMapForMidend(const GridMap* map);

  bool ready() const { return ready_; }

 private:
  Config BuildDefaultConfig() const;
  void LoadConfigOverrides(ros::NodeHandle& pnh);
  void ConfigureModules(bool enable_ros_io);
  bool BuildSupportData(const std::vector<SE2State>& segment,
                        Eigen::Matrix3d* init_state,
                        Eigen::Matrix3d* final_state,
                        std::vector<Eigen::Vector3d>* points,
                        std::vector<Eigen::Vector3d>* accelerations,
                        std::vector<Eigen::Matrix3d>* rotations,
                        Eigen::VectorXd* initial_times) const;
  Trajectory ConvertTrajectory(const ::Trajectory<TRAJ_ORDER>& strict_traj) const;

  std::shared_ptr<ros::NodeHandle> nh_;
  Config config_;
  PCSmapManager::Ptr pcs_map_manager_;
  SweptVolumeManager::Ptr swept_volume_manager_;
  std::shared_ptr<OriTraj> midend_;
  std::shared_ptr<TrajOptimizer> backend_;

  bool initialized_ = false;
  bool ready_ = false;
};

}  // namespace isweep_planner
