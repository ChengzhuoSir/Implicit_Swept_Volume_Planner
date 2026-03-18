#include "esv_planner/strict_frontend.h"

#include <algorithm>
#include <cmath>
#include <set>

#include <ros/package.h>

namespace esv_planner {

namespace {

Eigen::Vector3d ToStrictState(const SE2State& state) {
  return Eigen::Vector3d(state.x, state.y, state.yaw);
}

std::string ResolveMeshPath() {
  const std::string package_path = ros::package::getPath("esv_planner");
  if (!package_path.empty()) {
    return package_path + "/meshes/robot_t_shape.obj";
  }
  return "src/esv_planner/meshes/robot_t_shape.obj";
}

void AssignYaw(std::vector<TopoWaypoint>* waypoints) {
  if (waypoints == nullptr || waypoints->empty()) {
    return;
  }
  for (size_t i = 0; i < waypoints->size(); ++i) {
    Eigen::Vector2d delta = Eigen::Vector2d::Zero();
    if (i + 1 < waypoints->size()) {
      delta = (*waypoints)[i + 1].pos - (*waypoints)[i].pos;
    } else if (i > 0) {
      delta = (*waypoints)[i].pos - (*waypoints)[i - 1].pos;
    }
    (*waypoints)[i].yaw =
        delta.norm() > 1e-9 ? std::atan2(delta.y(), delta.x()) : 0.0;
    (*waypoints)[i].has_yaw = true;
  }
}

}  // namespace

StrictFrontend::StrictFrontend() {}

Config StrictFrontend::BuildDefaultConfig() const {
  Config config;
  config.inputdata = ResolveMeshPath();
  config.poly_params = std::vector<double>(3, 0.0);
  config.meshTopic = "/esv_planner/mesh";
  config.edgeTopic = "/esv_planner/mesh_edges";
  config.vertexTopic = "/esv_planner/mesh_vertices";
  config.occupancy_resolution = 0.10;
  config.kernel_size = 7;
  config.kernel_yaw_num = 18;
  config.eps = 0.08;
  config.debug_output = false;
  config.sta_threshold = 1;
  config.front_end_safeh = 0.05;
  config.vehicleMass = 0.61;
  config.gravAcc = 9.8;
  config.horizDrag = 0.10;
  config.vertDrag = 0.10;
  config.parasDrag = 0.01;
  config.speedEps = 0.0001;
  config.kernel_size = 7;
  config.kernel_yaw_num = 18;
  return config;
}

void StrictFrontend::LoadConfigOverrides(ros::NodeHandle& pnh) {
  pnh.param("inputdata", config_.inputdata, config_.inputdata);
  pnh.param("meshTopic", config_.meshTopic, config_.meshTopic);
  pnh.param("edgeTopic", config_.edgeTopic, config_.edgeTopic);
  pnh.param("vertexTopic", config_.vertexTopic, config_.vertexTopic);
  pnh.param("occupancy_resolution", config_.occupancy_resolution,
            config_.occupancy_resolution);
  pnh.param("poly_params", config_.poly_params, config_.poly_params);
  pnh.param("kernel_size", config_.kernel_size, config_.kernel_size);
  pnh.param("kernel_yaw_num", config_.kernel_yaw_num, config_.kernel_yaw_num);
  pnh.param("eps", config_.eps, config_.eps);
  pnh.param("debug_output", config_.debug_output, config_.debug_output);
  pnh.param("sta_threshold", config_.sta_threshold, config_.sta_threshold);
  pnh.param("front_end_safeh", config_.front_end_safeh, config_.front_end_safeh);
  pnh.param("vehicleMass", config_.vehicleMass, config_.vehicleMass);
  pnh.param("gravAcc", config_.gravAcc, config_.gravAcc);
  pnh.param("horizDrag", config_.horizDrag, config_.horizDrag);
  pnh.param("vertDrag", config_.vertDrag, config_.vertDrag);
  pnh.param("parasDrag", config_.parasDrag, config_.parasDrag);
  pnh.param("speedEps", config_.speedEps, config_.speedEps);
}

void StrictFrontend::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                const FootprintModel& footprint) {
  (void)footprint;
  config_ = BuildDefaultConfig();
  LoadConfigOverrides(pnh);
  ConfigureModules(true);
}

void StrictFrontend::InitializeForTesting(const FootprintModel& footprint) {
  (void)footprint;
  config_ = BuildDefaultConfig();
  ConfigureModules(false);
}

void StrictFrontend::ConfigureModules(bool enable_ros_io) {
  pcs_map_manager_.reset(new PCSmapManager(config_));
  swept_volume_manager_.reset(new SweptVolumeManager(config_));
  astar_.reset(new AstarPathSearcher);

  if (enable_ros_io) {
    ros::NodeHandle nh;
    swept_volume_manager_->init(nh, config_);
    astar_->init(nh);
  } else {
    swept_volume_manager_->flatness.reset(
        config_.vehicleMass, config_.gravAcc, config_.horizDrag,
        config_.vertDrag, config_.parasDrag, config_.speedEps);
    swept_volume_manager_->initShape(config_);
  }
  astar_->kernel_size = config_.kernel_size;
  initialized_ = true;
}

bool StrictFrontend::UpdateMap(const nav_msgs::OccupancyGrid& map) {
  latest_map_ = map;
  return ResetSearchMap(map);
}

bool StrictFrontend::ResetSearchMap(const nav_msgs::OccupancyGrid& map) {
  if (!initialized_) {
    return false;
  }
  if (!OccupancyGridBridge::Populate(map, pcs_map_manager_.get())) {
    return false;
  }
  pcs_map_manager_->occupancy_map->generateESDF3d();
  uint8_t* kernel = pcs_map_manager_->generateMapKernel2D(config_.kernel_size);
  swept_volume_manager_->setMapKernel(
      kernel, pcs_map_manager_->occupancy_map->X_size,
      pcs_map_manager_->occupancy_map->Y_size,
      pcs_map_manager_->occupancy_map->Z_size);
  astar_->initGridMap(pcs_map_manager_, swept_volume_manager_);
  ready_ = true;
  return true;
}

bool StrictFrontend::BuildCroppedMap(const SE2State& start, const SE2State& goal,
                                     nav_msgs::OccupancyGrid* cropped) const {
  if (cropped == nullptr || latest_map_.info.width == 0 || latest_map_.info.height == 0) {
    return false;
  }

  const double margin = 4.0;
  const double resolution = latest_map_.info.resolution;
  const double origin_x = latest_map_.info.origin.position.x;
  const double origin_y = latest_map_.info.origin.position.y;
  const double min_x = std::min(start.x, goal.x) - margin;
  const double max_x = std::max(start.x, goal.x) + margin;
  const double min_y = std::min(start.y, goal.y) - margin;
  const double max_y = std::max(start.y, goal.y) + margin;

  int x0 = static_cast<int>(std::floor((min_x - origin_x) / resolution));
  int x1 = static_cast<int>(std::ceil((max_x - origin_x) / resolution));
  int y0 = static_cast<int>(std::floor((min_y - origin_y) / resolution));
  int y1 = static_cast<int>(std::ceil((max_y - origin_y) / resolution));

  x0 = std::max(0, x0);
  y0 = std::max(0, y0);
  x1 = std::min(static_cast<int>(latest_map_.info.width) - 1, x1);
  y1 = std::min(static_cast<int>(latest_map_.info.height) - 1, y1);
  if (x0 >= x1 || y0 >= y1) {
    return false;
  }

  const int cropped_width = x1 - x0 + 1;
  const int cropped_height = y1 - y0 + 1;
  if (cropped_width >= static_cast<int>(0.95 * latest_map_.info.width) &&
      cropped_height >= static_cast<int>(0.95 * latest_map_.info.height)) {
    return false;
  }

  cropped->header = latest_map_.header;
  cropped->info = latest_map_.info;
  cropped->info.width = cropped_width;
  cropped->info.height = cropped_height;
  cropped->info.origin.position.x = origin_x + static_cast<double>(x0) * resolution;
  cropped->info.origin.position.y = origin_y + static_cast<double>(y0) * resolution;
  cropped->data.assign(static_cast<size_t>(cropped_width * cropped_height), 0);

  for (int y = 0; y < cropped_height; ++y) {
    const int src_y = y0 + y;
    const size_t src_offset = static_cast<size_t>(src_y) * latest_map_.info.width;
    const size_t dst_offset = static_cast<size_t>(y) * cropped_width;
    for (int x = 0; x < cropped_width; ++x) {
      cropped->data[dst_offset + x] = latest_map_.data[src_offset + (x0 + x)];
    }
  }

  return true;
}

bool StrictFrontend::SearchSingle(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& goal,
                                  std::vector<Eigen::Vector3d>* path) {
  if (path == nullptr) {
    return false;
  }
  astar_->AstarPathSearch(start, goal);
  if (!astar_->success_flag) {
    astar_->reset();
    return false;
  }
  *path = astar_->getPath();
  astar_->reset();
  return !path->empty();
}

TopoPath StrictFrontend::MakeTopoPath(
    const std::vector<Eigen::Vector3d>& path) const {
  TopoPath topo;
  for (size_t i = 0; i < path.size(); ++i) {
    TopoWaypoint waypoint(path[i].head<2>(), path[i].z());
    waypoint.has_yaw = true;
    topo.waypoints.push_back(waypoint);
  }
  AssignYaw(&topo.waypoints);
  topo.computeLength();
  return topo;
}

bool StrictFrontend::TransitionCollisionFree(const Eigen::Vector3d& from,
                                             const Eigen::Vector3d& to) const {
  std::vector<Eigen::Vector2d> aabb_points;
  const Eigen::Vector3d midpoint = 0.5 * (from + to);
  const double half_x = std::abs(to.x() - from.x()) * 0.5 + config_.kernel_size;
  const double half_y = std::abs(to.y() - from.y()) * 0.5 + config_.kernel_size;
  pcs_map_manager_->getPointsInAABB2D(midpoint, half_x, half_y, aabb_points);
  return swept_volume_manager_->checkSubSWCollision(from, to, aabb_points);
}

TopoPath StrictFrontend::ShortcutPath(
    const std::vector<Eigen::Vector3d>& path) const {
  if (path.size() < 2) {
    return TopoPath();
  }

  std::vector<Eigen::Vector3d> shortened;
  shortened.push_back(path.front());
  size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    size_t best = anchor + 1;
    for (size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
      if (TransitionCollisionFree(path[anchor], path[candidate])) {
        best = candidate;
        break;
      }
    }
    shortened.push_back(path[best]);
    anchor = best;
  }
  return MakeTopoPath(shortened);
}

std::vector<TopoPath> StrictFrontend::Search(const SE2State& start,
                                             const SE2State& goal) {
  std::vector<TopoPath> candidates;
  if (!ready_) {
    return candidates;
  }

  std::vector<Eigen::Vector3d> base_path;
  nav_msgs::OccupancyGrid cropped_map;
  bool search_ok = false;
  const bool has_cropped_map =
      BuildCroppedMap(start, goal, &cropped_map) && ResetSearchMap(cropped_map);
  if (has_cropped_map) {
    ROS_INFO("Strict front-end: cropped search map %u x %u",
             cropped_map.info.width, cropped_map.info.height);
    search_ok = SearchSingle(ToStrictState(start), ToStrictState(goal), &base_path);
  }
  if (!search_ok && !has_cropped_map) {
    ROS_WARN("Strict front-end: cropped search failed, retrying on full map");
    ResetSearchMap(latest_map_);
    search_ok = SearchSingle(ToStrictState(start), ToStrictState(goal), &base_path);
  } else if (!search_ok) {
    ROS_WARN("Strict front-end: cropped search failed within budget");
  }
  if (!search_ok) {
    return candidates;
  }

  candidates.push_back(MakeTopoPath(base_path));
  const TopoPath shortcut = ShortcutPath(base_path);
  if (shortcut.waypoints.size() >= 2 &&
      (candidates.front().waypoints.size() != shortcut.waypoints.size() ||
       std::abs(candidates.front().length - shortcut.length) > 1e-6)) {
    candidates.push_back(shortcut);
  }

  if (candidates.size() < 2 && base_path.size() >= 3) {
    std::vector<Eigen::Vector3d> sparse_path;
    sparse_path.push_back(base_path.front());
    sparse_path.push_back(base_path[base_path.size() / 2]);
    sparse_path.push_back(base_path.back());
    candidates.push_back(MakeTopoPath(sparse_path));
  }

  return candidates;
}

}  // namespace esv_planner
