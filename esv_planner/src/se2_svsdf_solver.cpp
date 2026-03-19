#include "esv_planner/se2_svsdf_solver.h"

#include <algorithm>
#include <cmath>

#include <ros/package.h>

#include "utils/debug_publisher.hpp"

namespace esv_planner {

namespace {

Eigen::Matrix3d MakeRotation(double yaw) {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = std::cos(yaw);
  rotation(0, 1) = -std::sin(yaw);
  rotation(1, 0) = std::sin(yaw);
  rotation(1, 1) = std::cos(yaw);
  return rotation;
}

Eigen::Vector3d ToStrictPoint(const SE2State& state) {
  return Eigen::Vector3d(state.x, state.y, state.yaw);
}

std::string ResolveMeshPath() {
  const std::string package_path = ros::package::getPath("esv_planner");
  if (!package_path.empty()) {
    return package_path + "/meshes/robot_t_shape.obj";
  }
  return "src/esv_planner/meshes/robot_t_shape.obj";
}

}  // namespace

Se2SvsdfSolver::Se2SvsdfSolver() {}

Config Se2SvsdfSolver::BuildDefaultConfig() const {
  Config config;
  config.threads_num = 8;
  config.poly_params = std::vector<double>(3, 0.0);
  config.mapBound = {-10.0, 20.0, -5.0, 55.0, -0.1, 0.1};
  config.voxelWidth = 0.10;
  config.testRate = 0.0;
  config.ts = 2.0;
  config.meshTopic = "/esv_planner/mesh";
  config.edgeTopic = "/esv_planner/mesh_edges";
  config.vertexTopic = "/esv_planner/mesh_vertices";
  config.inputdata = ResolveMeshPath();
  config.t_min = 0.0;
  config.t_max = 1.0;
  config.transparency = 1.0;
  config.momentum = 0.0;
  config.occupancy_resolution = 0.10;
  config.debug_output = false;
  config.sta_threshold = 1;
  config.selfmapresu = 0.1;
  config.colli_thres = 0.08;
  config.weight_v = 8.0;
  config.weight_a = 4.0;
  config.weight_p = 24.0;
  config.weight_pr = 18.0;
  config.weight_ar = 0.0;
  config.weight_omg = 6.0;
  config.weight_theta = 6.0;
  config.safety_hor = 0.08;
  config.vmax = 1.2;
  config.omgmax = 2.5;
  config.thetamax = 6.0;
  config.rho = 2.2;
  config.rho_mid_end = 1.5;
  config.inittime = 0.9;
  config.mem_size = 16;
  config.past = 64;
  config.min_step = 1.0e-32;
  config.g_epsilon = 0.0;
  config.RelCostTol = 1.0e-8;
  config.vehicleMass = 0.61;
  config.gravAcc = 9.8;
  config.horizDrag = 0.10;
  config.vertDrag = 0.10;
  config.parasDrag = 0.01;
  config.speedEps = 0.0001;
  config.smoothingEps = 1.0e-2;
  config.integralIntervs = 4;
  config.relCostTol = 1.0e-8;
  config.relCostTolMidEnd = 1.0e-8;
  config.eps = 0.08;
  config.loadStartEnd = false;
  config.kernel_size = 7;
  config.kernel_yaw_num = 18;
  config.front_end_safeh = 0.05;
  config.enableearlyExit = false;
  config.debugpause = 1;
  return config;
}

void Se2SvsdfSolver::LoadConfigOverrides(ros::NodeHandle& pnh) {
  pnh.param("meshTopic", config_.meshTopic, config_.meshTopic);
  pnh.param("edgeTopic", config_.edgeTopic, config_.edgeTopic);
  pnh.param("vertexTopic", config_.vertexTopic, config_.vertexTopic);
  pnh.param("inputdata", config_.inputdata, config_.inputdata);
  pnh.param("occupancy_resolution", config_.occupancy_resolution,
            config_.occupancy_resolution);
  pnh.param("kernel_size", config_.kernel_size, config_.kernel_size);
  pnh.param("kernel_yaw_num", config_.kernel_yaw_num, config_.kernel_yaw_num);
  pnh.param("threads_num", config_.threads_num, config_.threads_num);
  pnh.param("debug_output", config_.debug_output, config_.debug_output);
  pnh.param("sta_threshold", config_.sta_threshold, config_.sta_threshold);
  pnh.param("colli_thres", config_.colli_thres, config_.colli_thres);
  pnh.param("weight_v", config_.weight_v, config_.weight_v);
  pnh.param("weight_a", config_.weight_a, config_.weight_a);
  pnh.param("weight_p", config_.weight_p, config_.weight_p);
  pnh.param("weight_pr", config_.weight_pr, config_.weight_pr);
  pnh.param("weight_ar", config_.weight_ar, config_.weight_ar);
  pnh.param("weight_omg", config_.weight_omg, config_.weight_omg);
  pnh.param("weight_theta", config_.weight_theta, config_.weight_theta);
  pnh.param("safety_hor", config_.safety_hor, config_.safety_hor);
  pnh.param("vmax", config_.vmax, config_.vmax);
  pnh.param("omgmax", config_.omgmax, config_.omgmax);
  pnh.param("thetamax", config_.thetamax, config_.thetamax);
  pnh.param("rho", config_.rho, config_.rho);
  pnh.param("rho_mid_end", config_.rho_mid_end, config_.rho_mid_end);
  pnh.param("inittime", config_.inittime, config_.inittime);
  pnh.param("mem_size", config_.mem_size, config_.mem_size);
  pnh.param("past", config_.past, config_.past);
  pnh.param("min_step", config_.min_step, config_.min_step);
  pnh.param("g_epsilon", config_.g_epsilon, config_.g_epsilon);
  pnh.param("relCostTol", config_.relCostTol, config_.relCostTol);
  pnh.param("relCostTolMidEnd", config_.relCostTolMidEnd, config_.relCostTolMidEnd);
  pnh.param("eps", config_.eps, config_.eps);
  pnh.param("smoothingEps", config_.smoothingEps, config_.smoothingEps);
  pnh.param("integralIntervs", config_.integralIntervs, config_.integralIntervs);
  pnh.param("vehicleMass", config_.vehicleMass, config_.vehicleMass);
  pnh.param("gravAcc", config_.gravAcc, config_.gravAcc);
  pnh.param("horizDrag", config_.horizDrag, config_.horizDrag);
  pnh.param("vertDrag", config_.vertDrag, config_.vertDrag);
  pnh.param("parasDrag", config_.parasDrag, config_.parasDrag);
  pnh.param("speedEps", config_.speedEps, config_.speedEps);
  pnh.param("front_end_safeh", config_.front_end_safeh, config_.front_end_safeh);
}

void Se2SvsdfSolver::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                const FootprintModel& footprint) {
  (void)footprint;
  nh_.reset(new ros::NodeHandle(nh));
  config_ = BuildDefaultConfig();
  LoadConfigOverrides(pnh);
  ConfigureModules(true);
}

void Se2SvsdfSolver::InitializeForTesting(const FootprintModel& footprint) {
  (void)footprint;
  config_ = BuildDefaultConfig();
  ConfigureModules(false);
}

void Se2SvsdfSolver::ConfigureModules(bool enable_ros_io) {
  pcs_map_manager_.reset(new PCSmapManager(config_));
  swept_volume_manager_.reset(new SweptVolumeManager(config_));
  midend_.reset(new OriTraj);
  backend_.reset(new TrajOptimizer);

  if (enable_ros_io) {
    debug_publisher::init(*nh_);
    swept_volume_manager_->init(*nh_, config_);
    midend_->setParam(*nh_, config_);
    backend_->setParam(*nh_, config_);
  } else {
    swept_volume_manager_->flatness.reset(
        config_.vehicleMass, config_.gravAcc, config_.horizDrag,
        config_.vertDrag, config_.parasDrag, config_.speedEps);
    swept_volume_manager_->initShape(config_);

    midend_->conf = config_;
    midend_->weightPR = config_.weight_pr;
    midend_->weightAR = config_.weight_ar;
    midend_->rho = config_.rho_mid_end;
    midend_->vmax = config_.vmax;
    midend_->omgmax = config_.omgmax;
    midend_->weight_v = config_.weight_v;
    midend_->weight_omg = config_.weight_omg;
    midend_->weight_a = config_.weight_a;
    midend_->smooth_fac = config_.smoothingEps;
    midend_->integralRes = config_.integralIntervs;
    midend_->flatmap.reset(config_.vehicleMass, config_.gravAcc, config_.horizDrag,
                           config_.vertDrag, config_.parasDrag, config_.speedEps);

    backend_->conf = config_;
    backend_->vmax = config_.vmax;
    backend_->omgmax = config_.omgmax;
    backend_->thetamax = config_.thetamax;
    backend_->threads_num = config_.threads_num;
    backend_->rho = config_.rho;
    backend_->weight_a = config_.weight_a;
    backend_->weight_p = config_.weight_p;
    backend_->weight_v = config_.weight_v;
    backend_->weight_omg = config_.weight_omg;
    backend_->weight_theta = config_.weight_theta;
    backend_->smooth_fac = config_.smoothingEps;
    backend_->integralRes = config_.integralIntervs;
    backend_->safety_hor = config_.safety_hor;
    backend_->bdx = config_.kernel_size * config_.occupancy_resolution;
    backend_->bdy = backend_->bdx;
    backend_->bdz = backend_->bdx;
    backend_->flatmap.reset(config_.vehicleMass, config_.gravAcc, config_.horizDrag,
                            config_.vertDrag, config_.parasDrag, config_.speedEps);
  }

  backend_->setEnvironment(swept_volume_manager_);
  backend_->setGridMap(pcs_map_manager_);
  initialized_ = true;
}

bool Se2SvsdfSolver::UpdateMap(const nav_msgs::OccupancyGrid& map) {
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
  ready_ = true;
  return true;
}

void Se2SvsdfSolver::setGridMapForMidend(const GridMap* map) {
  if (midend_) {
    midend_->grid_map_ = map;
    midend_->weight_esdf_ = 80.0;
    midend_->esdf_margin_ = 0.30;
  }
}

bool Se2SvsdfSolver::BuildSupportData(
    const std::vector<SE2State>& segment, Eigen::Matrix3d* init_state,
    Eigen::Matrix3d* final_state, std::vector<Eigen::Vector3d>* points,
    std::vector<Eigen::Vector3d>* accelerations,
    std::vector<Eigen::Matrix3d>* rotations,
    Eigen::VectorXd* initial_times) const {
  if (segment.size() < 2 || init_state == nullptr || final_state == nullptr ||
      points == nullptr || accelerations == nullptr || rotations == nullptr ||
      initial_times == nullptr) {
    return false;
  }

  init_state->setZero();
  final_state->setZero();
  init_state->col(0) = ToStrictPoint(segment.front());
  final_state->col(0) = ToStrictPoint(segment.back());

  points->clear();
  accelerations->clear();
  rotations->clear();

  for (size_t i = 1; i + 1 < segment.size(); ++i) {
    points->push_back(ToStrictPoint(segment[i]));
    const Eigen::Matrix3d rotation = MakeRotation(segment[i].yaw);
    rotations->push_back(rotation);
    accelerations->push_back(rotation * Eigen::Vector3d::UnitZ());
  }

  const int piece_count = static_cast<int>(points->size()) + 1;
  initial_times->resize(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    Eigen::Vector3d p0 = (i == 0) ? init_state->col(0) : (*points)[i - 1];
    Eigen::Vector3d p1 = (i == piece_count - 1) ? final_state->col(0) : (*points)[i];
    double dist = (p1 - p0).head<2>().norm();
    
    double yaw0 = segment[i].yaw;
    double yaw1 = segment[i+1].yaw;
    double yaw_diff = std::abs(normalizeAngle(yaw1 - yaw0));
    
    // Allocate time based on linear distance (max ~1.0m/s) and angular distance (max ~0.5rad/s)
    double time_for_dist = dist / 1.0;
    double time_for_yaw = yaw_diff / 0.5;
    
    (*initial_times)(i) = std::max({config_.inittime, time_for_dist, time_for_yaw});
  }
  return true;
}

Trajectory Se2SvsdfSolver::ConvertTrajectory(
    const ::Trajectory<TRAJ_ORDER>& strict_traj) const {
  Trajectory output;
  const int piece_count = strict_traj.getPieceNum();
  const Eigen::VectorXd durations = strict_traj.getDurations();
  output.pos_pieces.resize(static_cast<size_t>(piece_count));
  output.yaw_pieces.resize(static_cast<size_t>(piece_count));

  for (int i = 0; i < piece_count; ++i) {
    const typename ::Trajectory<TRAJ_ORDER>::CoefficientMat& coeff_mat =
        strict_traj.getindexCoeffMat(i);
    PolyPiece piece;
    YawPolyPiece yaw_piece;
    piece.duration = durations(i);
    yaw_piece.duration = piece.duration;
    for (int j = 0; j < 6; ++j) {
      const int src_col = 5 - j;
      piece.coeffs(0, j) = coeff_mat(0, src_col);
      piece.coeffs(1, j) = coeff_mat(1, src_col);
      yaw_piece.coeffs(0, j) = coeff_mat(2, src_col);
    }
    if (i == 0) {
      Eigen::Vector3d minco_pos0 = strict_traj.getPos(0.0);
      ROS_INFO("ConvertTrajectory piece0: dur=%.4f minco_pos0=(%.4f,%.4f,%.4f) "
               "poly c0=(%.4f,%.4f) c1=(%.4f,%.4f) c5=(%.6f,%.6f)",
               piece.duration, minco_pos0(0), minco_pos0(1), minco_pos0(2),
               piece.coeffs(0,0), piece.coeffs(1,0),
               piece.coeffs(0,1), piece.coeffs(1,1),
               piece.coeffs(0,5), piece.coeffs(1,5));
    }
    output.pos_pieces[static_cast<size_t>(i)] = piece;
    output.yaw_pieces[static_cast<size_t>(i)] = yaw_piece;
  }

  return output;
}

bool Se2SvsdfSolver::Solve(const std::vector<SE2State>& segment,
                           Trajectory* trajectory, bool preserve_shape) {
  if (!ready_ || trajectory == nullptr) {
    return false;
  }

  Eigen::Matrix3d init_state = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> accelerations;
  std::vector<Eigen::Matrix3d> rotations;
  Eigen::VectorXd initial_times;
  if (!BuildSupportData(segment, &init_state, &final_state, &points,
                        &accelerations, &rotations, &initial_times)) {
    return false;
  }

  const int piece_count = static_cast<int>(points.size()) + 1;
  ::Trajectory<TRAJ_ORDER> strict_traj;
  Eigen::VectorXd opt_x;
  const bool mid_ok = midend_->getOriTraj(init_state, final_state, points,
                                          initial_times, accelerations,
                                          rotations, piece_count, strict_traj,
                                          opt_x);
  if (!mid_ok) {
    return false;
  }

  if (preserve_shape) {
    *trajectory = ConvertTrajectory(strict_traj);
    return !trajectory->empty();
  }

  swept_volume_manager_->updateTraj(strict_traj);
  backend_->pcsmap_manager->aabb_points.clear();
  Eigen::Vector3d last_point(999.0, 999.0, 999.0);
  for (size_t i = 0; i < points.size(); ++i) {
    backend_->pcsmap_manager->getPointsInAABBOutOfLastOne(
        points[i], last_point, config_.kernel_size * config_.occupancy_resolution / 3.0,
        config_.kernel_size * config_.occupancy_resolution / 3.0,
        config_.kernel_size * config_.occupancy_resolution / 3.0);
    last_point = points[i];
  }
  backend_->parallel_points.clear();
  for (std::unordered_map<int, Eigen::Vector3d>::const_iterator it =
           backend_->pcsmap_manager->aabb_points.begin();
       it != backend_->pcsmap_manager->aabb_points.end(); ++it) {
    backend_->parallel_points.push_back(it->second);
  }
  backend_->parallel_points_num =
      static_cast<int>(backend_->parallel_points.size());
  backend_->lastTstar.assign(backend_->parallel_points.size(), 0.0);

  const int ret = backend_->optimize_traj_lmbm(init_state, final_state, opt_x,
                                               piece_count, strict_traj);
  if (ret <= 0) {
    return false;
  }

  *trajectory = ConvertTrajectory(strict_traj);
  return !trajectory->empty();
}

}  // namespace esv_planner
