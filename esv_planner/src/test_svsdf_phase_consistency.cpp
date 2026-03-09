#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>
#include <esv_planner/svsdf_evaluator.h>
#include <esv_planner/trajectory_optimizer.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace esv_planner;

namespace {

bool loadPgm(const std::string& path,
             int& width, int& height, int& maxval,
             std::vector<uint8_t>& pixels) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) return false;

  std::string magic;
  ifs >> magic;
  if (magic != "P5") return false;

  auto skipComments = [&]() {
    while (ifs.peek() == '#' || ifs.peek() == '\n' ||
           ifs.peek() == '\r' || ifs.peek() == ' ') {
      if (ifs.peek() == '#') {
        std::string line;
        std::getline(ifs, line);
      } else {
        ifs.get();
      }
    }
  };

  skipComments();
  ifs >> width;
  skipComments();
  ifs >> height;
  skipComments();
  ifs >> maxval;
  ifs.get();

  if (width <= 0 || height <= 0 || maxval <= 0) return false;
  pixels.resize(static_cast<size_t>(width) * height);
  ifs.read(reinterpret_cast<char*>(pixels.data()),
           static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(ifs);
}

nav_msgs::OccupancyGrid::Ptr pgmToOccupancyGrid(const std::vector<uint8_t>& pixels,
                                                int width, int height,
                                                int maxval, double resolution) {
  auto grid = boost::make_shared<nav_msgs::OccupancyGrid>();
  grid->info.resolution = static_cast<float>(resolution);
  grid->info.width = static_cast<uint32_t>(width);
  grid->info.height = static_cast<uint32_t>(height);
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->header.frame_id = "map";
  grid->data.resize(static_cast<size_t>(width) * height);

  for (int row = 0; row < height; ++row) {
    const int pgm_row = height - 1 - row;
    for (int col = 0; col < width; ++col) {
      const uint8_t pix = pixels[static_cast<size_t>(pgm_row) * width + col];
      int8_t occ = 0;
      if (pix >= 254) occ = 0;
      else if (pix == 0) occ = 100;
      else occ = static_cast<int8_t>(100 - static_cast<int>(pix) * 100 / maxval);
      grid->data[static_cast<size_t>(row) * width + col] = occ;
    }
  }

  return grid;
}

double segmentLength(const std::vector<SE2State>& waypoints) {
  if (waypoints.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    len += (waypoints[i].position() - waypoints[i - 1].position()).norm();
  }
  return len;
}

OptimizerParams defaultParams() {
  OptimizerParams params;
  params.max_iterations = 100;
  params.lambda_smooth = 1.0;
  params.lambda_time = 1.0;
  params.lambda_safety = 10.0;
  params.lambda_dynamics = 1.0;
  params.lambda_pos_residual = 5.0;
  params.lambda_yaw_residual = 2.0;
  params.max_vel = 1.0;
  params.max_acc = 2.0;
  params.max_yaw_rate = 1.5;
  params.safety_margin = 0.1;
  params.step_size = 0.005;
  return params;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_svsdf_phase_consistency",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path =
      "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";
  int width = 0, height = 0, maxval = 0;
  std::vector<uint8_t> pixels;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    std::cerr << "[test] FAIL: could not load maze map\n";
    return 1;
  }

  auto occ = pgmToOccupancyGrid(pixels, width, height, maxval, 0.05);
  GridMap map;
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  FootprintModel footprint;
  footprint.setPolygon({{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
                        {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topology;
  topology.init(map, checker, 800, 20, 14, footprint.inscribedRadius());

  SE2SequenceGenerator se2gen;
  se2gen.init(map, checker, 0.15, 5);

  SvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, defaultParams());

  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);

  topology.buildRoadmap(start.position(), goal.position());
  auto topo_paths = topology.searchPaths();
  topology.shortenPaths(topo_paths);

  bool found_inconsistency = false;
  for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
    const auto segments = se2gen.generate(topo_paths[pi], start, goal);
    if (segments.empty()) continue;

    std::vector<Trajectory> seg_trajs(segments.size());
    std::vector<double> seg_times(segments.size(), 0.5);
    bool all_built = true;
    for (size_t si = 0; si < segments.size(); ++si) {
      seg_times[si] =
          std::max(0.5, segmentLength(segments[si].waypoints) / 1.0);
      if (segments[si].risk == RiskLevel::HIGH) {
        seg_trajs[si] = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
      } else {
        seg_trajs[si] = optimizer.optimizeR2(segments[si].waypoints, seg_times[si]);
      }
      if (seg_trajs[si].empty()) {
        all_built = false;
        break;
      }
    }
    if (!all_built) continue;

    const Trajectory stitched = optimizer.stitch(segments, seg_trajs);
    if (stitched.empty()) continue;
    Trajectory exact_concat;
    for (const auto& seg_traj : seg_trajs) {
      exact_concat.pos_pieces.insert(exact_concat.pos_pieces.end(),
                                     seg_traj.pos_pieces.begin(),
                                     seg_traj.pos_pieces.end());
      exact_concat.yaw_pieces.insert(exact_concat.yaw_pieces.end(),
                                     seg_traj.yaw_pieces.begin(),
                                     seg_traj.yaw_pieces.end());
    }
    const double exact_concat_clearance =
        svsdf.evaluateTrajectory(exact_concat, 0.05);
    const double stitched_clearance = svsdf.evaluateTrajectory(stitched, 0.05);
    if (stitched_clearance < 0.0) continue;

    for (size_t si = 0; si < seg_trajs.size(); ++si) {
      const double seg_clearance = svsdf.evaluateTrajectory(seg_trajs[si], 0.05);
      if (seg_clearance < 0.0) {
        std::cout << "[test] inconsistency path=" << pi
                  << " seg=" << si
                  << " risk=" << (segments[si].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
                  << " seg_clearance=" << seg_clearance
                  << " exact_concat_clearance=" << exact_concat_clearance
                  << " stitched_clearance=" << stitched_clearance << "\n";
        found_inconsistency = true;
        break;
      }
    }
    if (found_inconsistency) break;
  }

  if (found_inconsistency) {
    std::cerr << "[test] FAIL: SVSDF trajectory evaluation depends on time-offset/concatenation phase\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
