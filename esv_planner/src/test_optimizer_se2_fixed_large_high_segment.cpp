#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>
#include <esv_planner/continuous_svsdf_evaluator.h>
#include <esv_planner/trajectory_optimizer.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <chrono>
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

double segmentLength(const std::vector<SE2State>& wps) {
  if (wps.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < wps.size(); ++i) {
    len += (wps[i].position() - wps[i - 1].position()).norm();
  }
  return len;
}

double continuousChainClearance(const std::vector<SE2State>& waypoints,
                                const ContinuousSvsdfEvaluator& svsdf,
                                double sample_step) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double len = (b.position() - a.position()).norm();
    const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
    const int linear_steps = std::max(
        1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
    const int yaw_steps =
        std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.1)));
    const int n_steps = std::max(linear_steps, yaw_steps);
    for (int s = 0; s <= n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      SE2State st;
      st.x = a.x + (b.x - a.x) * t;
      st.y = a.y + (b.y - a.y) * t;
      st.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
      min_clearance = std::min(min_clearance, svsdf.evaluate(st));
    }
  }
  return min_clearance;
}

}  // namespace

int main(int argc, char** argv) {
  using Clock = std::chrono::steady_clock;
  ros::init(argc, argv, "test_optimizer_se2_fixed_large_high_segment",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path =
      "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";
  int width = 0, height = 0, maxval = 0;
  std::vector<uint8_t> pixels;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    std::cerr << "[test] FAIL: cannot load maze map\n";
    return 1;
  }

  auto occ = pgmToOccupancyGrid(pixels, width, height, maxval, 0.05);
  GridMap map;
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  FootprintModel footprint;
  footprint.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topo;
  topo.init(map, checker, 800, 20, 14, footprint.inscribedRadius());
  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);
  topo.buildRoadmap(start.position(), goal.position());
  auto paths = topo.searchPaths();
  topo.shortenPaths(paths);
  if (paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one fixed-case topo path\n";
    return 1;
  }

  SE2SequenceGenerator se2gen;
  se2gen.init(map, checker, 0.15, 5);
  ContinuousSvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  OptimizerParams params;
  params.max_iterations = 100;
  params.lambda_safety = 10.0;
  params.safety_margin = 0.1;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  auto segments = se2gen.generate(paths.front(), start, goal);
  if (segments.empty()) {
    std::cerr << "[test] FAIL: fixed-case path produced no segments\n";
    return 1;
  }

  size_t target_seg_idx = segments.size();
  size_t target_seg_size = 0;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto& seg = segments[i];
    if (seg.risk != RiskLevel::HIGH || seg.waypoints.size() <= 6) continue;
    if (seg.waypoints.size() > target_seg_size) {
      target_seg_idx = i;
      target_seg_size = seg.waypoints.size();
    }
  }

  if (target_seg_idx == segments.size()) {
    std::cout << "[test] PASS: fixed case no longer contains large HIGH segments\n";
    return 0;
  }

  const auto& seg = segments[target_seg_idx];
  const double chain_clearance = continuousChainClearance(seg.waypoints, svsdf, 0.05);
  const double seg_time = std::max(0.5, segmentLength(seg.waypoints) /
                                          std::max(0.10, params.max_vel));
  std::cout << "[test] target_high_seg=" << target_seg_idx
            << " wps=" << seg.waypoints.size()
            << " chain_clearance=" << chain_clearance
            << " seg_time=" << seg_time << std::endl;
  const auto t_begin = Clock::now();
  const OptimizerResult result = optimizer.optimizeSE2Detailed(seg.waypoints, seg_time);
  const auto t_end = Clock::now();
  const double elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_begin).count();

  double clearance = -kInf;
  if (!result.traj.empty()) {
    clearance = svsdf.evaluateTrajectory(result.traj);
  }

  std::cout << "[test] se2_success=" << (result.success ? 1 : 0)
            << " se2_clearance=" << clearance
            << " elapsed_ms=" << elapsed_ms
            << " source_mode=" << static_cast<int>(result.source_info.source_mode)
            << "\n";

  if (!result.success || result.traj.empty()) {
    std::cerr << "[test] FAIL: expected large HIGH segment to produce a trajectory\n";
    return 1;
  }
  if (clearance < params.safety_margin - 1e-6) {
    std::cerr << "[test] FAIL: expected large HIGH segment to reach target clearance\n";
    return 1;
  }
  if (elapsed_ms > 15000.0) {
    std::cerr << "[test] FAIL: expected bounded runtime for large HIGH segment\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
