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

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
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

double segmentChainClearance(const std::vector<SE2State>& waypoints,
                             const ContinuousSvsdfEvaluator& svsdf,
                             double sample_step) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto& a = waypoints[i];
    const auto& b = waypoints[i + 1];
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

double topoPathClearance(const TopoPath& path,
                         const ContinuousSvsdfEvaluator& svsdf,
                         const SE2State& start,
                         const SE2State& goal) {
  if (path.waypoints.empty()) return -kInf;
  double min_clearance = std::min(svsdf.evaluate(start), svsdf.evaluate(goal));
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    min_clearance = std::min(
        min_clearance, svsdf.evaluate(SE2State(wp.pos.x(), wp.pos.y(), yaw)));
  }
  return min_clearance;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_optimizer_se2_maze_l_path1_high_segment",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path =
      "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";

  int width = 0;
  int height = 0;
  int maxval = 0;
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
      {0.30, 0.30}, {0.30, 0.10}, {-0.10, 0.10},
      {-0.10, -0.30}, {-0.30, -0.30}, {-0.30, 0.30}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  const SE2State start(1.0, 8.5, 0.0);
  const SE2State goal(8.5, 1.0, -1.57);

  TopologyPlanner topo;
  topo.init(map, checker, 800, 20, 14, footprint.inscribedRadius());
  topo.buildRoadmap(start.position(), goal.position());
  auto paths = topo.searchPaths();
  topo.shortenPaths(paths);
  if (paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one maze path\n";
    return 1;
  }

  SE2SequenceGenerator se2gen;
  se2gen.init(map, checker, 0.15, 5);
  ContinuousSvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  std::vector<size_t> topo_order(paths.size());
  for (size_t i = 0; i < paths.size(); ++i) {
    topo_order[i] = i;
  }
  std::sort(topo_order.begin(), topo_order.end(),
            [&](size_t lhs, size_t rhs) {
              const double lhs_clearance =
                  topoPathClearance(paths[lhs], svsdf, start, goal);
              const double rhs_clearance =
                  topoPathClearance(paths[rhs], svsdf, start, goal);
              if (std::abs(lhs_clearance - rhs_clearance) > 1e-6) {
                return lhs_clearance > rhs_clearance;
              }
              return paths[lhs].length < paths[rhs].length;
            });

  size_t target_path_idx = paths.size();
  std::vector<MotionSegment> target_segments;
  for (const size_t path_idx : topo_order) {
    auto segments = se2gen.generate(paths[path_idx], start, goal);
    if (segments.empty()) continue;
    int high_count = 0;
    for (const auto& seg : segments) {
      if (seg.risk == RiskLevel::HIGH) {
        ++high_count;
      }
    }
    if (high_count == 2) {
      target_path_idx = path_idx;
      target_segments = std::move(segments);
      break;
    }
  }

  if (target_path_idx == paths.size()) {
    std::cerr << "[test] FAIL: did not find the maze/L two-HIGH topology path\n";
    return 1;
  }

  OptimizerParams params;
  params.max_iterations = 100;
  params.lambda_safety = 10.0;
  params.safety_margin = 0.1;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  int high_seen = 0;
  for (size_t si = 0; si < target_segments.size(); ++si) {
    const auto& seg = target_segments[si];
    if (seg.risk != RiskLevel::HIGH) continue;
    ++high_seen;
    const double chain_clearance =
        segmentChainClearance(seg.waypoints, svsdf, 0.025);
    const double seg_time =
        std::max(0.5, segmentLength(seg.waypoints) / std::max(0.10, params.max_vel));
    const OptimizerResult result =
        optimizer.optimizeSE2Detailed(seg.waypoints, seg_time);
    const double clearance =
        result.traj.empty() ? -kInf : svsdf.evaluateTrajectory(result.traj);

    std::cout << "[test] path=" << target_path_idx
              << " high_seg=" << si
              << " high_rank=" << high_seen
              << " wps=" << seg.waypoints.size()
              << " chain_clearance=" << chain_clearance
              << " se2_success=" << (result.success ? 1 : 0)
              << " traj_empty=" << (result.traj.empty() ? 1 : 0)
              << " se2_clearance=" << clearance << "\n";

    if (!result.success || result.traj.empty()) {
      std::cerr << "[test] FAIL: maze/L two-HIGH path produced empty SE2 trajectory\n";
      return 1;
    }
    if (clearance + 1e-6 < params.safety_margin) {
      std::cerr << "[test] FAIL: maze/L two-HIGH path stayed below safety_margin\n";
      return 1;
    }
  }

  if (high_seen != 2) {
    std::cerr << "[test] FAIL: expected exactly two HIGH segments on the target path\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
