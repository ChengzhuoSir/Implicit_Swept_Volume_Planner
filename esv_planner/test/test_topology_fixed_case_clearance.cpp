#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/continuous_svsdf_evaluator.h>

#include <algorithm>
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

double topoPathClearance(const TopoPath& path,
                         const ContinuousSvsdfEvaluator& svsdf,
                         const SE2State& start,
                         const SE2State& goal) {
  if (path.waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  min_clearance = std::min(min_clearance, svsdf.evaluate(start));
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    min_clearance = std::min(min_clearance, svsdf.evaluate(
        SE2State(wp.pos.x(), wp.pos.y(), yaw)));
  }
  min_clearance = std::min(min_clearance, svsdf.evaluate(goal));
  return min_clearance;
}

double segmentClearance(const SE2State& a,
                        const SE2State& b,
                        const ContinuousSvsdfEvaluator& svsdf) {
  const double len = (b.position() - a.position()).norm();
  const double yaw_delta = std::abs(normalizeAngle(b.yaw - a.yaw));
  const int n_steps = std::max(
      2, std::max(static_cast<int>(std::ceil(len / 0.05)),
                  static_cast<int>(std::ceil(yaw_delta / 0.10))));
  double min_clearance = kInf;
  for (int i = 0; i <= n_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_steps);
    SE2State sample;
    sample.x = a.x + (b.x - a.x) * t;
    sample.y = a.y + (b.y - a.y) * t;
    sample.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
    min_clearance = std::min(min_clearance, svsdf.evaluate(sample));
  }
  return min_clearance;
}

void printFirstNegative(const TopoPath& path,
                        const ContinuousSvsdfEvaluator& svsdf,
                        TopologyPlanner& topology,
                        const SE2State& start,
                        const SE2State& goal) {
  std::vector<SE2State> states;
  std::vector<TopoWaypoint> topo_states;
  states.reserve(path.waypoints.size() + 2);
  states.push_back(start);
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    topo_states.push_back(wp);
    states.emplace_back(wp.pos.x(), wp.pos.y(), yaw);
  }
  states.push_back(goal);

  for (size_t wi = 0; wi < path.waypoints.size(); ++wi) {
    std::cerr << "[test] wp[" << wi << "]=(" << path.waypoints[wi].pos.x()
              << ", " << path.waypoints[wi].pos.y() << ") yaw="
              << (path.waypoints[wi].has_yaw ? path.waypoints[wi].yaw : 999.0)
              << " clearance="
              << svsdf.evaluate(SE2State(path.waypoints[wi].pos.x(),
                                         path.waypoints[wi].pos.y(),
                                         path.waypoints[wi].has_yaw ? path.waypoints[wi].yaw : start.yaw))
              << "\n";
  }

  for (size_t i = 0; i < states.size(); ++i) {
    const double clearance = svsdf.evaluate(states[i]);
    if (clearance < 0.0) {
      std::cerr << "[test] first_negative_idx=" << i
                << " pose=(" << states[i].x << ", " << states[i].y
                << ", " << states[i].yaw << ")"
                << " clearance=" << clearance << "\n";
      if (i > 0 && i + 1 < states.size() && i - 1 < topo_states.size()) {
        TopoWaypoint repaired = topo_states[i - 1];
        repaired.yaw = states[i].yaw;
        repaired.has_yaw = true;
        const bool ok = topology.repairWaypointBodyFrame(repaired, 0.1);
        const double repaired_clearance = svsdf.evaluate(
            SE2State(repaired.pos.x(), repaired.pos.y(), repaired.yaw));
        std::cerr << "[test] single_wp_repair_ok=" << (ok ? 1 : 0)
                  << " repaired_pose=(" << repaired.pos.x() << ", " << repaired.pos.y()
                  << ", " << repaired.yaw << ")"
                  << " repaired_clearance=" << repaired_clearance << "\n";
        const double skip_clearance = segmentClearance(states[i - 1], states[i + 1], svsdf);
        std::cerr << "[test] skip_prev_next_clearance=" << skip_clearance << "\n";
        if (i + 2 < states.size()) {
          std::cerr << "[test] skip_prev_next2_clearance="
                    << segmentClearance(states[i - 1], states[i + 2], svsdf) << "\n";
        }
        if (i + 3 < states.size()) {
          std::cerr << "[test] skip_prev_next3_clearance="
                    << segmentClearance(states[i - 1], states[i + 3], svsdf) << "\n";
        }
      }
      return;
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_topology_fixed_case_clearance",
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
  footprint.setPolygon({{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
                        {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topology;
  topology.init(map, checker, 800, 20, 1, footprint.inscribedRadius());

  ContinuousSvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);

  topology.buildRoadmap(start.position(), goal.position());
  auto topo_paths = topology.searchPaths();
  topology.shortenPaths(topo_paths);

  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one topology path\n";
    return 1;
  }

  size_t best_idx = 0;
  double best_clearance = -kInf;
  for (size_t i = 0; i < topo_paths.size(); ++i) {
    const double clearance = topoPathClearance(topo_paths[i], svsdf, start, goal);
    std::cout << "[test] path=" << i
              << " waypoints=" << topo_paths[i].waypoints.size()
              << " clearance=" << clearance << "\n";
    if (clearance > best_clearance) {
      best_clearance = clearance;
      best_idx = i;
    }
  }

  std::cout << "[test] best_idx=" << best_idx
            << " best_clearance=" << best_clearance << "\n";
  if (best_clearance < 0.0) {
    printFirstNegative(topo_paths[best_idx], svsdf, topology, start, goal);
    std::cerr << "[test] FAIL: expected best shortened topology path to be nonnegative under continuous evaluator\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
