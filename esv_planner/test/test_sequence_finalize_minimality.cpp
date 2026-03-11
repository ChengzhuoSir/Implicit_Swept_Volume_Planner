#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>

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

size_t countHighSegments(const std::vector<MotionSegment>& segments) {
  size_t count = 0;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) ++count;
  }
  return count;
}

size_t totalHighWaypoints(const std::vector<MotionSegment>& segments) {
  size_t total = 0;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) total += seg.waypoints.size();
  }
  return total;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_sequence_finalize_minimality",
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
  footprint.setPolygon({{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
                        {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topology;
  topology.init(map, checker, 800, 20, 14, footprint.inscribedRadius());

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.15, 5);

  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);

  topology.buildRoadmap(start.position(), goal.position());
  auto topo_paths = topology.searchPaths();
  topology.shortenPaths(topo_paths);
  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one topology path\n";
    return 1;
  }

  const auto core = generator.generateCore(topo_paths.front(), start, goal);
  const auto final = generator.generate(topo_paths.front(), start, goal);

  const size_t core_high = countHighSegments(core);
  const size_t final_high = countHighSegments(final);
  const size_t core_high_wp = totalHighWaypoints(core);
  const size_t final_high_wp = totalHighWaypoints(final);

  std::cout << "[test] core_high=" << core_high
            << " final_high=" << final_high
            << " core_high_wp=" << core_high_wp
            << " final_high_wp=" << final_high_wp << "\n";

  if (final_high > core_high) {
    std::cerr << "[test] FAIL: finalize should not create more HIGH segments than the core recursion\n";
    return 1;
  }
  if (final_high_wp > core_high_wp) {
    std::cerr << "[test] FAIL: finalize should not expand total HIGH waypoint context\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
