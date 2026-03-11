#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>

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
  if (!ifs.is_open()) {
    std::cerr << "[test] cannot open map: " << path << "\n";
    return false;
  }

  std::string magic;
  ifs >> magic;
  if (magic != "P5") {
    std::cerr << "[test] map is not P5 pgm: " << path << "\n";
    return false;
  }

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

FootprintModel makeTFootprint() {
  FootprintModel footprint;
  footprint.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);
  return footprint;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_se2_fixed_case_fragmentation",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path =
      "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";
  int width = 0;
  int height = 0;
  int maxval = 0;
  std::vector<uint8_t> pixels;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    return 1;
  }

  GridMap map;
  auto occ = pgmToOccupancyGrid(pixels, width, height, maxval, 0.05);
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topology;
  topology.init(map, checker, 800, 20, 14, footprint.inscribedRadius());

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.10, 5);

  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);

  topology.buildRoadmap(start.position(), goal.position());
  auto topo_paths = topology.searchPaths();
  topology.shortenPaths(topo_paths);
  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one topo path\n";
    return 1;
  }

  auto segments = generator.generate(topo_paths.front(), start, goal);
  if (segments.empty()) {
    std::cerr << "[test] FAIL: expected motion segments\n";
    return 1;
  }

  int high_count = 0;
  int small_high_count = 0;
  int max_high_size = 0;
  int max_consecutive_high = 0;
  int consecutive_high = 0;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      ++high_count;
      ++consecutive_high;
      if (static_cast<int>(seg.waypoints.size()) <= 4) {
        ++small_high_count;
      }
      max_high_size = std::max(max_high_size, static_cast<int>(seg.waypoints.size()));
      max_consecutive_high = std::max(max_consecutive_high, consecutive_high);
    } else {
      consecutive_high = 0;
    }
  }

  std::cout << "[test] segments=" << segments.size()
            << " high_count=" << high_count
            << " small_high_count=" << small_high_count
            << " max_high_size=" << max_high_size
            << " max_consecutive_high=" << max_consecutive_high << "\n";

  if (high_count > 12) {
    std::cerr << "[test] FAIL: expected fixed case to avoid pathological HIGH fragmentation\n";
    return 1;
  }
  if (small_high_count > 1) {
    std::cerr << "[test] FAIL: expected generator to avoid many tiny HIGH windows\n";
    return 1;
  }
  if (max_consecutive_high > 3) {
    std::cerr << "[test] FAIL: expected local repairs to keep HIGH chains bounded\n";
    return 1;
  }
  if (max_high_size > 12) {
    std::cerr << "[test] FAIL: expected HIGH windows to stay bounded\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
