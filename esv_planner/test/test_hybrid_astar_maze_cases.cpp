#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/collision_checker.h"
#include "esv_planner/hybrid_astar.h"

using namespace esv_planner;

static bool loadPgm(const std::string& path,
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
        std::string dummy;
        std::getline(ifs, dummy);
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

static nav_msgs::OccupancyGrid::Ptr
pgmToOccupancyGrid(const std::vector<uint8_t>& pixels,
                   int width, int height, int maxval,
                   double resolution) {
  nav_msgs::OccupancyGrid::Ptr grid(new nav_msgs::OccupancyGrid());
  grid->info.resolution = static_cast<float>(resolution);
  grid->info.width = static_cast<uint32_t>(width);
  grid->info.height = static_cast<uint32_t>(height);
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->info.origin.position.z = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->header.frame_id = "map";
  grid->data.resize(static_cast<size_t>(width) * height);

  for (int row = 0; row < height; ++row) {
    int pgm_row = height - 1 - row;
    for (int col = 0; col < width; ++col) {
      uint8_t pix = pixels[static_cast<size_t>(pgm_row) * width + col];
      int8_t occ;
      if (pix >= 254) {
        occ = 0;
      } else if (pix == 0) {
        occ = 100;
      } else {
        occ = static_cast<int8_t>(100 - static_cast<int>(pix) * 100 / maxval);
      }
      grid->data[static_cast<size_t>(row) * width + col] = occ;
    }
  }
  return grid;
}

static FootprintModel makeFootprint() {
  FootprintModel footprint;
  std::vector<Eigen::Vector2d> verts;
  verts.emplace_back(0.25, 0.3);
  verts.emplace_back(0.25, -0.3);
  verts.emplace_back(-0.25, -0.3);
  verts.emplace_back(-0.25, -0.1);
  verts.emplace_back(-0.6, -0.1);
  verts.emplace_back(-0.6, 0.1);
  verts.emplace_back(-0.25, 0.1);
  verts.emplace_back(-0.25, 0.3);
  footprint.setPolygon(verts);
  footprint.setInscribedRadius(0.1);
  return footprint;
}

int main() {
  int w = 0, h = 0, maxv = 0;
  std::vector<uint8_t> px;
  const std::string map_path = "src/esv_planner/maps/maze.pgm";
  if (!loadPgm(map_path, w, h, maxv, px)) {
    std::cerr << "[FAIL] Cannot load map: " << map_path << "\n";
    return 1;
  }

  auto occ = pgmToOccupancyGrid(px, w, h, maxv, 0.05);

  GridMap grid;
  grid.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));

  FootprintModel footprint = makeFootprint();
  grid.inflateByRadius(footprint.inscribedRadius());

  CollisionChecker checker;
  checker.init(grid, footprint, 18);

  HybridAStarParams params;
  HybridAStarPlanner planner;
  planner.init(grid, checker, params);

  std::vector<std::pair<SE2State, SE2State>> cases = {
      {SE2State(0.50, 4.11, -1.55), SE2State(9.28, 5.61, 1.59)},
      {SE2State(0.50, 4.11, -1.55), SE2State(8.00, 8.00, 1.57)},
      {SE2State(1.00, 1.00, 0.00), SE2State(8.00, 8.00, 1.57)},
  };

  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& s = cases[i].first;
    const auto& g = cases[i].second;

    auto s_safe = checker.safeYawIndices(s.x, s.y);
    auto g_safe = checker.safeYawIndices(g.x, g.y);

    std::vector<SE2State> path;
    bool ok = planner.plan(s, g, path);

    std::cout << "case[" << i << "] start=(" << s.x << "," << s.y << "," << s.yaw
              << ") goal=(" << g.x << "," << g.y << "," << g.yaw << ")\n";
    std::cout << "  safe_yaw_count: start=" << s_safe.size()
              << " goal=" << g_safe.size() << "\n";
    std::cout << "  hybrid_ok=" << (ok ? "true" : "false")
              << " path_size=" << path.size() << "\n";
  }

  return 0;
}
