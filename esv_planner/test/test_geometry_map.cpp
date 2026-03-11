#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/geometry_map.h>
#include <esv_planner/grid_map.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
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

bool approxVec(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double tol) {
  return (a - b).norm() <= tol;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_geometry_map", ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path = "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";

  int width = 0, height = 0, maxval = 0;
  std::vector<uint8_t> pixels;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    return 1;
  }

  auto occ = pgmToOccupancyGrid(pixels, width, height, maxval, 0.05);
  GridMap map;
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  const GeometryMap& geometry = map.geometryMap();
  std::cout << "[test] boundary_points=" << geometry.boundaryPoints().size()
            << " boundary_segments=" << geometry.segments().size() << "\n";

  if (geometry.boundaryPoints().empty() || geometry.segments().empty()) {
    std::cerr << "[test] FAIL: expected geometry map to extract non-empty obstacle boundaries\n";
    return 1;
  }

  const Eigen::Vector2d start(1.0, 8.5);
  double dist_a = -1.0;
  double dist_b = -1.0;
  const Eigen::Vector2d nearest_a = geometry.nearestObstacle(start, &dist_a);
  const Eigen::Vector2d nearest_b = geometry.nearestObstacle(start, &dist_b);
  std::cout << "[test] nearest=(" << nearest_a.x() << "," << nearest_a.y()
            << ") dist_a=" << dist_a
            << " dist_b=" << dist_b << "\n";

  if (!(dist_a >= 0.0) || !(dist_b >= 0.0) ||
      !approxVec(nearest_a, nearest_b, 1e-9) ||
      std::abs(dist_a - dist_b) > 1e-9) {
    std::cerr << "[test] FAIL: expected stable nearest-obstacle query results\n";
    return 1;
  }

  const auto local = geometry.queryLocalSegments(nearest_a, 0.6, 24);
  std::cout << "[test] local_segments=" << local.size() << "\n";
  if (local.empty()) {
    std::cerr << "[test] FAIL: expected a non-empty local obstacle neighborhood near the nearest obstacle\n";
    return 1;
  }
  if (local.size() > 24) {
    std::cerr << "[test] FAIL: expected local obstacle neighborhood query to stay bounded\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
