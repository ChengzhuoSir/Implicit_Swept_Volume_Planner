#include <iostream>
#include <vector>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#include "esv_planner/common.h"
#include "esv_planner/grid_map.h"
#include "esv_planner/footprint_model.h"
#include "esv_planner/collision_checker.h"
#include "esv_planner/hybrid_astar.h"

using namespace esv_planner;

static nav_msgs::OccupancyGrid::Ptr makeBorderMap() {
  nav_msgs::OccupancyGrid::Ptr grid(new nav_msgs::OccupancyGrid());
  grid->info.resolution = 0.05;
  grid->info.width = 200;
  grid->info.height = 200;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->header.frame_id = "map";
  grid->data.assign(grid->info.width * grid->info.height, 0);

  const int w = static_cast<int>(grid->info.width);
  const int h = static_cast<int>(grid->info.height);
  for (int x = 0; x < w; ++x) {
    grid->data[x] = 100;
    grid->data[(h - 1) * w + x] = 100;
  }
  for (int y = 0; y < h; ++y) {
    grid->data[y * w] = 100;
    grid->data[y * w + (w - 1)] = 100;
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
  GridMap map;
  auto msg = makeBorderMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(msg));

  FootprintModel footprint = makeFootprint();
  map.inflateByRadius(footprint.inscribedRadius());

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  HybridAStarParams params;
  HybridAStarPlanner planner;
  planner.init(map, checker, params);

  // Start is too close to boundary: all yaws are colliding at this exact position.
  // A robust planner should recover to a nearby feasible pose and still find a path.
  SE2State start(0.30, 5.0, 0.0);
  SE2State goal(4.00, 5.0, 0.0);

  std::vector<SE2State> path;
  bool ok = planner.plan(start, goal, path);

  if (!ok || path.size() < 2) {
    std::cerr << "[FAIL] Expected Hybrid A* to recover and find a path, but got failure.\n";
    return 1;
  }

  std::cout << "[PASS] Hybrid A* recovered and produced path of size " << path.size() << "\n";
  return 0;
}
