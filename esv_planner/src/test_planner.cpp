/**
 * test_planner.cpp — Offline integration test for the ESV planner pipeline.
 *
 * Loads a P5 binary PGM map, converts to nav_msgs::OccupancyGrid,
 * runs the full 3-stage planning pipeline, prints diagnostics.
 * Exit 0 = PASS, 1 = FAIL.
 *
 * Usage:  rosrun esv_planner test_planner [path/to/map.pgm]
 */

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

#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace esv_planner;

// ---------------------------------------------------------------------------
// P5 binary PGM loader
// ---------------------------------------------------------------------------
static bool loadPgm(const std::string& path,
                    int& width, int& height, int& maxval,
                    std::vector<uint8_t>& pixels)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    std::cerr << "[test] Cannot open PGM file: " << path << "\n";
    return false;
  }

  // Read magic
  std::string magic;
  ifs >> magic;
  if (magic != "P5") {
    std::cerr << "[test] Not a P5 binary PGM (got \"" << magic << "\")\n";
    return false;
  }

  // Skip comments and whitespace between header tokens
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

  // Consume the single whitespace byte after maxval before binary data
  ifs.get();

  if (width <= 0 || height <= 0 || maxval <= 0) {
    std::cerr << "[test] Invalid PGM header: " << width << "x" << height
              << " maxval=" << maxval << "\n";
    return false;
  }

  pixels.resize(static_cast<size_t>(width) * height);
  ifs.read(reinterpret_cast<char*>(pixels.data()),
           static_cast<std::streamsize>(pixels.size()));

  if (!ifs) {
    std::cerr << "[test] Failed to read pixel data (expected "
              << pixels.size() << " bytes)\n";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Convert PGM pixels -> nav_msgs::OccupancyGrid
//   pixel 254 (white/free)  -> occupancy 0
//   pixel 0   (black/wall)  -> occupancy 100
//   in-between              -> linear mapping
// ---------------------------------------------------------------------------
static nav_msgs::OccupancyGrid::Ptr
pgmToOccupancyGrid(const std::vector<uint8_t>& pixels,
                   int width, int height, int maxval,
                   double resolution)
{
  auto grid = boost::make_shared<nav_msgs::OccupancyGrid>();
  grid->info.resolution = static_cast<float>(resolution);
  grid->info.width  = static_cast<uint32_t>(width);
  grid->info.height = static_cast<uint32_t>(height);
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->info.origin.position.z = 0.0;
  grid->info.origin.orientation.w = 1.0;
  grid->header.frame_id = "map";
  grid->header.stamp = ros::Time::now();
  grid->data.resize(static_cast<size_t>(width) * height);

  for (int row = 0; row < height; ++row) {
    // PGM row 0 = top, OccupancyGrid row 0 = bottom -> flip vertically
    int pgm_row = height - 1 - row;
    for (int col = 0; col < width; ++col) {
      uint8_t pix = pixels[static_cast<size_t>(pgm_row) * width + col];
      int8_t occ;
      if (pix >= 254) {
        occ = 0;    // free
      } else if (pix == 0) {
        occ = 100;  // occupied
      } else {
        occ = static_cast<int8_t>(
            100 - static_cast<int>(pix) * 100 / maxval);
      }
      grid->data[static_cast<size_t>(row) * width + col] = occ;
    }
  }
  return grid;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_planner", ros::init_options::NoSigintHandler);
  ros::Time::init();

  // --- Map path from argv[1] or default ------------------------------------
  std::string map_path;
  if (argc >= 2) {
    map_path = argv[1];
  } else {
    map_path = "/home/chengzhuo/workspace/plan/src/esv_planner/maps/office.pgm";
  }

  std::cout << "========================================\n";
  std::cout << "  ESV Planner — Offline Test Node\n";
  std::cout << "========================================\n";
  std::cout << "[test] Map file: " << map_path << "\n";

  auto t_total_start = std::chrono::steady_clock::now();

  // =========================================================================
  // 1. Load PGM and convert to OccupancyGrid
  // =========================================================================
  int pgm_w = 0, pgm_h = 0, pgm_maxval = 0;
  std::vector<uint8_t> pgm_pixels;
  if (!loadPgm(map_path, pgm_w, pgm_h, pgm_maxval, pgm_pixels)) {
    std::cerr << "[test] FAIL: could not load PGM map.\n";
    return 1;
  }

  const double resolution = 0.05;
  nav_msgs::OccupancyGrid::Ptr occ_grid =
      pgmToOccupancyGrid(pgm_pixels, pgm_w, pgm_h, pgm_maxval, resolution);

  std::cout << "[test] PGM loaded: " << pgm_w << " x " << pgm_h
            << "  maxval=" << pgm_maxval << "\n";

  // =========================================================================
  // 2. Set up components
  // =========================================================================

  // GridMap
  GridMap grid_map;
  grid_map.fromOccupancyGrid(
      boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ_grid));
  grid_map.computeEsdf();

  if (!grid_map.ready()) {
    std::cerr << "[test] FAIL: GridMap not ready after init.\n";
    return 1;
  }

  std::cout << "[test] GridMap: " << grid_map.width() << " x "
            << grid_map.height() << "  resolution=" << grid_map.resolution()
            << "\n";

  // ESDF range
  const auto& esdf = grid_map.esdf();
  double esdf_min = *std::min_element(esdf.begin(), esdf.end());
  double esdf_max = *std::max_element(esdf.begin(), esdf.end());
  std::cout << "[test] ESDF range: [" << esdf_min << ", " << esdf_max << "]\n";

  // FootprintModel — T-shape with specified vertices
  FootprintModel footprint;
  {
    std::vector<Eigen::Vector2d> verts;
    verts.push_back(Eigen::Vector2d( 0.25,  0.3));
    verts.push_back(Eigen::Vector2d( 0.25, -0.3));
    verts.push_back(Eigen::Vector2d(-0.25, -0.3));
    verts.push_back(Eigen::Vector2d(-0.25, -0.1));
    verts.push_back(Eigen::Vector2d(-0.60, -0.1));
    verts.push_back(Eigen::Vector2d(-0.60,  0.1));
    verts.push_back(Eigen::Vector2d(-0.25,  0.1));
    verts.push_back(Eigen::Vector2d(-0.25,  0.3));
    footprint.setPolygon(verts);
  }
  std::cout << "[test] Footprint: T-shape, inscribed_r="
            << footprint.inscribedRadius()
            << "  circumscribed_r=" << footprint.circumscribedRadius() << "\n";

  // CollisionChecker (18 yaw bins)
  CollisionChecker checker;
  checker.init(grid_map, footprint, 18);
  checker.generateRobotKernels();

  // TopologyPlanner
  TopologyPlanner topo;
  topo.init(grid_map, checker,
            /*num_samples=*/520, /*knn=*/18, /*max_paths=*/14,
            footprint.inscribedRadius());

  // SE2SequenceGenerator
  SE2SequenceGenerator se2gen;
  se2gen.init(grid_map, checker, /*disc_step=*/0.15, /*max_push=*/5);

  // SvsdfEvaluator
  SvsdfEvaluator svsdf;
  svsdf.init(grid_map, footprint);

  // TrajectoryOptimizer with default OptimizerParams
  OptimizerParams opt_params;
  TrajectoryOptimizer optimizer;
  optimizer.init(grid_map, svsdf, opt_params);

  // =========================================================================
  // 3. Run 3-stage pipeline
  // =========================================================================
  SE2State start(1.0, 1.0, 0.0);
  SE2State goal(8.0, 8.0, 1.57);

  std::cout << "\n[test] Start: (" << start.x << ", " << start.y
            << ", " << start.yaw << ")\n";
  std::cout << "[test] Goal:  (" << goal.x << ", " << goal.y
            << ", " << goal.yaw << ")\n\n";

  // --- Stage 1: Topology search --------------------------------------------
  auto t1 = std::chrono::steady_clock::now();

  topo.buildRoadmap(start.position(), goal.position());
  std::vector<TopoPath> paths = topo.searchPaths();
  topo.shortenPaths(paths);

  auto t2 = std::chrono::steady_clock::now();
  double dt_topo_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

  std::cout << "[stage1] Topological paths found: " << paths.size()
            << "  (" << dt_topo_ms << " ms)\n";

  if (paths.empty()) {
    std::cerr << "[test] FAIL: no topological paths found.\n";
    return 1;
  }

  for (size_t i = 0; i < paths.size(); ++i) {
    std::cout << "  path[" << i << "]: length=" << paths[i].length
              << "  waypoints=" << paths[i].points.size() << "\n";
  }

  // --- Stage 2 + 3: SE(2) sequence generation + trajectory optimization ----
  auto t3 = std::chrono::steady_clock::now();

  std::vector<Trajectory> candidates;
  candidates.reserve(paths.size());

  for (size_t pi = 0; pi < paths.size(); ++pi) {
    std::cout << "\n--- Path " << pi << " ---\n";

    // Stage 2: SE(2) motion sequence
    std::vector<MotionSegment> segments = se2gen.generate(paths[pi], start, goal);
    std::cout << "  [stage2] Segments: " << segments.size() << "\n";

    int low_count = 0, high_count = 0;
    size_t total_wps = 0;
    for (size_t si = 0; si < segments.size(); ++si) {
      const char* risk_str =
          (segments[si].risk == RiskLevel::LOW) ? "LOW" : "HIGH";
      std::cout << "    seg[" << si << "]: risk=" << risk_str
                << "  waypoints=" << segments[si].waypoints.size() << "\n";
      if (segments[si].risk == RiskLevel::LOW) ++low_count;
      else ++high_count;
      total_wps += segments[si].waypoints.size();
    }
    std::cout << "  [stage2] Risk: LOW=" << low_count
              << "  HIGH=" << high_count
              << "  total_waypoints=" << total_wps << "\n";

    if (segments.empty()) continue;

    // Stage 3: Trajectory optimization per segment
    double total_time = paths[pi].length / opt_params.max_vel;
    if (total_time < 1.0) total_time = 1.0;

    std::vector<Trajectory> seg_trajs;
    seg_trajs.reserve(segments.size());

    for (size_t si = 0; si < segments.size(); ++si) {
      double seg_time = total_time *
          static_cast<double>(segments[si].waypoints.size()) /
          static_cast<double>(std::max(total_wps, static_cast<size_t>(1)));
      if (seg_time < 0.5) seg_time = 0.5;

      Trajectory seg_traj;
      if (segments[si].risk == RiskLevel::HIGH) {
        seg_traj = optimizer.optimizeSE2(segments[si].waypoints, seg_time);
      } else {
        seg_traj = optimizer.optimizeR2(segments[si].waypoints, seg_time);
      }
      seg_trajs.push_back(seg_traj);
    }

    Trajectory full_traj = optimizer.stitch(segments, seg_trajs);
    if (!full_traj.empty()) {
      candidates.push_back(full_traj);
      std::cout << "  [stage3] Trajectory: duration=" << full_traj.totalDuration()
                << " s  pieces=" << full_traj.pos_pieces.size() << "\n";
    } else {
      std::cout << "  [stage3] Trajectory: EMPTY (optimization failed)\n";
    }
  }

  auto t4 = std::chrono::steady_clock::now();
  double dt_opt_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
  std::cout << "\n[stage2+3] SE2 + optimization time: " << dt_opt_ms << " ms\n";

  if (candidates.empty()) {
    std::cerr << "[test] FAIL: no valid trajectory candidates.\n";
    return 1;
  }

  // --- Select best trajectory ----------------------------------------------
  Trajectory best = optimizer.selectBest(candidates);
  if (best.empty()) {
    std::cerr << "[test] FAIL: selectBest returned empty trajectory.\n";
    return 1;
  }

  // =========================================================================
  // 4. Diagnostics
  // =========================================================================
  double best_duration = best.totalDuration();
  int best_pieces = static_cast<int>(best.pos_pieces.size());

  // Evaluate minimum SVSDF along the best trajectory
  double min_svsdf = std::numeric_limits<double>::max();
  {
    double dt_sample = 0.05;
    for (double t = 0.0; t <= best_duration; t += dt_sample) {
      SE2State st = best.sample(t);
      double sv = svsdf.evaluate(st);
      if (sv < min_svsdf) min_svsdf = sv;
    }
  }

  auto t_total_end = std::chrono::steady_clock::now();
  double dt_total_ms =
      std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

  bool pass = (min_svsdf > -opt_params.safety_margin) && (best_pieces > 0);

  std::cout << "\n========================================\n";
  std::cout << "  DIAGNOSTICS\n";
  std::cout << "========================================\n";
  std::cout << "  Map size:            " << grid_map.width() << " x "
            << grid_map.height() << "\n";
  std::cout << "  ESDF range:          [" << esdf_min << ", " << esdf_max << "]\n";
  std::cout << "  Topo paths:          " << paths.size() << "\n";
  std::cout << "  Best traj duration:  " << best_duration << " s\n";
  std::cout << "  Best traj pieces:    " << best_pieces << "\n";
  std::cout << "  Min SVSDF:           " << min_svsdf << "\n";
  std::cout << "  Topology time:       " << dt_topo_ms << " ms\n";
  std::cout << "  SE2+Optim time:      " << dt_opt_ms << " ms\n";
  std::cout << "  Total time:          " << dt_total_ms << " ms\n";
  std::cout << "  Status:              " << (pass ? "PASS" : "FAIL") << "\n";
  std::cout << "========================================\n";

  return pass ? 0 : 1;
}
