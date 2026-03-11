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
#include <string>
#include <vector>

using namespace esv_planner;

namespace {

struct RobotCase {
  std::string name;
  std::vector<Eigen::Vector2d> verts;
  double inscribed_radius = 0.10;
};

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

double segmentLength(const std::vector<SE2State>& wps) {
  if (wps.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < wps.size(); ++i) {
    len += (wps[i].position() - wps[i - 1].position()).norm();
  }
  return len;
}

double topoPathClearance(const TopoPath& path,
                         const ContinuousSvsdfEvaluator& svsdf,
                         const SE2State& start,
                         const SE2State& goal) {
  if (path.waypoints.empty()) return -kInf;
  double min_clearance = svsdf.evaluate(start);
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    min_clearance =
        std::min(min_clearance, svsdf.evaluate(SE2State(wp.pos.x(), wp.pos.y(), yaw)));
  }
  min_clearance = std::min(min_clearance, svsdf.evaluate(goal));
  return min_clearance;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_stitch_seam_connector", ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string map_path =
      "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";

  int width = 0, height = 0, maxval = 0;
  std::vector<uint8_t> pixels;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    return 1;
  }

  auto occ = pgmToOccupancyGrid(pixels, width, height, maxval, 0.05);
  GridMap map;
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.computeEsdf();

  const SE2State start(1.0, 8.5, 0.0);
  const SE2State goal(8.5, 1.0, -1.57);
  const std::vector<RobotCase> robots = {
      {"T", {{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
             {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}}, 0.10},
      {"L", {{0.30, 0.30}, {0.30, 0.10}, {-0.10, 0.10},
             {-0.10, -0.30}, {-0.30, -0.30}, {-0.30, 0.30}}, 0.10},
  };

  bool all_robots_ok = true;
  for (const auto& robot : robots) {
    FootprintModel footprint;
    footprint.setPolygon(robot.verts);
    footprint.setInscribedRadius(robot.inscribed_radius);

    CollisionChecker checker;
    checker.init(map, footprint, 18);

    TopologyPlanner topo;
    topo.init(map, checker, 800, 20, 14, footprint.inscribedRadius());
    topo.buildRoadmap(start.position(), goal.position());
    auto paths = topo.searchPaths();
    topo.shortenPaths(paths);

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

    std::vector<size_t> topo_order(paths.size());
    for (size_t i = 0; i < paths.size(); ++i) topo_order[i] = i;
    std::sort(topo_order.begin(), topo_order.end(),
              [&](size_t lhs, size_t rhs) {
                const double lhs_clearance = topoPathClearance(paths[lhs], svsdf, start, goal);
                const double rhs_clearance = topoPathClearance(paths[rhs], svsdf, start, goal);
                if (std::abs(lhs_clearance - rhs_clearance) > 1e-6) {
                  return lhs_clearance > rhs_clearance;
                }
                return paths[lhs].length < paths[rhs].length;
              });

    bool found_segment_safe_path = false;
    bool seam_regression_found = false;

    for (size_t rank = 0; rank < topo_order.size(); ++rank) {
      const size_t path_idx = topo_order[rank];
      auto segments = se2gen.generate(paths[path_idx], start, goal);
      if (segments.empty()) continue;

      std::vector<Trajectory> seg_trajs(segments.size());
      std::vector<double> seg_times(segments.size(), 0.5);
      bool all_segments_safe = true;
      double min_segment_clearance = kInf;

      for (size_t si = 0; si < segments.size(); ++si) {
        seg_times[si] = std::max(0.5, segmentLength(segments[si].waypoints) /
                                        std::max(0.10, params.max_vel));
        Trajectory tr;
        if (segments[si].risk == RiskLevel::HIGH) {
          tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
        } else {
          tr = optimizer.optimizeR2(segments[si].waypoints, seg_times[si]);
        }
        if (tr.empty()) {
          all_segments_safe = false;
          break;
        }
        const double seg_clearance = svsdf.evaluateTrajectory(tr);
        std::cout << "[test] robot=" << robot.name
                  << " path=" << path_idx
                  << " seg=" << si
                  << " risk=" << (segments[si].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
                  << " seg_clearance=" << seg_clearance << "\n";
        if (seg_clearance + 1e-6 < params.safety_margin) {
          all_segments_safe = false;
          break;
        }
        min_segment_clearance = std::min(min_segment_clearance, seg_clearance);
        seg_trajs[si] = tr;
      }

      if (!all_segments_safe) continue;
      found_segment_safe_path = true;

      const Trajectory full = optimizer.stitch(segments, seg_trajs);
      const double stitched_clearance =
          full.empty() ? -kInf : svsdf.evaluateTrajectory(full);
      double max_seam_pos_jump = 0.0;
      double max_seam_yaw_jump = 0.0;
      for (size_t si = 0; si + 1 < seg_trajs.size(); ++si) {
        const SE2State left = seg_trajs[si].sample(seg_trajs[si].totalDuration());
        const SE2State right = seg_trajs[si + 1].sample(0.0);
        max_seam_pos_jump =
            std::max(max_seam_pos_jump, (left.position() - right.position()).norm());
        max_seam_yaw_jump = std::max(
            max_seam_yaw_jump, std::abs(normalizeAngle(left.yaw - right.yaw)));
      }
      double raw_seam_pos_jump = 0.0;
      double raw_seam_yaw_jump = 0.0;
      for (size_t si = 0; si + 1 < segments.size(); ++si) {
        const SE2State& left = segments[si].waypoints.back();
        const SE2State& right = segments[si + 1].waypoints.front();
        raw_seam_pos_jump =
            std::max(raw_seam_pos_jump, (left.position() - right.position()).norm());
        raw_seam_yaw_jump = std::max(
            raw_seam_yaw_jump, std::abs(normalizeAngle(left.yaw - right.yaw)));
      }
      std::cout << "[test] robot=" << robot.name
                << " path=" << path_idx
                << " min_segment_clearance=" << min_segment_clearance
                << " stitched_clearance=" << stitched_clearance
                << " raw_seam_pos_jump=" << raw_seam_pos_jump
                << " raw_seam_yaw_jump=" << raw_seam_yaw_jump
                << " seam_pos_jump=" << max_seam_pos_jump
                << " seam_yaw_jump=" << max_seam_yaw_jump
                << " stitched_duration=" << full.totalDuration() << "\n";

      if (stitched_clearance + 1e-6 < params.safety_margin ||
          stitched_clearance + 0.025 < min_segment_clearance) {
        seam_regression_found = true;
        break;
      }
    }

    if (!found_segment_safe_path) {
      std::cerr << "[test] FAIL: did not find any maze/" << robot.name
                << " path whose segments are all >= safety_margin\n";
      all_robots_ok = false;
    }
    if (seam_regression_found) {
      std::cerr << "[test] FAIL: stitch degraded a segment-safe path below the safety target for robot "
                << robot.name << "\n";
      all_robots_ok = false;
    }
  }

  if (!all_robots_ok) {
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
