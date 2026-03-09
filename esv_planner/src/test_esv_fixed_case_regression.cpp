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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace esv_planner;

namespace {

struct FixedCaseReport {
  size_t topo_paths = 0;
  size_t accepted_candidates = 0;
  bool final_esv_valid = false;
  double best_topo_waypoint_clearance = -kInf;
  double best_sequence_chain_clearance = -kInf;
  double best_segment_optimized_clearance = -kInf;
  double best_r2_clearance = -kInf;
  double best_se2_clearance = -kInf;
  double best_stitched_clearance = -kInf;
  double best_final_accepted_clearance = -kInf;
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
    const double dx = wps[i].x - wps[i - 1].x;
    const double dy = wps[i].y - wps[i - 1].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

double waypointPathClearance(const std::vector<SE2State>& waypoints,
                             const SvsdfEvaluator& svsdf) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (const auto& wp : waypoints) {
    min_clearance = std::min(min_clearance, svsdf.evaluate(wp));
  }
  return min_clearance;
}

double continuousChainClearance(const std::vector<SE2State>& waypoints,
                                const SvsdfEvaluator& svsdf,
                                double sample_step) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
    const double len = (b.position() - a.position()).norm();
    const int n_steps = std::max(1, static_cast<int>(std::ceil(len / sample_step)));
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
                         const SvsdfEvaluator& svsdf,
                         const SE2State& start,
                         const SE2State& goal) {
  if (path.waypoints.empty()) return -kInf;

  std::vector<SE2State> sampled;
  sampled.reserve(path.waypoints.size() + 2);
  sampled.push_back(start);
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    sampled.emplace_back(wp.pos.x(), wp.pos.y(), yaw);
  }
  sampled.push_back(goal);
  return waypointPathClearance(sampled, svsdf);
}

void printFirstCollidingWaypoint(const TopoPath& path,
                                 const SvsdfEvaluator& svsdf,
                                 const CollisionChecker& checker,
                                 const SE2State& start,
                                 const SE2State& goal) {
  std::vector<SE2State> sampled;
  sampled.reserve(path.waypoints.size() + 2);
  sampled.push_back(start);
  for (const auto& wp : path.waypoints) {
    const double yaw = wp.has_yaw ? wp.yaw : start.yaw;
    sampled.emplace_back(wp.pos.x(), wp.pos.y(), yaw);
  }
  sampled.push_back(goal);

  for (size_t i = 0; i < sampled.size(); ++i) {
    const double clearance = svsdf.evaluate(sampled[i]);
    if (clearance < 0.0) {
      const auto safe_bins = checker.safeYawIndices(sampled[i].x, sampled[i].y);
      double nearest_safe_yaw = 0.0;
      double nearest_safe_err = kInf;
      for (int bin : safe_bins) {
        const double yaw = checker.yawFromBin(bin);
        const double err = std::abs(normalizeAngle(yaw - sampled[i].yaw));
        if (err < nearest_safe_err) {
          nearest_safe_err = err;
          nearest_safe_yaw = yaw;
        }
      }
      std::cout << "[test]     first_negative_waypoint_idx=" << i
                << " pose=(" << sampled[i].x << ", " << sampled[i].y << ", " << sampled[i].yaw << ")"
                << " clearance=" << clearance
                << " safe_yaw_count=" << safe_bins.size();
      if (!safe_bins.empty()) {
        std::cout << " nearest_safe_yaw=" << nearest_safe_yaw
                  << " nearest_safe_err=" << nearest_safe_err;
      }
      std::cout << "\n";
      return;
    }
  }
}

bool trajectoryValid(const Trajectory& traj,
                     const SvsdfEvaluator& svsdf,
                     const TrajectoryOptimizer& optimizer,
                     double* out_min_svsdf = nullptr) {
  if (traj.empty()) return false;
  const double min_svsdf = svsdf.evaluateTrajectory(traj, 0.05);
  if (out_min_svsdf) *out_min_svsdf = min_svsdf;
  return min_svsdf >= 0.0 && optimizer.dynamicsFeasible(traj);
}

FixedCaseReport runFixedCase() {
  const std::string map_path = "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";

  int width = 0, height = 0, maxval = 0;
  std::vector<uint8_t> pixels;
  FixedCaseReport report;
  if (!loadPgm(map_path, width, height, maxval, pixels)) {
    return report;
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

  SE2SequenceGenerator se2gen;
  se2gen.init(map, checker, 0.15, 5);

  SvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

  OptimizerParams params;
  params.max_iterations = 100;
  params.lambda_smooth = 1.0;
  params.lambda_time = 1.0;
  params.lambda_safety = 10.0;
  params.lambda_dynamics = 1.0;
  params.lambda_pos_residual = 5.0;
  params.lambda_yaw_residual = 2.0;
  params.max_vel = 1.0;
  params.max_acc = 2.0;
  params.max_yaw_rate = 1.5;
  params.safety_margin = 0.1;
  params.step_size = 0.005;

  TrajectoryOptimizer optimizer;
  optimizer.init(map, svsdf, params);

  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);

  topology.buildRoadmap(start.position(), goal.position());
  auto topo_paths = topology.searchPaths();
  const auto raw_topo_paths = topo_paths;
  topology.shortenPaths(topo_paths);
  report.topo_paths = topo_paths.size();

  const double start_clearance = svsdf.evaluate(start);
  const double goal_clearance = svsdf.evaluate(goal);
  std::cout << "[test] target_case_start=(" << start.x << ", " << start.y << ", " << start.yaw << ")\n";
  std::cout << "[test] target_case_goal=(" << goal.x << ", " << goal.y << ", " << goal.yaw << ")\n";
  std::cout << "[test] start_clearance=" << start_clearance
            << " start_free=" << (checker.isFree(start) ? 1 : 0)
            << " goal_clearance=" << goal_clearance
            << " goal_free=" << (checker.isFree(goal) ? 1 : 0) << "\n";
  std::cout << "[test] topo_paths=" << topo_paths.size() << "\n";

  for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
    double raw_topo_clearance = -kInf;
    if (pi < raw_topo_paths.size()) {
      raw_topo_clearance = topoPathClearance(raw_topo_paths[pi], svsdf, start, goal);
    }
    const double topo_clearance = topoPathClearance(topo_paths[pi], svsdf, start, goal);
    report.best_topo_waypoint_clearance = std::max(report.best_topo_waypoint_clearance, topo_clearance);
    std::cout << "[test] path=" << pi
              << " raw_topo_waypoints=" << (pi < raw_topo_paths.size() ? raw_topo_paths[pi].waypoints.size() : 0)
              << " raw_topo_clearance=" << raw_topo_clearance
              << " topo_waypoints=" << topo_paths[pi].waypoints.size()
              << " topo_clearance=" << topo_clearance << "\n";
    if (topo_clearance < 0.0) {
      printFirstCollidingWaypoint(topo_paths[pi], svsdf, checker, start, goal);
    }

    auto segments = se2gen.generate(topo_paths[pi], start, goal);
    if (segments.empty()) {
      std::cout << "[test]   segments=0\n";
      continue;
    }

    std::vector<Trajectory> seg_trajs(segments.size());
    std::vector<double> seg_times(segments.size(), 0.5);
    bool path_ok = true;

    for (size_t si = 0; si < segments.size(); ++si) {
      seg_times[si] = std::max(0.5, segmentLength(segments[si].waypoints) /
                                      std::max(0.10, params.max_vel));
      const double raw_seg_clearance = waypointPathClearance(segments[si].waypoints, svsdf);
      const double chain_clearance = continuousChainClearance(
          segments[si].waypoints, svsdf, 0.05);
      report.best_sequence_chain_clearance =
          std::max(report.best_sequence_chain_clearance, chain_clearance);
      std::cout << "[test]   seg=" << si
                << " risk=" << (segments[si].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
                << " wps=" << segments[si].waypoints.size()
                << " raw_clearance=" << raw_seg_clearance
                << " chain_clearance=" << chain_clearance << "\n";

      Trajectory tr;
      double min_svsdf = -kInf;
      if (segments[si].risk == RiskLevel::HIGH) {
        tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
        if (!tr.empty()) {
          min_svsdf = svsdf.evaluateTrajectory(tr, 0.05);
          report.best_se2_clearance = std::max(report.best_se2_clearance, min_svsdf);
          report.best_segment_optimized_clearance =
              std::max(report.best_segment_optimized_clearance, min_svsdf);
        }
        std::cout << "[test]     se2_clearance=" << min_svsdf
                  << " traj_empty=" << (tr.empty() ? 1 : 0) << "\n";
        if (tr.empty()) {
          path_ok = false;
          break;
        }
      } else {
        tr = optimizer.optimizeR2(segments[si].waypoints, seg_times[si]);
        if (tr.empty()) {
          std::cout << "[test]     r2_upgrade_to_se2=1\n";
          tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
          if (!tr.empty()) {
            segments[si].risk = RiskLevel::HIGH;
          }
        }
        if (!tr.empty()) {
          min_svsdf = svsdf.evaluateTrajectory(tr, 0.05);
          report.best_r2_clearance = std::max(report.best_r2_clearance, min_svsdf);
          report.best_segment_optimized_clearance =
              std::max(report.best_segment_optimized_clearance, min_svsdf);
        }
        std::cout << "[test]     r2_clearance=" << min_svsdf
                  << " traj_empty=" << (tr.empty() ? 1 : 0) << "\n";
      }

      if (tr.empty()) {
        path_ok = false;
        break;
      }
      seg_trajs[si] = tr;
    }

    if (!path_ok) continue;

    Trajectory full = optimizer.stitch(segments, seg_trajs);
    double stitched_clearance = -kInf;
    if (!full.empty()) {
      stitched_clearance = svsdf.evaluateTrajectory(full, 0.05);
      report.best_stitched_clearance = std::max(report.best_stitched_clearance, stitched_clearance);
    }
    std::cout << "[test]   stitched_clearance=" << stitched_clearance
              << " stitched_empty=" << (full.empty() ? 1 : 0)
              << " dynamics_ok=" << (optimizer.dynamicsFeasible(full) ? 1 : 0) << "\n";

    bool valid = trajectoryValid(full, svsdf, optimizer, &stitched_clearance);
    if (!valid) {
      bool fallback_ok = true;
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk == RiskLevel::HIGH) continue;
        Trajectory fallback_traj = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
        double fallback_clearance = -kInf;
        if (!fallback_traj.empty()) {
          fallback_clearance = svsdf.evaluateTrajectory(fallback_traj, 0.05);
        }
        std::cout << "[test]     fallback_se2_seg=" << si
                  << " clearance=" << fallback_clearance
                  << " traj_empty=" << (fallback_traj.empty() ? 1 : 0) << "\n";
        if (fallback_traj.empty() || fallback_clearance < 0.0) {
          fallback_ok = false;
          break;
        }
        seg_trajs[si] = fallback_traj;
      }

      if (fallback_ok) {
        full = optimizer.stitch(segments, seg_trajs);
        valid = trajectoryValid(full, svsdf, optimizer, &stitched_clearance);
        std::cout << "[test]   fallback_stitched_clearance=" << stitched_clearance
                  << " fallback_dynamics_ok=" << (optimizer.dynamicsFeasible(full) ? 1 : 0) << "\n";
      }
    }

    if (valid) {
      ++report.accepted_candidates;
      report.final_esv_valid = true;
      report.best_final_accepted_clearance =
          std::max(report.best_final_accepted_clearance, stitched_clearance);
    }
  }

  return report;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_esv_fixed_case_regression",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  const FixedCaseReport report = runFixedCase();

  std::cout << "[test] summary topo_paths=" << report.topo_paths
            << " accepted_candidates=" << report.accepted_candidates
            << " final_esv_valid=" << (report.final_esv_valid ? 1 : 0)
            << " best_topo_clearance=" << report.best_topo_waypoint_clearance
            << " best_sequence_chain_clearance=" << report.best_sequence_chain_clearance
            << " best_segment_optimized_clearance=" << report.best_segment_optimized_clearance
            << " best_r2_clearance=" << report.best_r2_clearance
            << " best_se2_clearance=" << report.best_se2_clearance
            << " best_stitched_clearance=" << report.best_stitched_clearance
            << " best_final_accepted_clearance=" << report.best_final_accepted_clearance
            << "\n";

  if (report.topo_paths == 0) {
    std::cerr << "[test] FAIL: topology stage produced no paths for the fixed case\n";
    return 1;
  }
  if (!report.final_esv_valid) {
    std::cerr << "[test] FAIL: fixed case still requires fallback; use diagnostics above to identify the first failing layer\n";
    return 1;
  }
  if (report.accepted_candidates < 1) {
    std::cerr << "[test] FAIL: expected at least one accepted ESV candidate for the fixed case\n";
    return 1;
  }
  if (report.best_final_accepted_clearance < 0.0) {
    std::cerr << "[test] FAIL: accepted fixed-case trajectory must satisfy min_svsdf >= 0.0\n";
    return 1;
  }

  return 0;
}
