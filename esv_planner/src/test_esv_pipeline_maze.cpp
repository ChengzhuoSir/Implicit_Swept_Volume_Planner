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
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace esv_planner;

namespace {

struct RobotShape {
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
    int pgm_row = height - 1 - row;
    for (int col = 0; col < width; ++col) {
      uint8_t pix = pixels[static_cast<size_t>(pgm_row) * width + col];
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
    double dx = wps[i].x - wps[i - 1].x;
    double dy = wps[i].y - wps[i - 1].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

double segmentChainClearance(const std::vector<SE2State>& waypoints,
                             const SvsdfEvaluator& svsdf,
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

Trajectory buildLinearTrajectory(const std::vector<SE2State>& waypoints,
                                 double total_time) {
  Trajectory traj;
  if (waypoints.size() < 2) return traj;

  double total_len = 0.0;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    total_len += (waypoints[i + 1].position() - waypoints[i].position()).norm();
  }
  if (total_len < 1e-9) total_len = 1.0;

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto& a = waypoints[i];
    const auto& b = waypoints[i + 1];
    const double seg_len = (b.position() - a.position()).norm();
    const double T = std::max(0.05, total_time * seg_len / total_len);
    const Eigen::Vector2d delta = b.position() - a.position();

    PolyPiece pp;
    pp.duration = T;
    pp.coeffs.col(0) = a.position();
    pp.coeffs.col(1) = delta / T;
    pp.coeffs.col(2).setZero();
    pp.coeffs.col(3).setZero();
    pp.coeffs.col(4).setZero();
    pp.coeffs.col(5).setZero();
    traj.pos_pieces.push_back(pp);

    YawPolyPiece yp;
    yp.duration = T;
    yp.coeffs(0, 0) = a.yaw;
    yp.coeffs(0, 1) = normalizeAngle(b.yaw - a.yaw) / T;
    yp.coeffs(0, 2) = 0.0;
    yp.coeffs(0, 3) = 0.0;
    yp.coeffs(0, 4) = 0.0;
    yp.coeffs(0, 5) = 0.0;
    traj.yaw_pieces.push_back(yp);
  }
  return traj;
}

bool evaluateTrajectory(const Trajectory& traj,
                        const SvsdfEvaluator& svsdf,
                        const OptimizerParams& params) {
  if (traj.empty()) return false;
  double min_svsdf = svsdf.evaluateTrajectory(traj, 0.05);
  double max_vel = 0.0;
  double max_acc = 0.0;
  double max_yaw_rate = 0.0;
  const double total = traj.totalDuration();
  for (double t = 0.0; t <= total + 1e-9; t += 0.02) {
    const double tt = std::min(t, total);
    max_vel = std::max(max_vel, traj.sampleVelocity(tt).norm());
    max_acc = std::max(max_acc, traj.sampleAcceleration(tt).norm());

    double acc = 0.0;
    for (size_t i = 0; i < traj.yaw_pieces.size(); ++i) {
      const auto& yp = traj.yaw_pieces[i];
      if (tt <= acc + yp.duration || i + 1 == traj.yaw_pieces.size()) {
        max_yaw_rate = std::max(max_yaw_rate, std::abs(yp.velocity(tt - acc)));
        break;
      }
      acc += yp.duration;
    }
  }

  return min_svsdf >= 0.0 &&
         max_vel <= params.max_vel * 1.10 &&
         max_acc <= params.max_acc * 1.10 &&
         max_yaw_rate <= params.max_yaw_rate * 1.10;
}

Trajectory runPaperAlignedEsv(const std::vector<TopoPath>& topo_paths,
                              const SE2State& start,
                              const SE2State& goal,
                              SE2SequenceGenerator& se2gen,
                              SvsdfEvaluator& svsdf,
                              TrajectoryOptimizer& optimizer,
                              const OptimizerParams& params,
                              int& accepted_candidates,
                              int& total_high_risk_segments,
                              int& accepted_high_risk_segments) {
  std::vector<Trajectory> candidates;
  accepted_candidates = 0;
  total_high_risk_segments = 0;
  accepted_high_risk_segments = 0;

  for (size_t path_idx = 0; path_idx < topo_paths.size(); ++path_idx) {
    const auto& path = topo_paths[path_idx];
    auto segments = se2gen.generate(path, start, goal);
    if (segments.empty()) continue;

    int path_high_risk_segments = 0;
    for (const auto& seg : segments) {
      if (seg.risk == RiskLevel::HIGH) {
        ++total_high_risk_segments;
        ++path_high_risk_segments;
      }
    }
    std::cout << "[test] path_high_risk_segments=" << path_high_risk_segments
              << " path_segments=" << segments.size() << "\n";
    for (const auto& seg : segments) {
      const double chain_clearance =
          segmentChainClearance(seg.waypoints, svsdf, 0.025);
      const double nominal_time =
          std::max(0.5, segmentLength(seg.waypoints) / std::max(0.10, params.max_vel));
      const Trajectory linear_traj = buildLinearTrajectory(seg.waypoints, nominal_time);
      const double linear_clearance =
          linear_traj.empty() ? -kInf : svsdf.evaluateTrajectory(linear_traj, 0.05);
      std::cout << "[test]   seg risk="
                << (seg.risk == RiskLevel::HIGH ? "HIGH" : "LOW")
                << " size=" << seg.waypoints.size()
                << " chain_clearance=" << chain_clearance
                << " linear_traj_clearance=" << linear_clearance << "\n";
    }

    std::vector<Trajectory> seg_trajs(segments.size());
    std::vector<double> seg_times(segments.size(), 0.5);
    for (size_t si = 0; si < segments.size(); ++si) {
      seg_times[si] = std::max(0.5, segmentLength(segments[si].waypoints) /
                                      std::max(0.10, params.max_vel));
    }

    bool ok = true;
    for (size_t si = 0; si < segments.size(); ++si) {
      if (segments[si].risk != RiskLevel::HIGH) continue;
      Trajectory tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
      if (tr.empty()) {
        std::cout << "[test]   path=" << path_idx
                  << " reject: high segment " << si << " produced empty SE2 traj\n";
        ok = false;
        break;
      }
      seg_trajs[si] = tr;
    }
    if (!ok) continue;

    for (size_t si = 0; si < segments.size(); ++si) {
      if (segments[si].risk == RiskLevel::HIGH) continue;
      Trajectory tr = optimizer.optimizeR2(segments[si].waypoints, seg_times[si]);
      if (tr.empty()) {
        std::cout << "[test]   path=" << path_idx
                  << " reject: low segment " << si << " produced empty R2 traj\n";
        ok = false;
        break;
      }
      seg_trajs[si] = tr;
    }
    if (!ok) continue;

    Trajectory full = optimizer.stitch(segments, seg_trajs);
    bool valid = evaluateTrajectory(full, svsdf, params);
    const double full_clearance = full.empty() ? -kInf : svsdf.evaluateTrajectory(full, 0.05);
    std::cout << "[test]   path=" << path_idx
              << " stitched_clearance=" << full_clearance
              << " stitched_valid=" << valid << "\n";
    if (!valid) {
      bool fallback_ok = true;
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk == RiskLevel::HIGH) continue;
        Trajectory tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
        if (tr.empty() || svsdf.evaluateTrajectory(tr, 0.05) < 0.0) {
          std::cout << "[test]   path=" << path_idx
                    << " fallback failed on low segment " << si
                    << " clearance="
                    << (tr.empty() ? -kInf : svsdf.evaluateTrajectory(tr, 0.05)) << "\n";
          fallback_ok = false;
          break;
        }
        seg_trajs[si] = tr;
      }
      if (fallback_ok) {
        full = optimizer.stitch(segments, seg_trajs);
        valid = evaluateTrajectory(full, svsdf, params);
        const double fallback_clearance =
            full.empty() ? -kInf : svsdf.evaluateTrajectory(full, 0.05);
        std::cout << "[test]   path=" << path_idx
                  << " fallback_stitched_clearance=" << fallback_clearance
                  << " fallback_valid=" << valid << "\n";
      }
    }

    if (valid) {
      candidates.push_back(full);
      ++accepted_candidates;
      accepted_high_risk_segments += path_high_risk_segments;
      std::cout << "[test]   path=" << path_idx << " accepted\n";
    } else {
      std::cout << "[test]   path=" << path_idx << " rejected\n";
    }
  }

  if (candidates.empty()) return Trajectory();
  return optimizer.selectBest(candidates);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_esv_pipeline_maze", ros::init_options::NoSigintHandler);
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
  map.computeEsdf();

  std::vector<RobotShape> robots = {
      {"T", {{0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
             {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}}, 0.10},
      {"L", {{0.30, 0.30}, {0.30, 0.10}, {-0.10, 0.10},
             {-0.10, -0.30}, {-0.30, -0.30}, {-0.30, 0.30}}, 0.10},
  };

  const SE2State start(1.0, 8.5, 0.0);
  const SE2State goal(8.5, 1.0, -1.57);

  bool all_valid = true;
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

    SvsdfEvaluator svsdf;
    svsdf.init(map, footprint);

    OptimizerParams params;
    params.max_iterations = 100;
    params.lambda_safety = 10.0;
    params.safety_margin = 0.1;

    TrajectoryOptimizer optimizer;
    optimizer.init(map, svsdf, params);

    int accepted_candidates = 0;
    int total_high_risk_segments = 0;
    int accepted_high_risk_segments = 0;
    Trajectory traj = runPaperAlignedEsv(paths, start, goal, se2gen,
                                         svsdf, optimizer, params,
                                         accepted_candidates,
                                         total_high_risk_segments,
                                         accepted_high_risk_segments);

    std::cout << "[test] robot=" << robot.name
              << " topo_paths=" << paths.size()
              << " accepted_candidates=" << accepted_candidates
              << " high_risk_segments=" << total_high_risk_segments
              << " accepted_high_risk_segments=" << accepted_high_risk_segments
              << " traj_empty=" << traj.empty() << "\n";

    if (accepted_candidates < 2) {
      std::cerr << "[test] FAIL: expected at least two accepted maze candidates for robot "
                << robot.name << "\n";
      all_valid = false;
    }

    if (traj.empty()) {
      std::cerr << "[test] FAIL: expected a valid final trajectory for robot "
                << robot.name << "\n";
      all_valid = false;
    }

    if (robot.name == "T" && accepted_high_risk_segments > 3) {
      std::cerr << "[test] FAIL: expected at most 3 accepted high-risk segments for robot T\n";
      all_valid = false;
    }
    if (robot.name == "L" && accepted_high_risk_segments > 2) {
      std::cerr << "[test] FAIL: expected at most 2 accepted high-risk segments for robot L\n";
      all_valid = false;
    }
  }

  if (!all_valid) {
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
