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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace esv_planner;

namespace {

struct Scenario {
  std::string name;
  std::string map_path;
  SE2State start;
  SE2State goal;
};

struct RobotShape {
  std::string name;
  std::vector<Eigen::Vector2d> verts;
  double inscribed_radius = 0.10;
};

struct Metrics {
  bool success = false;
  double runtime_ms = 0.0;
  double duration_s = 0.0;
  double length_m = 0.0;
  double min_svsdf = -std::numeric_limits<double>::infinity();
  double max_vel = 0.0;
  double max_acc = 0.0;
  int num_pieces = 0;
  int num_topo_paths = 0;
};

struct Config {
  std::string office_map = "/home/chengzhuo/workspace/plan/src/esv_planner/maps/office.pgm";
  std::string maze_map = "/home/chengzhuo/workspace/plan/src/esv_planner/maps/maze.pgm";
  std::string metrics_csv = "/home/chengzhuo/workspace/plan/src/esv_planner/benchmark_metrics.csv";
  std::string traj_csv = "/home/chengzhuo/workspace/plan/src/esv_planner/benchmark_trajectories.csv";
  int runs = 10;
};

// ---------------------------------------------------------------------------
// PGM loader
// ---------------------------------------------------------------------------
bool loadPgm(const std::string& path,
             int& width, int& height, int& maxval,
             std::vector<uint8_t>& pixels) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    std::cerr << "[bench] cannot open map: " << path << "\n";
    return false;
  }

  std::string magic;
  ifs >> magic;
  if (magic != "P5") {
    std::cerr << "[bench] map is not P5 pgm: " << path << "\n";
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
      int8_t occ;
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

std::vector<SE2State> flattenSegments(const std::vector<MotionSegment>& segs) {
  std::vector<SE2State> out;
  for (size_t si = 0; si < segs.size(); ++si) {
    const auto& wps = segs[si].waypoints;
    for (size_t i = 0; i < wps.size(); ++i) {
      if (!out.empty() && i == 0) {
        double dx = out.back().x - wps[i].x;
        double dy = out.back().y - wps[i].y;
        double dyaw = normalizeAngle(out.back().yaw - wps[i].yaw);
        if (std::sqrt(dx * dx + dy * dy) < 1e-6 && std::abs(dyaw) < 1e-6) continue;
      }
      out.push_back(wps[i]);
    }
  }
  return out;
}

bool evaluateTrajectoryMetrics(const Trajectory& traj,
                               const SvsdfEvaluator& svsdf,
                               const OptimizerParams& params,
                               Metrics& m,
                               bool require_dynamics) {
  if (traj.empty()) return false;

  m.duration_s = traj.totalDuration();
  m.num_pieces = static_cast<int>(traj.pos_pieces.size());
  m.min_svsdf = svsdf.evaluateTrajectory(traj, 0.05);
  m.max_vel = 0.0;
  m.max_acc = 0.0;
  m.length_m = 0.0;

  SE2State prev = traj.sample(0.0);
  for (double t = 0.02; t <= m.duration_s + 1e-9; t += 0.02) {
    double tt = std::min(t, m.duration_s);
    SE2State st = traj.sample(tt);
    double dx = st.x - prev.x;
    double dy = st.y - prev.y;
    m.length_m += std::sqrt(dx * dx + dy * dy);
    prev = st;

    Eigen::Vector2d vel = traj.sampleVelocity(tt);
    Eigen::Vector2d acc = traj.sampleAcceleration(tt);
    m.max_vel = std::max(m.max_vel, vel.norm());
    m.max_acc = std::max(m.max_acc, acc.norm());
  }

  bool collision_ok = (m.min_svsdf >= -params.safety_margin);
  bool dynamics_ok = (m.max_vel <= params.max_vel * 1.10) &&
                     (m.max_acc <= params.max_acc * 1.10);
  return require_dynamics ? (collision_ok && dynamics_ok) : collision_ok;
}

Trajectory runPaperAlignedEsv(const std::vector<TopoPath>& topo_paths,
                              const SE2State& start,
                              const SE2State& goal,
                              SE2SequenceGenerator& se2gen,
                              SvsdfEvaluator& svsdf,
                              TrajectoryOptimizer& optimizer,
                              const OptimizerParams& params,
                              int& accepted_candidates) {
  std::vector<Trajectory> candidates;
  accepted_candidates = 0;

  for (const auto& path : topo_paths) {
    auto segments = se2gen.generate(path, start, goal);
    if (segments.empty()) continue;

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
      if (tr.empty() || svsdf.evaluateTrajectory(tr, 0.05) < -params.safety_margin) {
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
        ok = false;
        break;
      }
      seg_trajs[si] = tr;
    }
    if (!ok) continue;

    Trajectory full = optimizer.stitch(segments, seg_trajs);
    Metrics mm;
    bool valid = evaluateTrajectoryMetrics(full, svsdf, params, mm, false);
    if (!valid) {
      bool fallback_ok = true;
      for (size_t si = 0; si < segments.size(); ++si) {
        if (segments[si].risk == RiskLevel::HIGH) continue;
        Trajectory tr = optimizer.optimizeSE2(segments[si].waypoints, seg_times[si]);
        if (tr.empty() || svsdf.evaluateTrajectory(tr, 0.05) < -params.safety_margin) {
          fallback_ok = false;
          break;
        }
        seg_trajs[si] = tr;
      }
      if (fallback_ok) {
        full = optimizer.stitch(segments, seg_trajs);
        valid = evaluateTrajectoryMetrics(full, svsdf, params, mm, false);
      }
    }

    if (valid && !full.empty()) {
      candidates.push_back(full);
      ++accepted_candidates;
    }
  }

  if (candidates.empty()) return Trajectory();
  return optimizer.selectBest(candidates);
}

void writeTrajectoryCsv(std::ofstream& ofs,
                        const std::string& scenario,
                        const std::string& robot,
                        int run_id,
                        const std::string& method,
                        const std::string& topo_mode,
                        const Trajectory& traj) {
  if (traj.empty()) return;
  double total = traj.totalDuration();
  for (double t = 0.0; t <= total + 1e-9; t += 0.05) {
    double tt = std::min(t, total);
    SE2State st = traj.sample(tt);
    ofs << scenario << "," << robot << "," << run_id << ","
        << method << "," << topo_mode << ","
        << std::fixed << std::setprecision(4) << tt << ","
        << st.x << "," << st.y << "," << st.yaw << "\n";
  }
}

std::string argValue(int argc, char** argv, const std::string& key, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == key) return argv[i + 1];
  }
  return def;
}

int argInt(int argc, char** argv, const std::string& key, int def) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == key) return std::atoi(argv[i + 1]);
  }
  return def;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "benchmark_planner", ros::init_options::NoSigintHandler);
  ros::Time::init();

  Config cfg;
  cfg.office_map = argValue(argc, argv, "--map_office", cfg.office_map);
  cfg.maze_map = argValue(argc, argv, "--map_maze", cfg.maze_map);
  cfg.metrics_csv = argValue(argc, argv, "--csv", cfg.metrics_csv);
  cfg.traj_csv = argValue(argc, argv, "--traj_csv", cfg.traj_csv);
  cfg.runs = std::max(1, argInt(argc, argv, "--runs", cfg.runs));

  std::vector<Scenario> scenarios = {
      {"office", cfg.office_map, SE2State(1.0, 1.0, 0.0), SE2State(8.0, 8.0, 1.57)},
      {"maze", cfg.maze_map, SE2State(1.0, 8.5, 0.0), SE2State(8.5, 1.0, -1.57)}};

  std::vector<RobotShape> robots;
  {
    RobotShape t;
    t.name = "T";
    t.verts = {
        {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
        {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}};
    robots.push_back(t);
  }
  {
    RobotShape l;
    l.name = "L";
    l.verts = {
        {0.30, 0.30}, {0.30, 0.10}, {-0.10, 0.10},
        {-0.10, -0.30}, {-0.30, -0.30}, {-0.30, 0.30}};
    robots.push_back(l);
  }

  std::ofstream metrics_ofs(cfg.metrics_csv);
  std::ofstream traj_ofs(cfg.traj_csv);
  if (!metrics_ofs.is_open() || !traj_ofs.is_open()) {
    std::cerr << "[bench] cannot open output csv files\n";
    return 1;
  }

  metrics_ofs << "scenario,robot,run,method,topo_mode,success,runtime_ms,duration_s,"
                 "length_m,min_svsdf,max_vel,max_acc,num_pieces,num_topo_paths\n";
  traj_ofs << "scenario,robot,run,method,topo_mode,t,x,y,yaw\n";

  for (const auto& sc : scenarios) {
    int w = 0, h = 0, maxval = 0;
    std::vector<uint8_t> pix;
    if (!loadPgm(sc.map_path, w, h, maxval, pix)) {
      std::cerr << "[bench] skip scenario " << sc.name << " (map missing)\n";
      continue;
    }
    auto occ = pgmToOccupancyGrid(pix, w, h, maxval, 0.05);

    for (const auto& rb : robots) {
      for (int run = 0; run < cfg.runs; ++run) {
        // Shared modules for this run/robot/scenario.
        GridMap map;
        map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
        map.computeEsdf();

        FootprintModel fp;
        fp.setPolygon(rb.verts);
        fp.setInscribedRadius(rb.inscribed_radius);

        CollisionChecker checker;
        checker.init(map, fp, 18);
        checker.generateRobotKernels();

        TopologyPlanner topo;
        topo.init(map, checker, 800, 20, 14, fp.inscribedRadius());
        topo.buildRoadmap(sc.start.position(), sc.goal.position());
        auto paths = topo.searchPaths();
        topo.shortenPaths(paths);

        SE2SequenceGenerator se2gen;
        se2gen.init(map, checker, 0.15, 5);

        SvsdfEvaluator svsdf;
        svsdf.init(map, fp);

        OptimizerParams params;
        params.max_iterations = 100;
        params.lambda_safety = 10.0;
        params.safety_margin = 0.1;

        // ----------------------- ESV (proposed) ------------------------------
        {
          auto ts = std::chrono::steady_clock::now();
          TrajectoryOptimizer optimizer;
          optimizer.init(map, svsdf, params);

          int accepted = 0;
          Trajectory traj = runPaperAlignedEsv(paths, sc.start, sc.goal, se2gen,
                                               svsdf, optimizer, params, accepted);
          auto te = std::chrono::steady_clock::now();

          Metrics m;
          m.runtime_ms = std::chrono::duration<double, std::milli>(te - ts).count();
          m.num_topo_paths = accepted;
          m.success = evaluateTrajectoryMetrics(traj, svsdf, params, m, false);

          metrics_ofs << sc.name << "," << rb.name << "," << run
                      << ",ESV,multi," << (m.success ? 1 : 0) << ","
                      << std::fixed << std::setprecision(3)
                      << m.runtime_ms << "," << m.duration_s << "," << m.length_m << ","
                      << m.min_svsdf << "," << m.max_vel << "," << m.max_acc << ","
                      << m.num_pieces << "," << m.num_topo_paths << "\n";
          if (m.success) {
            writeTrajectoryCsv(traj_ofs, sc.name, rb.name, run, "ESV", "multi", traj);
          }
        }

        // Prepare path A/B for baselines.
        std::vector<std::pair<std::string, TopoPath>> baseline_paths;
        if (!paths.empty()) {
          baseline_paths.push_back({"A", paths.front()});
          if (paths.size() > 1 &&
              std::abs(paths.back().length - paths.front().length) > map.resolution()) {
            baseline_paths.push_back({"B", paths.back()});
          }
        }

        for (const auto& p : baseline_paths) {
          const std::string& topo_mode = p.first;
          const TopoPath& path = p.second;
          auto segs = se2gen.generate(path, sc.start, sc.goal);
          auto flat = flattenSegments(segs);
          if (flat.size() < 2) {
            continue;
          }
          double total_time = std::max(1.0, path.length / std::max(0.10, params.max_vel));

          // ------------------- SVSDF-like baseline --------------------------
          {
            auto ts = std::chrono::steady_clock::now();
            TrajectoryOptimizer optimizer;
            optimizer.init(map, svsdf, params);
            Trajectory traj = optimizer.optimizeSE2(flat, total_time);
            auto te = std::chrono::steady_clock::now();

            Metrics m;
            m.runtime_ms = std::chrono::duration<double, std::milli>(te - ts).count();
            m.num_topo_paths = 1;
            m.success = evaluateTrajectoryMetrics(traj, svsdf, params, m, false);

            metrics_ofs << sc.name << "," << rb.name << "," << run
                        << ",SVSDF_like," << topo_mode << ","
                        << (m.success ? 1 : 0) << ","
                        << std::fixed << std::setprecision(3)
                        << m.runtime_ms << "," << m.duration_s << "," << m.length_m << ","
                        << m.min_svsdf << "," << m.max_vel << "," << m.max_acc << ","
                        << m.num_pieces << "," << m.num_topo_paths << "\n";
            if (m.success) {
              writeTrajectoryCsv(traj_ofs, sc.name, rb.name, run,
                                 "SVSDF_like", topo_mode, traj);
            }
          }

          // ------------------- RC-ESDF-like baseline ------------------------
          {
            OptimizerParams rc_params = params;
            rc_params.lambda_safety = 0.0;  // classic discrete/path-guided tendency
            auto ts = std::chrono::steady_clock::now();
            TrajectoryOptimizer optimizer;
            optimizer.init(map, svsdf, rc_params);
            Trajectory traj = optimizer.optimizeR2(flat, total_time);
            auto te = std::chrono::steady_clock::now();

            Metrics m;
            m.runtime_ms = std::chrono::duration<double, std::milli>(te - ts).count();
            m.num_topo_paths = 1;
            m.success = evaluateTrajectoryMetrics(traj, svsdf, params, m, false);

            metrics_ofs << sc.name << "," << rb.name << "," << run
                        << ",RC_ESDF_like," << topo_mode << ","
                        << (m.success ? 1 : 0) << ","
                        << std::fixed << std::setprecision(3)
                        << m.runtime_ms << "," << m.duration_s << "," << m.length_m << ","
                        << m.min_svsdf << "," << m.max_vel << "," << m.max_acc << ","
                        << m.num_pieces << "," << m.num_topo_paths << "\n";
            if (m.success) {
              writeTrajectoryCsv(traj_ofs, sc.name, rb.name, run,
                                 "RC_ESDF_like", topo_mode, traj);
            }
          }
        }

        std::cout << "[bench] done: scenario=" << sc.name
                  << " robot=" << rb.name
                  << " run=" << run << "\n";
      }
    }
  }

  std::cout << "[bench] metrics: " << cfg.metrics_csv << "\n";
  std::cout << "[bench] traj:    " << cfg.traj_csv << "\n";
  return 0;
}
