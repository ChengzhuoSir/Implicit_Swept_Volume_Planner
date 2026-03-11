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

#include <cstdint>
#include <fstream>
#include <iostream>
#include <chrono>
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

double segmentLength(const std::vector<SE2State>& wps) {
  if (wps.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < wps.size(); ++i) {
    len += (wps[i].position() - wps[i - 1].position()).norm();
  }
  return len;
}

double continuousChainClearance(const std::vector<SE2State>& waypoints,
                                const ContinuousSvsdfEvaluator& svsdf,
                                double sample_step) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const SE2State& a = waypoints[i];
    const SE2State& b = waypoints[i + 1];
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

std::vector<SE2State> sampleTrajectoryStates(const Trajectory& traj, double sample_step) {
  std::vector<SE2State> states;
  if (traj.empty()) return states;
  const double total = traj.totalDuration();
  const int n_steps =
      std::max(1, static_cast<int>(std::ceil(total / std::max(1e-3, sample_step))));
  states.reserve(static_cast<size_t>(n_steps) + 1);
  for (int i = 0; i <= n_steps; ++i) {
    const double t = (i == n_steps) ? total : std::min(total, i * sample_step);
    states.push_back(traj.sample(t));
  }
  return states;
}

double pointToSegmentDistance(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom < 1e-12) return (p - a).norm();
  const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom));
  const Eigen::Vector2d proj = a + t * ab;
  return (p - proj).norm();
}

double maxNearObstacleWaviness(const Trajectory& traj,
                               const ContinuousSvsdfEvaluator& svsdf,
                               double clearance_threshold,
                               double sample_step,
                               int* out_samples = nullptr) {
  const auto states = sampleTrajectoryStates(traj, sample_step);
  if (states.size() < 3) {
    if (out_samples) *out_samples = 0;
    return 0.0;
  }

  int samples = 0;
  double max_waviness = 0.0;
  for (size_t i = 1; i + 1 < states.size(); ++i) {
    const double c0 = svsdf.evaluate(states[i - 1]);
    const double c1 = svsdf.evaluate(states[i]);
    const double c2 = svsdf.evaluate(states[i + 1]);
    if (std::min(c0, std::min(c1, c2)) > clearance_threshold) {
      continue;
    }
    ++samples;
    max_waviness = std::max(max_waviness, pointToSegmentDistance(
        states[i].position(), states[i - 1].position(), states[i + 1].position()));
  }
  if (out_samples) *out_samples = samples;
  return max_waviness;
}

}  // namespace

int main(int argc, char** argv) {
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  ros::init(argc, argv, "test_optimizer_se2_fixed_high_segments",
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
  footprint.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);

  CollisionChecker checker;
  checker.init(map, footprint, 18);

  TopologyPlanner topo;
  topo.init(map, checker, 800, 20, 14, footprint.inscribedRadius());
  const SE2State start(1.06, 7.55, -1.57);
  const SE2State goal(8.97, 3.63, 1.52);
  const auto t_topo_begin = Clock::now();
  topo.buildRoadmap(start.position(), goal.position());
  auto paths = topo.searchPaths();
  topo.shortenPaths(paths);
  const auto t_topo_end = Clock::now();
  if (paths.empty()) {
    std::cerr << "[test] FAIL: expected at least one fixed-case topo path\n";
    return 1;
  }

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

  const size_t path_idx = 0;
  const auto t_seq_begin = Clock::now();
  auto segments = se2gen.generate(paths[path_idx], start, goal);
  const auto t_seq_end = Clock::now();
  if (segments.empty()) {
    std::cerr << "[test] FAIL: fixed-case path[0] produced no segments\n";
    return 1;
  }
  size_t target_seg_idx = segments.size();
  double target_chain_clearance = kInf;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto& seg = segments[i];
    if (seg.risk != RiskLevel::HIGH || seg.waypoints.size() > 6) continue;
    const double chain_clearance = continuousChainClearance(seg.waypoints, svsdf, 0.05);
    if (target_seg_idx == segments.size() || chain_clearance < target_chain_clearance) {
      target_seg_idx = i;
      target_chain_clearance = chain_clearance;
    }
  }

  if (target_seg_idx == segments.size()) {
    std::cerr << "[test] FAIL: expected a small fixed-case HIGH segment to target\n";
    return 1;
  }

  const auto& seg = segments[target_seg_idx];
  const double chain_clearance = continuousChainClearance(seg.waypoints, svsdf, 0.05);
  const double seg_time = std::max(0.5, segmentLength(seg.waypoints) /
                                          std::max(0.10, params.max_vel));
  const bool verbose = false;
  std::cout << "[test] target_high_seg=" << target_seg_idx
            << " wps=" << seg.waypoints.size()
            << " chain_clearance=" << chain_clearance << std::endl;
  if (verbose) {
    for (size_t wi = 0; wi < seg.waypoints.size(); ++wi) {
      const double wp_clearance = svsdf.evaluate(seg.waypoints[wi]);
      Eigen::Vector2d grad = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;
      svsdf.gradient(seg.waypoints[wi], grad, grad_yaw);
      std::cout << "[test]   wp[" << wi << "]=(" << seg.waypoints[wi].x
                << "," << seg.waypoints[wi].y
                << "," << seg.waypoints[wi].yaw
                << ") clearance=" << wp_clearance
                << " grad=(" << grad.x() << "," << grad.y() << ")"
                << " grad_yaw=" << grad_yaw << std::endl;
    }
  }
  if (verbose && seg.waypoints.size() >= 2) {
    const Eigen::Vector2d chord =
        seg.waypoints.back().position() - seg.waypoints.front().position();
    if (chord.norm() > 1e-9) {
      const Eigen::Vector2d normal(-chord.y(), chord.x());
      std::cout << "[test]   chord=(" << chord.x() << "," << chord.y()
                << ") normal=(" << normal.normalized().x() << ","
                << normal.normalized().y() << ")" << std::endl;
      const Eigen::Vector2d n = normal.normalized();
      for (double scale : {0.05, 0.10, 0.20, 0.30, 0.40}) {
        auto shifted_bad = seg.waypoints;
        for (size_t k = 1; k < shifted_bad.size(); ++k) {
          shifted_bad[k].x += scale * n.x();
          shifted_bad[k].y += scale * n.y();
        }
        std::cout << "[test]   bad_block_shift scale=" << scale
                  << " clearance="
                  << continuousChainClearance(shifted_bad, svsdf, 0.05)
                  << std::endl;

        auto shifted_all = seg.waypoints;
        for (size_t k = 0; k < shifted_all.size(); ++k) {
          shifted_all[k].x += scale * n.x();
          shifted_all[k].y += scale * n.y();
        }
        std::cout << "[test]   all_shift scale=" << scale
                  << " clearance="
                  << continuousChainClearance(shifted_all, svsdf, 0.05)
                  << std::endl;

        auto shifted_bad_neg = seg.waypoints;
        for (size_t k = 1; k < shifted_bad_neg.size(); ++k) {
          shifted_bad_neg[k].x -= scale * n.x();
          shifted_bad_neg[k].y -= scale * n.y();
        }
        std::cout << "[test]   bad_block_shift_neg scale=" << scale
                  << " clearance="
                  << continuousChainClearance(shifted_bad_neg, svsdf, 0.05)
                  << std::endl;

        auto shifted_bad_left = seg.waypoints;
        for (size_t k = 1; k < shifted_bad_left.size(); ++k) {
          shifted_bad_left[k].x -= scale;
        }
        std::cout << "[test]   bad_block_shift_left scale=" << scale
                  << " clearance="
                  << continuousChainClearance(shifted_bad_left, svsdf, 0.05)
                  << " clearance_fine="
                  << continuousChainClearance(shifted_bad_left, svsdf, 0.025)
                  << std::endl;
      }
    }
  }
  const auto t_seg_begin = Clock::now();
  const OptimizerResult result =
      optimizer.optimizeSE2Detailed(seg.waypoints, seg_time);
  const auto t_seg_end = Clock::now();
  const double clearance =
      result.traj.empty() ? -kInf : svsdf.evaluateTrajectory(result.traj);
  std::cout << "[test] path=" << path_idx
            << " high_seg=" << target_seg_idx
            << " wps=" << seg.waypoints.size()
            << " chain_clearance=" << chain_clearance
            << " se2_success=" << (result.success ? 1 : 0)
            << " se2_clearance=" << clearance
            << " elapsed_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   t_seg_end - t_seg_begin)
                   .count()
            << " source_mode=" << static_cast<int>(result.source_info.source_mode)
            << "\n";
  const bool target_ok = result.success && !result.traj.empty();

  if (!target_ok) {
    std::cerr << "[test] FAIL: targeted fixed-case HIGH segment still produces empty SE2 trajectory\n";
    return 1;
  }

  const auto t1 = Clock::now();
  const auto topo_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      t_topo_end - t_topo_begin).count();
  const auto seq_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      t_seq_end - t_seq_begin).count();
  const auto total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "[test] timing topo_ms="
            << topo_ms
            << " seq_ms="
            << seq_ms
            << " total_ms="
            << total_ms
            << "\n";

  if (total_ms > 12000) {
    std::cerr << "[test] FAIL: fixed-case targeted optimization is too slow (total_ms="
              << total_ms << " > 12000)\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
