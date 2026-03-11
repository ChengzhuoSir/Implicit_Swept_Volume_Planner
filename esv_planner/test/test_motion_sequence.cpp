#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>
#include <esv_planner/continuous_svsdf_evaluator.h>
#include <esv_planner/continuous_feasibility.h>

#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeSyntheticMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 100;
  map->info.height = 100;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);

  auto fillRect = [&](int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        map->data[static_cast<size_t>(y) * map->info.width + x] = 100;
      }
    }
  };

  fillRect(38, 20, 60, 74);
  fillRect(20, 20, 38, 38);
  return map;
}

FootprintModel makeTFootprint() {
  FootprintModel footprint;
  footprint.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  footprint.setInscribedRadius(0.1);
  return footprint;
}

double stateClearance(const SE2State& st, const ContinuousSvsdfEvaluator& svsdf) {
  return svsdf.evaluate(st);
}

double transitionLinearClearance(const SE2State& a,
                                 const SE2State& b,
                                 const ContinuousSvsdfEvaluator& svsdf,
                                 double sample_step) {
  const double len = (b.position() - a.position()).norm();
  const int n_steps = std::max(
      1, static_cast<int>(std::ceil(len / std::max(sample_step, 1e-3))));
  double min_clearance = kInf;
  for (int s = 0; s <= n_steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(n_steps);
    SE2State st;
    st.x = a.x + (b.x - a.x) * t;
    st.y = a.y + (b.y - a.y) * t;
    st.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * t);
    min_clearance = std::min(min_clearance, svsdf.evaluate(st));
  }
  return min_clearance;
}

double segmentLinearClearance(const std::vector<SE2State>& waypoints,
                              const ContinuousSvsdfEvaluator& svsdf) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    min_clearance = std::min(min_clearance, stateClearance(waypoints[i], svsdf));
  }
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    min_clearance = std::min(
        min_clearance, transitionLinearClearance(waypoints[i], waypoints[i + 1], svsdf, 0.05));
  }
  return min_clearance;
}

void printFirstUnsafeLinearPair(const std::vector<SE2State>& waypoints,
                                const ContinuousSvsdfEvaluator& svsdf) {
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double clearance =
        transitionLinearClearance(waypoints[i], waypoints[i + 1], svsdf, 0.05);
    if (clearance < 0.0) {
      std::cout << "[test]   first_unsafe_pair=" << i
                << " from=(" << waypoints[i].x << "," << waypoints[i].y
                << "," << waypoints[i].yaw << ")"
                << " to=(" << waypoints[i + 1].x << "," << waypoints[i + 1].y
                << "," << waypoints[i + 1].yaw << ")"
                << " clearance=" << clearance << "\n";
      return;
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_motion_sequence", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSyntheticMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);
  ContinuousSvsdfEvaluator svsdf;
  svsdf.init(map, footprint);
  GridContinuousFeasibilityChecker feasibility(map, checker, 0.10);

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.10, 5);

  TopologyPlanner planner;
  planner.init(map, checker, 100, 8, 4, footprint.inscribedRadius());

  TopoPath path;
  path.waypoints = {
      TopoWaypoint(0.70, 1.40),
      TopoWaypoint(1.30, 1.40),
      TopoWaypoint(1.30, 4.30),
      TopoWaypoint(4.40, 4.30),
      TopoWaypoint(4.40, 1.40)};
  path.computeLength();

  std::vector<TopoPath> topo_paths{path};
  const auto t0 = std::chrono::steady_clock::now();
  planner.shortenPaths(topo_paths);
  const auto t1 = std::chrono::steady_clock::now();
  std::cout << "[test] shorten_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
            << "\n";
  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected topology shortcut to produce a path\n";
    return 1;
  }
  path = topo_paths.front();

  const auto t2 = std::chrono::steady_clock::now();
  std::vector<MotionSegment> segments = generator.generate(
      path, SE2State(0.70, 1.40, 0.0), SE2State(4.40, 1.40, 0.0));
  const auto t3 = std::chrono::steady_clock::now();
  std::cout << "[test] generate_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
            << "\n";

  int high_count = 0;
  int small_high_count = 0;
  int total_high_waypoints = 0;
  int max_high_size = 0;
  int max_consecutive_high = 0;
  int consecutive_high = 0;
  bool has_replacement_segment = false;
  bool saw_safe_low_segment = false;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      ++high_count;
      ++consecutive_high;
      total_high_waypoints += static_cast<int>(seg.waypoints.size());
      max_high_size = std::max(max_high_size,
                               static_cast<int>(seg.waypoints.size()));
      if (static_cast<int>(seg.waypoints.size()) <= 4) {
        ++small_high_count;
      }
      max_consecutive_high = std::max(max_consecutive_high, consecutive_high);
    } else {
      consecutive_high = 0;
      saw_safe_low_segment = true;
    }
    if (seg.waypoints.size() >= 4) {
      has_replacement_segment = true;
    }
  }

  std::cout << "[test] segments=" << segments.size()
            << " high_count=" << high_count
            << " small_high_count=" << small_high_count
            << " total_high_waypoints=" << total_high_waypoints
            << " max_high_size=" << max_high_size
            << " max_consecutive_high=" << max_consecutive_high
            << " replacement_like=" << has_replacement_segment << "\n";
  for (size_t i = 0; i < segments.size(); ++i) {
    const double chain_clearance =
        feasibility.segmentClearance(segments[i].waypoints);
    std::cout << "[test] seg[" << i << "] risk="
              << (segments[i].risk == RiskLevel::HIGH ? "HIGH" : "LOW")
              << " size=" << segments[i].waypoints.size()
              << " chain_clearance=" << chain_clearance << "\n";
    if (segments[i].risk == RiskLevel::HIGH) {
      const size_t limit = std::min<size_t>(segments[i].waypoints.size(), 6);
      for (size_t j = 0; j < limit; ++j) {
        const auto& st = segments[i].waypoints[j];
        std::cout << "[test]   high[" << j << "]=(" << st.x << "," << st.y
                  << "," << st.yaw << ")\n";
      }
    } else if (chain_clearance < feasibility.requiredClearance()) {
      std::cerr << "[test] FAIL: expected LOW segments to satisfy default safety margin\n";
      printFirstUnsafeLinearPair(segments[i].waypoints, svsdf);
      return 1;
    }
  }

  if (!has_replacement_segment) {
    std::cerr << "[test] FAIL: expected SegAdjust-like expansion of a recoverable segment\n";
    return 1;
  }

  if (!saw_safe_low_segment) {
    std::cerr << "[test] FAIL: expected synthetic case to retain at least one safe LOW segment\n";
    return 1;
  }

  if (high_count > 4) {
    std::cerr << "[test] FAIL: expected bounded continuous generator to avoid HIGH fragmentation\n";
    return 1;
  }

  if (small_high_count > 1) {
    std::cerr << "[test] FAIL: expected generator to avoid many tiny HIGH windows\n";
    return 1;
  }

  if (max_consecutive_high > 3) {
    std::cerr << "[test] FAIL: expected HIGH chains to stay bounded\n";
    return 1;
  }

  if (max_high_size > 12 || total_high_waypoints > 28) {
    std::cerr << "[test] FAIL: expected recoverable synthetic case to avoid pathological HIGH expansion\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
