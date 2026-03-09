#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/se2_sequence_generator.h>
#include <esv_planner/svsdf_evaluator.h>

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

double stateClearance(const SE2State& st, const SvsdfEvaluator& svsdf) {
  return svsdf.evaluate(st);
}

double transitionLinearClearance(const SE2State& a,
                                 const SE2State& b,
                                 const SvsdfEvaluator& svsdf,
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
                              const SvsdfEvaluator& svsdf) {
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
                                const SvsdfEvaluator& svsdf) {
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
  SvsdfEvaluator svsdf;
  svsdf.init(map, footprint);

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
  planner.shortenPaths(topo_paths);
  if (topo_paths.empty()) {
    std::cerr << "[test] FAIL: expected topology shortcut to produce a path\n";
    return 1;
  }
  path = topo_paths.front();

  std::vector<MotionSegment> segments = generator.generate(
      path, SE2State(0.70, 1.40, 0.0), SE2State(4.40, 1.40, 0.0));

  int high_count = 0;
  int total_high_waypoints = 0;
  int max_high_size = 0;
  bool has_replacement_segment = false;
  for (const auto& seg : segments) {
    if (seg.risk == RiskLevel::HIGH) {
      ++high_count;
      total_high_waypoints += static_cast<int>(seg.waypoints.size());
      max_high_size = std::max(max_high_size,
                               static_cast<int>(seg.waypoints.size()));
    }
    if (seg.waypoints.size() >= 4) {
      has_replacement_segment = true;
    }
  }

  std::cout << "[test] segments=" << segments.size()
            << " high_count=" << high_count
            << " total_high_waypoints=" << total_high_waypoints
            << " max_high_size=" << max_high_size
            << " replacement_like=" << has_replacement_segment << "\n";
  for (size_t i = 0; i < segments.size(); ++i) {
    const double chain_clearance =
        segmentLinearClearance(segments[i].waypoints, svsdf);
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
    }
  }

  if (!has_replacement_segment) {
    std::cerr << "[test] FAIL: expected SegAdjust-like expansion of a recoverable segment\n";
    return 1;
  }

  if (max_high_size > 4) {
    std::cerr << "[test] FAIL: expected HIGH windows to stay local\n";
    return 1;
  }

  if (high_count > 5 || total_high_waypoints > 16) {
    std::cerr << "[test] FAIL: expected recoverable synthetic case to avoid large HIGH expansion\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
