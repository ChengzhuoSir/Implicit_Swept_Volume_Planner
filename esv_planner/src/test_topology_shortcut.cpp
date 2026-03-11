#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/collision_checker.h>
#include <esv_planner/topology_planner.h>
#include <esv_planner/continuous_svsdf_evaluator.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

using namespace esv_planner;

namespace {

void dumpWaypoints(const std::vector<TopoWaypoint>& waypoints) {
  for (size_t i = 0; i < waypoints.size(); ++i) {
    std::cerr << "[test] wp[" << i << "]=(" << waypoints[i].pos.x()
              << "," << waypoints[i].pos.y() << ") yaw="
              << (waypoints[i].has_yaw ? waypoints[i].yaw : 999.0) << "\n";
  }
}

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

bool waypointStatesFree(const std::vector<TopoWaypoint>& waypoints,
                        const ContinuousSvsdfEvaluator& evaluator,
                        std::string* failure) {
  if (waypoints.size() < 2) return false;
  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    Eigen::Vector2d delta = waypoints[i].pos - waypoints[i - 1].pos;
    double yaw = waypoints[i].has_yaw ? waypoints[i].yaw : std::atan2(delta.y(), delta.x());
    if (evaluator.evaluate(SE2State(waypoints[i].pos.x(), waypoints[i].pos.y(), yaw)) < 0.0) {
      if (failure != nullptr) {
        std::ostringstream oss;
        oss << "waypoint=" << i
            << " pos=(" << waypoints[i].pos.x() << "," << waypoints[i].pos.y() << ")"
            << " yaw=" << yaw;
        *failure = oss.str();
      }
      return false;
    }
  }
  return true;
}

bool segmentGeometryFree(const std::vector<TopoWaypoint>& waypoints,
                         const ContinuousSvsdfEvaluator& evaluator,
                         double sample_step,
                         std::string* failure) {
  if (waypoints.size() < 2) return false;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const auto& a = waypoints[i - 1];
    const auto& b = waypoints[i];
    const Eigen::Vector2d delta = b.pos - a.pos;
    const double dist = delta.norm();
    const int n_steps =
        std::max(2, static_cast<int>(std::ceil(dist / sample_step)) + 1);
    const double yaw_a = a.has_yaw ? a.yaw : std::atan2(delta.y(), delta.x());
    const double yaw_b = b.has_yaw ? b.yaw : yaw_a;
    for (int s = 1; s + 1 < n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps - 1);
      SE2State st;
      st.x = a.pos.x() + t * delta.x();
      st.y = a.pos.y() + t * delta.y();
      st.yaw = normalizeAngle(yaw_a + normalizeAngle(yaw_b - yaw_a) * t);
      const double clearance = evaluator.evaluate(st);
      if (clearance < 0.0) {
        if (failure != nullptr) {
          std::ostringstream oss;
          oss << "segment=" << (i - 1) << "->" << i
              << " sample_t=" << t
              << " pos=(" << st.x << "," << st.y << ")"
              << " clearance=" << clearance;
          *failure = oss.str();
        }
        return false;
      }
    }
  }
  return true;
}

size_t localObstacleCount(const GridMap& map,
                          const Eigen::Vector2d& center,
                          double radius) {
  auto lo = map.worldToGrid(center.x() - radius, center.y() - radius);
  auto hi = map.worldToGrid(center.x() + radius, center.y() + radius);
  size_t count = 0;
  const auto& occ = map.inflated();
  const double r2 = radius * radius;
  for (int gy = std::max(0, lo.y); gy <= std::min(map.height() - 1, hi.y); ++gy) {
    for (int gx = std::max(0, lo.x); gx <= std::min(map.width() - 1, hi.x); ++gx) {
      const int idx = gy * map.width() + gx;
      if (occ[idx] <= 50 && occ[idx] >= 0) continue;
      if ((map.gridToWorld(gx, gy) - center).squaredNorm() <= r2) {
        ++count;
      }
    }
  }
  return count;
}

double waypointChainClearance(const std::vector<TopoWaypoint>& waypoints,
                              const ContinuousSvsdfEvaluator& evaluator) {
  if (waypoints.empty()) return -kInf;
  double min_clearance = kInf;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const double yaw =
        waypoints[i].has_yaw ? waypoints[i].yaw :
        ((i > 0) ? std::atan2((waypoints[i].pos - waypoints[i - 1].pos).y(),
                              (waypoints[i].pos - waypoints[i - 1].pos).x())
                 : 0.0);
    min_clearance = std::min(
        min_clearance,
        evaluator.evaluate(SE2State(waypoints[i].pos.x(), waypoints[i].pos.y(), yaw)));
  }
  return min_clearance;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_topology_shortcut", ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeSyntheticMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));
  map.inflateByRadius(0.1);

  FootprintModel footprint = makeTFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);
  ContinuousSvsdfEvaluator evaluator;
  evaluator.init(map, footprint);

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

  const size_t original_points = path.waypoints.size();
  const double original_length = path.length;
  const double original_state_clearance = waypointChainClearance(path.waypoints, evaluator);
  std::vector<TopoPath> paths{path};
  planner.shortenPaths(paths);

  if (paths.empty()) {
    std::cerr << "[test] FAIL: expected one shortened path\n";
    std::cerr << "[test] original_chain_clearance="
              << waypointChainClearance(path.waypoints, evaluator) << "\n";
    dumpWaypoints(path.waypoints);
    for (size_t i = 1; i + 1 < path.waypoints.size(); ++i) {
      TopoWaypoint repaired = path.waypoints[i];
      repaired.yaw = std::atan2((path.waypoints[i].pos - path.waypoints[i - 1].pos).y(),
                                (path.waypoints[i].pos - path.waypoints[i - 1].pos).x());
      repaired.has_yaw = true;
      const bool ok = planner.repairWaypointBodyFrame(repaired, footprint.inscribedRadius());
      const double clearance = evaluator.evaluate(
          SE2State(repaired.pos.x(), repaired.pos.y(), repaired.yaw));
      std::cerr << "[test] repaired_wp[" << i << "] ok=" << ok
                << " pos=(" << repaired.pos.x() << "," << repaired.pos.y() << ")"
                << " yaw=" << repaired.yaw
                << " clearance=" << clearance << "\n";
    }
    return 1;
  }

  const TopoPath& shortened = paths.front();
  std::cout << "[test] original_points=" << original_points
            << " shortened_points=" << shortened.waypoints.size()
            << " length=" << shortened.length
            << " original_state_clearance=" << original_state_clearance
            << " shortened_state_clearance="
            << waypointChainClearance(shortened.waypoints, evaluator) << "\n";

  if (shortened.waypoints.size() > 1 && !shortened.waypoints[1].has_yaw) {
    std::cerr << "[test] FAIL: expected shortened waypoint to retain yaw metadata\n";
    return 1;
  }

  const double shortened_state_clearance =
      waypointChainClearance(shortened.waypoints, evaluator);
  if (shortened_state_clearance + 1e-6 < original_state_clearance) {
    std::cerr << "[test] FAIL: expected shortcut to avoid worsening chain clearance\n";
    dumpWaypoints(shortened.waypoints);
    return 1;
  }

  if (shortened.length > original_length + 1e-3) {
    std::cerr << "[test] FAIL: expected shortcut to avoid increasing reference path length\n";
    dumpWaypoints(shortened.waypoints);
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
