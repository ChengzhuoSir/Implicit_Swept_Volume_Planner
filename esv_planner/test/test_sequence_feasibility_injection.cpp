#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <esv_planner/collision_checker.h>
#include <esv_planner/continuous_feasibility.h>
#include <esv_planner/footprint_model.h>
#include <esv_planner/grid_map.h>
#include <esv_planner/se2_sequence_generator.h>

#include <iostream>
#include <memory>

using namespace esv_planner;

namespace {

nav_msgs::OccupancyGrid::Ptr makeOpenMap() {
  auto map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->info.resolution = 0.05;
  map->info.width = 40;
  map->info.height = 40;
  map->info.origin.position.x = 0.0;
  map->info.origin.position.y = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<size_t>(map->info.width) * map->info.height, 0);
  return map;
}

FootprintModel makeBoxFootprint() {
  FootprintModel footprint;
  footprint.setPolygon({
      {0.2, 0.2}, {0.2, -0.2}, {-0.2, -0.2}, {-0.2, 0.2}});
  footprint.setInscribedRadius(0.1);
  return footprint;
}

class FakeFeasibilityChecker final : public ContinuousFeasibilityChecker {
public:
  double requiredClearance() const override { return 0.1; }

  double stateClearance(const SE2State& state) const override {
    (void)state;
    return 1.0;
  }

  double transitionClearance(const SE2State& from,
                             const SE2State& to) const override {
    (void)from;
    (void)to;
    return 1.0;
  }

  bool safeYaw(SE2State& state, double desired_yaw) const override {
    state.yaw = desired_yaw;
    return true;
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_sequence_feasibility_injection",
            ros::init_options::NoSigintHandler);
  ros::Time::init();

  GridMap map;
  auto occ = makeOpenMap();
  map.fromOccupancyGrid(boost::const_pointer_cast<const nav_msgs::OccupancyGrid>(occ));

  FootprintModel footprint = makeBoxFootprint();
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.2, 2,
                 std::unique_ptr<ContinuousFeasibilityChecker>(
                     new FakeFeasibilityChecker()));

  TopoPath path;
  path.waypoints = {
      TopoWaypoint(0.5, 0.5),
      TopoWaypoint(1.5, 0.5),
      TopoWaypoint(2.5, 0.5)};
  path.computeLength();

  const auto segments = generator.generate(
      path, SE2State(0.5, 0.5, 0.0), SE2State(2.5, 0.5, 0.0));

  std::cout << "[test] segments=" << segments.size() << "\n";
  if (segments.size() != 1) {
    std::cerr << "[test] FAIL: injected feasibility checker should keep open-path sequence as one segment\n";
    return 1;
  }
  if (segments.front().risk != RiskLevel::LOW) {
    std::cerr << "[test] FAIL: injected feasibility checker should make the only segment LOW\n";
    return 1;
  }
  for (const auto& st : segments.front().waypoints) {
    if (std::abs(st.yaw) > 1e-9) {
      std::cerr << "[test] FAIL: injected feasibility checker should control safeYaw assignment\n";
      return 1;
    }
  }

  std::cout << "[test] PASS\n";
  return 0;
}
