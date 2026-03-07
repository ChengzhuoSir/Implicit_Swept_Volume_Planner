#include <esv_planner/planner_trigger_state.h>

#include <iostream>

using namespace esv_planner;

int main() {
  PlannerTriggerState state;
  state.setDefaultStartGoalAvailable();

  if (!state.onMapReceived()) {
    std::cerr << "[test] FAIL: defaults should plan once the map arrives\n";
    return 1;
  }

  if (state.onStartUpdated()) {
    std::cerr << "[test] FAIL: updating start should wait for goal selection\n";
    return 1;
  }

  if (!state.onGoalUpdated()) {
    std::cerr << "[test] FAIL: updating goal should trigger planning with current start\n";
    return 1;
  }

  std::cout << "[test] PASS\n";
  return 0;
}
