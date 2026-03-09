#pragma once

namespace esv_planner {

class PlannerTriggerState {
public:
  // Default start/goal values are placeholders for visualization/config only.
  // They must not arm planning before the user explicitly selects both poses.
  void setDefaultStartGoalAvailable() {
    has_start_ = false;
    has_goal_ = false;
  }

  bool onMapReceived() {
    has_map_ = true;
    return canPlan();
  }

  bool onStartUpdated() {
    has_start_ = true;
    return false;
  }

  bool onGoalUpdated() {
    has_goal_ = true;
    return canPlan();
  }

  bool canPlan() const {
    return has_map_ && has_start_ && has_goal_;
  }

private:
  bool has_map_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
};

}  // namespace esv_planner
