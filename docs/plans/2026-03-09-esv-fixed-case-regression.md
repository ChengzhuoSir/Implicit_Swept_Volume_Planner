# ESV Fixed-Case Regression Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the main-branch ESV pipeline publish a valid ESV trajectory for the fixed maze case `(0.77, 5.53, 0.07) -> (9.15, 3.04, 1.58)` instead of falling back to `Hybrid A*`, while preserving the strict collision gate.

**Architecture:** Add one dedicated regression harness for the fixed case, instrument the pipeline to report per-stage clearance, then fix the first failing layer with the smallest possible algorithm change. Keep the strict `min_svsdf >= 0.0` acceptance rule and re-run `maze + T/L` plus `demo.launch` after each targeted fix.

**Tech Stack:** ROS Noetic, catkin, C++, `nav_msgs/OccupancyGrid`, current `esv_planner` node/tests/optimizer code.

---

### Task 1: Add A Fixed-Case Regression Harness

**Files:**
- Create: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/CMakeLists.txt`
- Reference: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Reference: `esv_planner/src/test_planner.cpp`

**Step 1: Write the failing test**

Create `test_esv_fixed_case_regression.cpp` that:
- loads `esv_planner/maps/maze.pgm`
- uses the fixed start and goal
- runs the same pipeline stages as `ESVPlannerNode::tryPlan()`
- records:
  - number of topo paths
  - per-path segment counts
  - best `R2` segment clearance
  - best `SE2` segment clearance
  - stitched clearance
  - whether final selected output is ESV or fallback
- fails if no valid ESV trajectory exists

Test skeleton:

```cpp
int main(int argc, char** argv) {
  // init ROS time only
  // load maze map
  // start=(0.77, 5.53, 0.07), goal=(9.15, 3.04, 1.58)
  // run topology -> sequence -> optimize -> stitch
  // print per-stage clearance table
  // return 1 if final ESV trajectory is empty or min_svsdf < 0.0
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- `FAIL`
- diagnostic output showing the first layer where clearance becomes negative

**Step 3: Write minimal implementation**

No production code yet. Only finish the test harness until it fails for the intended reason.

**Step 4: Run test to verify it fails cleanly**

Run the same command again.

Expected:
- deterministic failure
- no missing-map or missing-build errors

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/CMakeLists.txt
git commit -m "test: add fixed-case ESV regression harness"
```

### Task 2: Add Per-Stage Clearance Diagnostics

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Reference: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Write the failing test**

Extend the new test to assert that diagnostics are available for each stage:
- `topo_sample_min_svsdf`
- `r2_min_svsdf`
- `se2_min_svsdf`
- `stitched_min_svsdf`

Example assertion:

```cpp
if (!report.has_topo || !report.has_stitch) {
  std::cerr << "[test] FAIL: missing stage diagnostics\n";
  return 1;
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- `FAIL` because the stage report structure/logging does not exist yet

**Step 3: Write minimal implementation**

Add minimal helper utilities to compute and return:
- sampled clearance for raw waypoint chains
- clearance after `optimizeR2()`
- clearance after `optimizeSE2()`
- clearance after `stitch()`

Do not change planner behavior yet. Only expose enough data to identify the first failing layer.

**Step 4: Run test to verify it passes**

Run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- test still fails on final trajectory validity
- diagnostics now print a stable failing stage

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/src/trajectory_optimizer.cpp esv_planner/include/esv_planner/trajectory_optimizer.h
git commit -m "test: add per-stage clearance diagnostics"
```

### Task 3: Fix The First Failing Layer Only

**Files:**
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Reference: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Write the failing test**

Based on Task 2 diagnostics, add one focused assertion for the first bad transition. Examples:
- raw topology path is safe but `optimizeR2()` makes it unsafe
- `optimizeSE2()` is safe but `stitch()` makes it unsafe
- single `LOW` segment should be promoted before `R2`

Example for a stitch regression:

```cpp
if (report.se2_min_svsdf >= 0.0 && report.stitched_min_svsdf < 0.0) {
  std::cerr << "[test] FAIL: stitch regressed a safe segment\n";
  return 1;
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- `FAIL` at the exact transition you are going to fix

**Step 3: Write minimal implementation**

Pick only one of these, depending on the failing layer identified in Task 2:
- promote tight `LOW` windows to `HIGH` before `R2`
- add a post-`R2` continuous-clearance guard that upgrades the segment to `SE2`
- make `stitch()` preserve safe single-segment or tight multi-segment corridors more conservatively

Do not combine multiple algorithm changes in the same commit.

**Step 4: Run test to verify it passes**

Run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- fixed case now returns a non-empty ESV trajectory
- `min_svsdf >= 0.0`

**Step 5: Commit**

Example:

```bash
git add esv_planner/src/se2_sequence_generator.cpp esv_planner/src/trajectory_optimizer.cpp esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "fix: preserve clearance for fixed ESV maze case"
```

### Task 4: Re-Run Main Planner Regressions

**Files:**
- Modify only if needed: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Modify only if needed: `esv_planner/src/test_planner.cpp`
- Reference: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Write the failing test**

If the fixed-case change breaks existing assumptions, update regression expectations so they match the strict gate and the corrected planner semantics.

Examples:
- `test_esv_pipeline_maze`
- `test_planner`

**Step 2: Run tests to verify failures**

Run:

```bash
./devel/lib/esv_planner/test_esv_pipeline_maze
./devel/lib/esv_planner/test_planner
```

Expected:
- identify whether regressions are real algorithm failures or outdated test expectations

**Step 3: Write minimal implementation**

Only patch the broken layer or stale expectation found above. Do not broaden scope.

**Step 4: Run tests to verify they pass**

Run:

```bash
./devel/lib/esv_planner/test_esv_pipeline_maze
./devel/lib/esv_planner/test_planner
./devel/lib/esv_planner/test_optimizer_select_best_clearance
./devel/lib/esv_planner/test_planner_trigger_state
```

Expected:
- all pass, or remaining failures are explicitly documented as known pre-existing optimizer gaps

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_pipeline_maze.cpp esv_planner/src/test_planner.cpp esv_planner/src/esv_planner_node.cpp
git commit -m "test: align planner regressions with strict ESV gate"
```

### Task 5: Validate In ROS Demo

**Files:**
- Reference: `esv_planner/launch/demo.launch`
- Reference: `esv_planner/config/planner.yaml`
- Reference: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Write the failing scenario script**

Use the fixed RViz case as the acceptance scenario:
- launch demo
- publish `2D Pose Estimate` equivalent start
- publish `2D Nav Goal` equivalent goal
- capture logs

**Step 2: Run scenario to verify current behavior**

Run:

```bash
source /home/chengzhuo/workspace/plan/devel/setup.zsh
roslaunch esv_planner demo.launch use_map_server:=true
```

Then publish:
- start `(0.77, 5.53, 0.07)`
- goal `(9.15, 3.04, 1.58)`

Expected before the final fix:
- `Falling back to Hybrid A* path.`

**Step 3: Write minimal implementation**

Only if needed, patch the node so logs clearly distinguish:
- `ESV success`
- `ESV rejected, using Hybrid A* fallback`

**Step 4: Run scenario to verify it passes**

Expected after the final algorithm fix:
- no fallback log for the fixed case
- final published result is ESV
- no negative-clearance diagnostics

**Step 5: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "feat: make fixed maze case succeed with ESV"
```

### Task 6: Final Verification And Handoff

**Files:**
- Review only: `docs/plans/2026-03-09-esv-fixed-case-regression-design.md`
- Review only: `docs/plans/2026-03-09-esv-fixed-case-regression.md`

**Step 1: Run the verification set**

Run:

```bash
catkin_make --pkg esv_planner --make-args \
  test_esv_fixed_case_regression \
  test_esv_pipeline_maze \
  test_planner \
  test_optimizer_select_best_clearance \
  test_planner_trigger_state \
  esv_planner_node
```

Then run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
./devel/lib/esv_planner/test_planner
./devel/lib/esv_planner/test_optimizer_select_best_clearance
./devel/lib/esv_planner/test_planner_trigger_state
```

**Step 2: Run final ROS validation**

Run `demo.launch` and replay the fixed case.

Expected:
- published ESV trajectory for the fixed case
- no fallback for that case
- strict collision gate preserved

**Step 3: Review git state**

Run:

```bash
git status --short
git log --oneline -10
```

Expected:
- clean working tree except for explicitly excluded user files
- one commit per verified task

**Step 4: Summarize residual risks**

Document any remaining limitations, especially:
- cases that still require `Hybrid A*` fallback
- known optimizer weaknesses
- benchmark gaps versus the paper

**Step 5: Commit documentation updates if needed**

```bash
git add docs/plans/2026-03-09-esv-fixed-case-regression-design.md docs/plans/2026-03-09-esv-fixed-case-regression.md
git commit -m "docs: finalize fixed-case ESV execution notes"
```
