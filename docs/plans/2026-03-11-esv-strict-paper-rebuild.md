# ESV Strict Paper Rebuild Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Rebuild `esv_planner` so the main online planner follows the paper's geometry, continuous-feasibility, and optimization structure instead of the current approximation-driven pipeline.

**Architecture:** First replace the runtime geometry representation and body-frame front-end, then unify continuous-feasibility semantics across topology, sequence, optimizer, and final validation, then rebuild the `SE2` and `R2` optimizers around continuous variables. Keep the current ROS shell, but remove non-paper online branches from the accepted-path logic as the rebuilt core becomes stable.

**Tech Stack:** ROS Noetic, C++, OccupancyGrid input, `libigl`, continuous optimizer backend (`LBFGS++` or `Ceres`), catkin, focused C++ regression tests.

---

### Task 1: Freeze The Rebuild Baseline

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Create: `docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline.md`

**Step 1: Write the failing or reporting baseline checks**
- Ensure the fixed case reports:
  - `best_topo_clearance`
  - `best_sequence_chain_clearance`
  - `best_segment_optimized_clearance`
  - `best_stitched_clearance`
  - `best_final_accepted_clearance`
- Ensure the secondary case `(0.72, 5.31, 0.29) -> (9.14, 5.74, 1.64)` can be replayed online and logged.
- Ensure `test_esv_pipeline_maze` reports accepted candidate counts and accepted `HIGH` counts.

**Step 2: Run baseline checks**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze esv_planner_node
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```
Expected:
- Current baseline recorded, even if `maze` still fails.

**Step 3: Save the baseline summary**
- Record the current metrics in `docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline.md`.

**Step 4: Commit**
```bash
git add docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline.md esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/src/test_esv_pipeline_maze.cpp esv_planner/src/esv_planner_node.cpp
git commit -m "baseline: record strict paper rebuild metrics"
```

### Task 2: Add The Geometry Map Layer

**Files:**
- Create: `esv_planner/include/esv_planner/geometry_map.h`
- Create: `esv_planner/src/geometry_map.cpp`
- Modify: `esv_planner/include/esv_planner/grid_map.h`
- Modify: `esv_planner/src/grid_map.cpp`
- Test: `esv_planner/src/test_geometry_map.cpp`

**Step 1: Write the failing test**
- Add a test that loads the maze map and expects:
  - obstacle boundary extraction is non-empty
  - nearest obstacle query returns stable results
  - local obstacle neighborhood query is bounded

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_geometry_map
./devel/lib/esv_planner/test_geometry_map
```
Expected:
- FAIL because `GeometryMap` does not exist yet.

**Step 3: Write minimal implementation**
- Implement contour extraction from occupancy grid
- Build segment/polygon storage and local neighborhood query
- Keep API small: boundary extraction, nearest obstacle query, local obstacle set query

**Step 4: Run test to verify it passes**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/geometry_map.h esv_planner/src/geometry_map.cpp esv_planner/include/esv_planner/grid_map.h esv_planner/src/grid_map.cpp esv_planner/src/test_geometry_map.cpp
git commit -m "feat: add geometry map layer for strict paper rebuild"
```

### Task 3: Rebuild The Body-Frame Front-End

**Files:**
- Create: `esv_planner/include/esv_planner/body_frame_sdf.h`
- Create: `esv_planner/src/body_frame_sdf.cpp`
- Modify: `esv_planner/include/esv_planner/topology_planner.h`
- Modify: `esv_planner/src/topology_planner.cpp`
- Modify: `esv_planner/src/footprint_model.cpp`
- Test: `esv_planner/src/test_body_frame_sdf.cpp`
- Test: `esv_planner/src/test_topology_body_push.cpp`

**Step 1: Write the failing tests**
- `test_body_frame_sdf`:
  - signed distance sign is correct for inside/outside robot-frame points
  - gradient direction is stable
- `test_topology_body_push`:
  - obstruction push raises local clearance on the maze map

**Step 2: Run tests to verify they fail**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_body_frame_sdf test_topology_body_push
./devel/lib/esv_planner/test_body_frame_sdf
./devel/lib/esv_planner/test_topology_body_push
```
Expected:
- FAIL before the module is fully wired.

**Step 3: Write minimal implementation**
- Add body-frame SDF module backed by `libigl`
- Replace `pushPointFromObstacle()` and shortcut obstruction repair with body-frame geometry queries
- Preserve orientation-aware topology waypoints

**Step 4: Run tests to verify they pass**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/body_frame_sdf.h esv_planner/src/body_frame_sdf.cpp esv_planner/include/esv_planner/topology_planner.h esv_planner/src/topology_planner.cpp esv_planner/src/footprint_model.cpp esv_planner/src/test_body_frame_sdf.cpp esv_planner/src/test_topology_body_push.cpp
git commit -m "feat: rebuild topology push-away around body-frame sdf"
```

### Task 4: Unify Continuous Feasibility Semantics

**Files:**
- Create: `esv_planner/include/esv_planner/continuous_feasibility.h`
- Create: `esv_planner/src/continuous_feasibility.cpp`
- Modify: `esv_planner/include/esv_planner/se2_sequence_generator.h`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Test: `esv_planner/src/test_motion_sequence.cpp`
- Test: `esv_planner/src/test_low_high_semantics.cpp`

**Step 1: Write the failing tests**
- `test_motion_sequence` should assert that `LOW` segments satisfy the unified continuous margin rule.
- `test_low_high_semantics` should assert that a sub-margin chain becomes `HIGH` under the same rule used later by final validation.

**Step 2: Run tests to verify they fail**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_motion_sequence test_low_high_semantics
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_low_high_semantics
```
Expected:
- FAIL before the new interface is fully used.

**Step 3: Write minimal implementation**
- Add unified feasibility interface
- Make `SafeYaw`, `SegAdjust`, and `LOW/HIGH` depend on that interface only
- Remove ad hoc local checks that do not match final acceptance semantics

**Step 4: Run tests to verify they pass**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/continuous_feasibility.h esv_planner/src/continuous_feasibility.cpp esv_planner/include/esv_planner/se2_sequence_generator.h esv_planner/src/se2_sequence_generator.cpp esv_planner/src/test_motion_sequence.cpp esv_planner/src/test_low_high_semantics.cpp
git commit -m "refactor: align se2 sequence semantics with unified continuous feasibility"
```

### Task 5: Replace The Sampled Collision Evaluator

**Files:**
- Create: `esv_planner/include/esv_planner/continuous_collision_evaluator.h`
- Create: `esv_planner/include/esv_planner/continuous_svsdf_evaluator.h`
- Create: `esv_planner/src/continuous_svsdf_evaluator.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: `esv_planner/src/test_continuous_svsdf_evaluator.cpp`

**Step 1: Write the failing test**
- `test_continuous_svsdf_evaluator` should show at least one case where old sampled clearance misses a collision but the bounded continuous evaluator detects it.

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_continuous_svsdf_evaluator
./devel/lib/esv_planner/test_continuous_svsdf_evaluator
```
Expected:
- FAIL before the new evaluator is connected.

**Step 3: Write minimal implementation**
- Add continuous collision evaluator interface
- Add bounded continuous evaluator using local geometry neighborhood queries and bounded interval subdivision
- Switch optimizer and final validation to the new evaluator

**Step 4: Run test to verify it passes**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/continuous_collision_evaluator.h esv_planner/include/esv_planner/continuous_svsdf_evaluator.h esv_planner/src/continuous_svsdf_evaluator.cpp esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/trajectory_optimizer.cpp esv_planner/src/esv_planner_node.cpp esv_planner/src/test_continuous_svsdf_evaluator.cpp
git commit -m "feat: replace sampled evaluator with bounded continuous collision kernel"
```

### Task 6: Rebuild The SE2 Optimizer Around Continuous Variables

**Files:**
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: `esv_planner/src/test_optimizer_se2_fixed_high_segments.cpp`
- Test: `esv_planner/src/test_optimizer_se2_fixed_large_high_segment.cpp`
- Test: `esv_planner/src/test_optimizer_se2_maze_t_high_segment.cpp`

**Step 1: Write or tighten the failing tests**
- Small `HIGH` window must produce non-empty `SE2` continuous output with `min_svsdf >= 0.1`
- Medium `HIGH` window must do the same
- `maze/T` target `HIGH` segment must no longer stop at `0.05`

**Step 2: Run tests to verify they fail under the old path**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_se2_fixed_high_segments test_optimizer_se2_fixed_large_high_segment test_optimizer_se2_maze_t_high_segment
./devel/lib/esv_planner/test_optimizer_se2_fixed_high_segments
./devel/lib/esv_planner/test_optimizer_se2_fixed_large_high_segment
./devel/lib/esv_planner/test_optimizer_se2_maze_t_high_segment
```
Expected:
- At least one target `HIGH` regression fails before the rebuild is complete.

**Step 3: Write minimal implementation**
- Move `SE2` away from waypoint-first acceptance and toward continuous-variable candidate generation and selection
- Retain only debug guards, not accepted-path dependence
- Preserve all candidate evaluations that hit the safety target; do not discard them due to overly strict shape gates

**Step 4: Run tests to verify they pass**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/trajectory_optimizer.cpp esv_planner/src/test_optimizer_se2_fixed_high_segments.cpp esv_planner/src/test_optimizer_se2_fixed_large_high_segment.cpp esv_planner/src/test_optimizer_se2_maze_t_high_segment.cpp
git commit -m "refactor: rebuild se2 optimizer toward continuous-variable path"
```

### Task 7: Rebuild The R2 Optimizer And Reduce Stitch

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: `esv_planner/src/test_stitch_seam_connector.cpp`
- Test: `esv_planner/src/test_svsdf_phase_consistency.cpp`
- Test: `esv_planner/src/test_esv_pipeline_maze.cpp`

**Step 1: Write the failing tests**
- `LOW` segments should not fail because the `R2` path is underpowered
- `stitch()` must not degrade segment-safe paths below `0.1`
- `maze/T` and `maze/L` should recover accepted candidates under the new gate

**Step 2: Run tests to verify they fail**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_stitch_seam_connector test_svsdf_phase_consistency test_esv_pipeline_maze
./devel/lib/esv_planner/test_stitch_seam_connector
./devel/lib/esv_planner/test_svsdf_phase_consistency
./devel/lib/esv_planner/test_esv_pipeline_maze
```
Expected:
- `maze` still fails before the rebuild is complete.

**Step 3: Write minimal implementation**
- Rebuild `R2` around the same continuous optimization structure
- Reduce `stitch()` to continuity-preserving connection only
- Keep the online acceptance gate at `>= 0.1`

**Step 4: Run tests to verify they pass**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/src/trajectory_optimizer.cpp esv_planner/src/test_stitch_seam_connector.cpp esv_planner/src/test_svsdf_phase_consistency.cpp esv_planner/src/test_esv_pipeline_maze.cpp
git commit -m "refactor: rebuild r2 path and reduce stitch to continuity connector"
```

### Task 8: Remove Non-Paper Online Branches

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_planner.cpp`

**Step 1: Write the failing checks**
- The final online planner must not publish a non-ESV fallback path
- Accepted online trajectories must satisfy `min_svsdf >= 0.1`

**Step 2: Run tests to verify the old branch still exists**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_planner esv_planner_node
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_planner
```
Expected:
- If old fallback branches remain, targeted assertions should fail.

**Step 3: Write minimal implementation**
- Remove online accepted-path dependence on non-paper branches
- Keep only ESV-produced trajectories in final publication

**Step 4: Run tests to verify they pass**
Run the same commands.
Expected:
- PASS

**Step 5: Commit**
```bash
git add esv_planner/src/esv_planner_node.cpp esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/src/test_planner.cpp
git commit -m "refactor: remove non-paper online fallback branches"
```

### Task 9: Run Full Validation And Record Results

**Files:**
- Modify: `docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline.md`
- Modify: `esv_planner/src/benchmark_planner.cpp`

**Step 1: Run the final validation set**
Run:
```bash
./devel/lib/esv_planner/test_continuous_svsdf_evaluator
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
timeout 30s roslaunch esv_planner demo.launch use_map_server:=true
```
Expected:
- fixed case passes
- secondary online case can be reproduced and evaluated
- maze regressions are back within acceptance thresholds

**Step 2: Update the baseline/results document**
- Record before/after metrics and unresolved risks.

**Step 3: Commit**
```bash
git add docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline.md esv_planner/src/benchmark_planner.cpp
git commit -m "docs: record strict paper rebuild validation results"
```
