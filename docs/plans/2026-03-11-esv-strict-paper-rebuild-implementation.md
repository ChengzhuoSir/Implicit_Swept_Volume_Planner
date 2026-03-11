# ESV Strict Paper Rebuild Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Rebuild `esv_planner` so the main online pipeline is structurally and numerically much closer to the paper, replacing the current engineering approximations in front-end geometry, continuous collision, and optimizer core.

**Architecture:** Keep the ROS/map/test shell, but replace the algorithmic core in ordered stages: geometry map, body-frame front-end, unified continuous feasibility, continuous evaluator, `SE2/R2` continuous optimization, then node cutover and performance cleanup. Each stage is gated by focused regressions, the fixed case, and `maze` regressions.

**Tech Stack:** ROS Noetic, C++, Eigen, `libigl` for body-frame geometry queries, `LBFGS++` (or equivalent header-only L-BFGS) for continuous optimization, catkin/catkin_make, existing test binaries in `esv_planner/src`.

---

### Task 1: Freeze Baseline And Add Acceptance Harnesses

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Create: `esv_planner/src/test_esv_second_case_regression.cpp`
- Modify: `esv_planner/CMakeLists.txt`
- Create: `docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline-v2.md`

**Step 1: Write the failing/recording tests**
- Add the second acceptance case `Start=(0.72, 5.31, 0.29) -> Goal=(9.14, 5.74, 1.64)`.
- Extend logging/metrics to capture:
  - `accepted_candidates`
  - `best_topo_clearance`
  - `best_sequence_chain_clearance`
  - `best_stitched_clearance`
  - `best_final_accepted_clearance`
  - total `HIGH` segments for accepted paths
  - runtime

**Step 2: Run tests to record baseline**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_second_case_regression test_esv_pipeline_maze
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_second_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```
Expected: current mixed green/red state is recorded, not fixed.

**Step 3: Save baseline note**
- Write current metrics and failure points into `docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline-v2.md`.

**Step 4: Commit**
```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/src/test_esv_pipeline_maze.cpp esv_planner/src/test_esv_second_case_regression.cpp esv_planner/CMakeLists.txt docs/plans/2026-03-11-esv-strict-paper-rebuild-baseline-v2.md docs/plans/2026-03-11-esv-strict-paper-rebuild-implementation.md
git commit -m "test: add strict paper rebuild baseline harnesses"
```

### Task 2: Build Geometry Map Layer

**Files:**
- Create: `esv_planner/include/esv_planner/geometry_map.h`
- Create: `esv_planner/src/geometry_map.cpp`
- Modify: `esv_planner/include/esv_planner/grid_map.h`
- Modify: `esv_planner/src/grid_map.cpp`
- Create: `esv_planner/src/test_geometry_map.cpp`
- Modify: `esv_planner/CMakeLists.txt`

**Step 1: Write the failing test**
- Test conversion from occupancy grid to:
  - boundary cell set
  - obstacle contour polylines
  - queryable local obstacle set around a pose
- Assert local obstacle query does not require scanning all occupied cells.

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_geometry_map
./devel/lib/esv_planner/test_geometry_map
```
Expected: FAIL because `GeometryMap` does not exist.

**Step 3: Write minimal implementation**
- Add `GeometryMap` that stores:
  - obstacle boundary points
  - contour segments/polyline chains
  - local query helpers
- Keep `GridMap` as input/ESDF owner, but expose `GeometryMap` for continuous geometry consumers.

**Step 4: Run test to verify it passes**
Run the same test binary and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/geometry_map.h esv_planner/src/geometry_map.cpp esv_planner/include/esv_planner/grid_map.h esv_planner/src/grid_map.cpp esv_planner/src/test_geometry_map.cpp esv_planner/CMakeLists.txt
git commit -m "feat: add geometry map layer for strict paper rebuild"
```

### Task 3: Replace Body-Frame Geometry With libigl-Backed Query Core

**Files:**
- Modify: `esv_planner/include/esv_planner/body_frame_sdf.h`
- Create: `esv_planner/src/body_frame_sdf.cpp`
- Modify: `esv_planner/include/esv_planner/footprint_model.h`
- Modify: `esv_planner/src/footprint_model.cpp`
- Create: `esv_planner/src/test_body_frame_sdf.cpp`
- Modify: `esv_planner/CMakeLists.txt`

**Step 1: Write the failing test**
- Verify signed distance, inside/outside, closest point, and gradient on representative footprint queries.
- Include non-convex footprint cases if supported by current robot shape assumptions.

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_body_frame_sdf
./devel/lib/esv_planner/test_body_frame_sdf
```
Expected: FAIL before new implementation is wired.

**Step 3: Write minimal implementation**
- Integrate `libigl`.
- Build a body-frame polygon/prism query core around generalized winding / signed-distance semantics.
- Replace the current header-only custom query as the main path.

**Step 4: Run test to verify it passes**
Run the same test and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/body_frame_sdf.h esv_planner/src/body_frame_sdf.cpp esv_planner/include/esv_planner/footprint_model.h esv_planner/src/footprint_model.cpp esv_planner/src/test_body_frame_sdf.cpp esv_planner/CMakeLists.txt
git commit -m "feat: integrate libigl-backed body-frame sdf"
```

### Task 4: Rebuild Topology Front-End Around Body-Frame Push-Away

**Files:**
- Modify: `esv_planner/src/topology_planner.cpp`
- Modify: `esv_planner/include/esv_planner/topology_planner.h`
- Modify: `esv_planner/include/esv_planner/common.h`
- Create: `esv_planner/src/test_topology_body_push.cpp`
- Modify: `esv_planner/src/test_topology_shortcut.cpp`

**Step 1: Write the failing test**
- Assert a colliding/near-colliding waypoint can be improved by body-frame push-away using local geometry map queries.
- Assert shortcut output improves raw topo path and is safe enough to hand to the sequence layer.

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_topology_body_push test_topology_shortcut
./devel/lib/esv_planner/test_topology_body_push
./devel/lib/esv_planner/test_topology_shortcut
```
Expected: FAIL or regress against the current shortcut logic.

**Step 3: Write minimal implementation**
- Replace `pushPointFromObstacle()` main path with body-frame gradient push-away using local geometry queries.
- Remove heavy translation/yaw brute search from the front-end path.
- Keep front-end responsibility limited to producing a cleaner reference chain, not solving every local window.

**Step 4: Run tests to verify they pass**
Run the two tests again and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/src/topology_planner.cpp esv_planner/include/esv_planner/topology_planner.h esv_planner/include/esv_planner/common.h esv_planner/src/test_topology_body_push.cpp esv_planner/src/test_topology_shortcut.cpp
git commit -m "fix: align topology planner with body-frame geometry"
```

### Task 5: Introduce Unified Continuous Feasibility Checker

**Files:**
- Create: `esv_planner/include/esv_planner/continuous_feasibility_checker.h`
- Modify: `esv_planner/include/esv_planner/continuous_feasibility.h`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/include/esv_planner/se2_sequence_generator.h`
- Create: `esv_planner/src/test_low_high_semantics.cpp`

**Step 1: Write the failing test**
- Add a regression that checks:
  - `LOW` means continuous-safe by the same rule used later in final validation
  - `HIGH` is produced when recursive repair fails under that same rule

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_low_high_semantics test_motion_sequence
./devel/lib/esv_planner/test_low_high_semantics
./devel/lib/esv_planner/test_motion_sequence
```
Expected: FAIL because current generator still relies on engineering compaction/expansion logic.

**Step 3: Write minimal implementation**
- Define unified continuous feasibility interface used by:
  - `safeYaw`
  - `transitionSafe`
  - `SegAdjust`
  - `LOW/HIGH` labeling
- Remove current compaction/expansion helpers from the main semantic path.

**Step 4: Run tests to verify they pass**
Run both tests and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/continuous_feasibility_checker.h esv_planner/include/esv_planner/continuous_feasibility.h esv_planner/include/esv_planner/se2_sequence_generator.h esv_planner/src/se2_sequence_generator.cpp esv_planner/src/test_low_high_semantics.cpp
git commit -m "refactor: align se2 sequence semantics with continuous feasibility"
```

### Task 6: Replace Sampled Collision Core With New Continuous Evaluator

**Files:**
- Modify: `esv_planner/include/esv_planner/continuous_collision_evaluator.h`
- Modify: `esv_planner/include/esv_planner/continuous_svsdf_evaluator.h`
- Modify: `esv_planner/src/continuous_svsdf_evaluator.cpp`
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Modify: `esv_planner/src/test_continuous_svsdf_evaluator.cpp`

**Step 1: Write the failing test**
- Add regressions for:
  - local obstacle candidate pruning
  - bounded interval refinement
  - consistency between state, segment, and trajectory evaluation

**Step 2: Run test to verify it fails**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_continuous_svsdf_evaluator
./devel/lib/esv_planner/test_continuous_svsdf_evaluator
```
Expected: FAIL or show mismatched semantics.

**Step 3: Write minimal implementation**
- Implement a bounded continuous evaluator over local geometry/ESDF queries.
- Keep complexity explicitly capped.
- Remove the old sampled evaluator from the final acceptance path.

**Step 4: Run test to verify it passes**
Run the evaluator test and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/continuous_collision_evaluator.h esv_planner/include/esv_planner/continuous_svsdf_evaluator.h esv_planner/src/continuous_svsdf_evaluator.cpp esv_planner/src/esv_planner_node.cpp esv_planner/src/test_continuous_svsdf_evaluator.cpp
git commit -m "feat: add bounded continuous collision evaluator"
```

### Task 7: Rebuild SE2 Optimizer Around Continuous Variables

**Files:**
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Create: `esv_planner/src/test_optimizer_continuous_se2.cpp`
- Modify: `esv_planner/src/test_optimizer_se2_fixed_high_segments.cpp`
- Modify: `esv_planner/src/test_optimizer_se2_maze_t_high_segment.cpp`

**Step 1: Write the failing tests**
- Require `SE2` optimizer to solve representative small and medium `HIGH` windows without empty trajectories and without guard-based fallback.

**Step 2: Run tests to verify they fail**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_continuous_se2 test_optimizer_se2_fixed_high_segments test_optimizer_se2_maze_t_high_segment
./devel/lib/esv_planner/test_optimizer_continuous_se2
./devel/lib/esv_planner/test_optimizer_se2_fixed_high_segments
./devel/lib/esv_planner/test_optimizer_se2_maze_t_high_segment
```
Expected: FAIL because optimizer is still waypoint/support-state first.

**Step 3: Write minimal implementation**
- Move `SE2` main path to continuous parameters + segment times.
- Use `LBFGS++` for the new objective.
- Keep current heuristic repairs only as temporary comparison/debug paths, not final accepted source.

**Step 4: Run tests to verify they pass**
Run all three tests and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/trajectory_optimizer.cpp esv_planner/src/test_optimizer_continuous_se2.cpp esv_planner/src/test_optimizer_se2_fixed_high_segments.cpp esv_planner/src/test_optimizer_se2_maze_t_high_segment.cpp
git commit -m "refactor: move se2 optimizer to continuous variables"
```

### Task 8: Rebuild R2 Optimizer And Reduce Stitch To Connector

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Create: `esv_planner/src/test_optimizer_continuous_r2.cpp`
- Create: `esv_planner/src/test_stitch_continuity.cpp`

**Step 1: Write the failing tests**
- Require `R2` to solve low-risk segments in the same continuous framework.
- Require `stitch()` to preserve segment-level safe geometry instead of degrading it.

**Step 2: Run tests to verify they fail**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_continuous_r2 test_stitch_continuity
./devel/lib/esv_planner/test_optimizer_continuous_r2
./devel/lib/esv_planner/test_stitch_continuity
```
Expected: FAIL under current `R2`/stitch behavior.

**Step 3: Write minimal implementation**
- Move `R2` into the continuous optimizer framework.
- Reduce `stitch()` to connection, continuity checks, and light retiming only.

**Step 4: Run tests to verify they pass**
Run both tests and ensure PASS.

**Step 5: Commit**
```bash
git add esv_planner/src/trajectory_optimizer.cpp esv_planner/src/test_optimizer_continuous_r2.cpp esv_planner/src/test_stitch_continuity.cpp
git commit -m "refactor: move r2 optimizer and stitch to continuous pipeline"
```

### Task 9: Cut Over Online Node And Restore Global Regressions

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Modify: `esv_planner/src/benchmark_planner.cpp`
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/test_esv_second_case_regression.cpp`
- Modify: `esv_planner/src/test_esv_pipeline_maze.cpp`

**Step 1: Write the failing/strict regression checks**
- Require all online accepted trajectories to satisfy `min_svsdf >= 0.1`.
- Require no `Hybrid A*` final publish fallback.
- Require fixed case and second case to be accepted by the rebuilt ESV chain.

**Step 2: Run tests to verify current failures**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_second_case_regression test_esv_pipeline_maze
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_second_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```
Expected: any remaining failures are exposed.

**Step 3: Write minimal implementation**
- Remove old sampled/guard/fallback assumptions from node and benchmark.
- Make the rebuilt paper-first chain the only online publish path.

**Step 4: Run full validation**
Run:
```bash
catkin_make --pkg esv_planner
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_second_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
source /home/chengzhuo/workspace/plan/devel/setup.zsh && roslaunch esv_planner demo.launch use_map_server:=true
```
Expected:
- fixed case PASS
- second case PASS
- `maze/T` and `maze/L` PASS
- online accepted trajectory `min_svsdf >= 0.1`

**Step 5: Commit**
```bash
git add esv_planner/src/esv_planner_node.cpp esv_planner/src/benchmark_planner.cpp esv_planner/src/test_esv_fixed_case_regression.cpp esv_planner/src/test_esv_second_case_regression.cpp esv_planner/src/test_esv_pipeline_maze.cpp
git commit -m "refactor: switch online pipeline to strict paper-first flow"
```

### Task 10: Performance Cleanup After Correctness

**Files:**
- Modify: `esv_planner/src/continuous_svsdf_evaluator.cpp`
- Modify: `esv_planner/src/topology_planner.cpp`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Create/Modify: benchmark notes under `docs/plans/`

**Step 1: Write/update performance regressions**
- Add timing breakdowns for front-end, sequence, evaluator, and optimizer.

**Step 2: Run baseline benchmark**
Run current benchmark/fixed case and record timings.

**Step 3: Write minimal performance improvements**
- Cache geometry queries.
- Prune weak topo candidates early.
- Avoid repeated `R2 -> SE2` recomputation.
- Keep RViz publication space-sampled and lightweight.

**Step 4: Re-run validation**
Run fixed case, second case, `maze`, and any benchmark scripts again.
Expected: correctness preserved, runtime reduced.

**Step 5: Commit**
```bash
git add esv_planner/src/continuous_svsdf_evaluator.cpp esv_planner/src/topology_planner.cpp esv_planner/src/se2_sequence_generator.cpp esv_planner/src/trajectory_optimizer.cpp docs/plans/
git commit -m "perf: optimize strict paper rebuild pipeline"
```
