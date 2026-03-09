# ESV Shape-Priority Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reduce oscillation and unnecessary bulge in the accepted fixed-case ESV trajectory while preserving the existing safety-first baseline.

**Architecture:** Keep the current safety-first pipeline intact and make shape quality measurable first. Then reduce deformation in `stitch()` and make final candidate selection shape-aware under the same hard safety gate. Avoid broad optimizer rewrites in the first shape iteration.

**Tech Stack:** ROS Noetic, C++, catkin, existing `esv_planner` tests, fixed-case regression harness, `roslaunch esv_planner demo.launch use_map_server:=true`.

---

### Task 1: Add fixed-case shape regression metrics

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Extend `test_esv_fixed_case_regression.cpp` so it computes, for the best accepted candidate:
- accepted segment-chain length,
- final sampled trajectory length,
- `length_ratio`,
- `heading_oscillation_count`,
- `max_lateral_bulge`.

Add assertions that only fire after safety acceptance:
- the fixed case still produces an accepted `ESV` candidate,
- `best_final_accepted_clearance >= 0.0`,
- and the shape metrics remain under explicit thresholds chosen from the current failing output.

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- safety conditions still pass,
- the new shape assertion fails for the intended reason.

**Step 3: Write minimal implementation**

Only modify the regression harness:
- add helpers to sample trajectory length and curvature/heading changes;
- add a helper to compute maximum lateral deviation from the accepted segment-chain polyline;
- print all shape metrics in the final summary.

Do not change planner logic in this task.

**Step 4: Run test to verify it fails for the intended reason**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- build succeeds,
- regression fails only because shape is not yet good enough.

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "test: add fixed-case shape regression metrics"
```

### Task 2: Reduce stitch-induced bulge

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Use the new fixed-case shape regression to make the first failing condition specifically attributable to final trajectory construction:
- accepted segment-chain is safe,
- final stitched trajectory is also safe,
- but `length_ratio` or `max_lateral_bulge` exceeds the threshold.

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- fail on bulge or length inflation after acceptance.

**Step 3: Write minimal implementation**

In `trajectory_optimizer.cpp`:
- keep exact concatenation as the preferred output when feasible;
- if re-fit is needed, increase sampling fidelity from accepted segment trajectories before fitting;
- avoid coarse spacing choices that introduce unnecessary bowing in the fixed case;
- keep the hard safety gate unchanged.

Do not change candidate ranking in this task.

**Step 4: Run test to verify it passes**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- fixed-case shape metrics improve enough to pass the first threshold,
- maze regression remains green.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "fix: reduce stitch-induced trajectory bulge"
```

### Task 3: Make final candidate selection shape-aware

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_esv_pipeline_maze.cpp`

**Step 1: Write the failing test**

Tighten the fixed-case regression so that among safe accepted candidates the chosen one must satisfy stronger shape bounds than the current selection.

If needed, add a focused unit test around selection semantics using synthetic trajectories with:
- equal safety,
- different bulge/length inflation,
- different smoothness cost.

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- failure shows current selection still prefers a shape-worse candidate.

**Step 3: Write minimal implementation**

In `trajectory_optimizer.cpp`:
- extend final selection cost so safe, feasible candidates are ranked by:
  - lower bulge proxy,
  - lower length inflation,
  - fewer oscillations or lower heading variation,
  - then secondary smoothness/time costs;
- keep `min_svsdf >= 0.0` and dynamics feasibility as hard filters.

**Step 4: Run test to verify it passes**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- fixed-case accepted trajectory now satisfies the stricter shape regression,
- maze regression still passes.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp \
        esv_planner/src/test_esv_pipeline_maze.cpp
git commit -m "fix: prefer less oscillatory ESV trajectories"
```

### Task 4: Verify shape-quality online on the fixed case

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: online `roslaunch esv_planner demo.launch use_map_server:=true`

**Step 1: Write the failing verification expectation**

Add concise node logs for the selected candidate:
- final `min_svsdf`,
- selected path index,
- total duration,
- final sampled path length,
- shape proxy values if readily available.

**Step 2: Run online verification to observe current behavior**

Run the fixed case through:
```bash
roslaunch esv_planner demo.launch use_map_server:=true
```

Publish:
- `/initialpose` with `(1.06, 7.55, -1.57)`
- `/move_base_simple/goal` with `(8.97, 3.63, 1.52)`

Expected:
- selected `ESV` trajectory is visible,
- logs make it clear whether the selected trajectory still shows residual inflation.

**Step 3: Write minimal implementation**

Only add the reporting needed for online shape verification. Do not change planning policy in this task.

**Step 4: Run verification to verify acceptance evidence is visible**

Run the same online command sequence again.

Expected:
- online logs clearly show the accepted `ESV` candidate and its key shape metrics,
- no fallback to `Hybrid A*`.

**Step 5: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "chore: log shape metrics for selected ESV trajectory"
```
