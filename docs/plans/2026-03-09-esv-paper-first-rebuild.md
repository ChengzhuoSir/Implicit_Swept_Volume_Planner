# ESV Paper-First Rebuild Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Rebuild the current ESV pipeline toward the paper's method in the main workspace, using the fixed maze case as the acceptance harness, with execution staged as safety first, then shape quality, then runtime.

**Architecture:** Keep the existing ROS node and test harnesses, but treat them as scaffolding. Close the paper gaps in sequence: tighten `LOW/HIGH` semantics around continuous safety, preserve segment feasibility through final trajectory construction, then align front-end and continuous collision semantics, and only then optimize runtime. Use targeted regression tests and one commit per verified subtask.

**Tech Stack:** ROS Noetic, C++, catkin, existing `esv_planner` tests, fixed-case regression harness, `roslaunch esv_planner demo.launch use_map_server:=true`.

---

### Task 1: Add a paper-aligned safety regression for the new fixed case

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Update the fixed case in `test_esv_fixed_case_regression.cpp` to use:
- start `(1.06, 7.55, -1.57)`
- goal `(8.97, 3.63, 1.52)`

Add explicit assertions for:
- accepted `ESV` candidate count is at least 1
- no `Hybrid A*` fallback is needed in the regression harness semantics
- best accepted trajectory has `min_svsdf >= 0.0`
- best accepted trajectory is dynamically feasible

Add stage diagnostics for:
- best topology clearance
- best sequence-chain clearance
- best per-segment optimized clearance
- best stitched clearance

**Step 2: Run test to verify it fails**

Run: `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: FAIL because the current code is not yet aligned to the new fixed case semantics.

**Step 3: Write minimal implementation**

Adjust only the regression harness so it measures the new case and prints the required metrics. Do not change planner code in this task.

**Step 4: Run test to verify it passes or fails for the intended planner reason**

Run: `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression && ./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: The test builds and fails only because the planner does not yet satisfy the new fixed-case safety goal.

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "test: add paper-aligned fixed-case safety regression"
```

### Task 2: Align `LOW` classification with continuous safety semantics

**Files:**
- Modify: `esv_planner/include/esv_planner/se2_sequence_generator.h`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/src/test_motion_sequence.cpp`
- Test: `esv_planner/src/test_motion_sequence.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Extend `test_motion_sequence.cpp` so that it explicitly fails when a segment is labeled `LOW` but its conservative continuous chain still has negative clearance.

Keep the test focused on one behavior:
- a `LOW` segment must be continuously feasible under the same conservative semantics used downstream.

**Step 2: Run test to verify it fails**

Run: `./devel/lib/esv_planner/test_motion_sequence`

Expected: FAIL if any `LOW` segment still hides a negative continuous-clearance transition.

**Step 3: Write minimal implementation**

In `se2_sequence_generator.cpp`:
- add or tighten a conservative continuous-clearance check for candidate `LOW` chains;
- when a candidate `LOW` chain fails that check, do not keep it as `LOW`;
- instead route it through local repair or upgrade it to `HIGH`.

Do not broaden this into front-end or optimizer changes.

**Step 4: Run test to verify it passes**

Run:
- `catkin_make --pkg esv_planner --make-args test_motion_sequence test_esv_fixed_case_regression`
- `./devel/lib/esv_planner/test_motion_sequence`
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected:
- `test_motion_sequence` passes
- fixed-case regression either improves or still fails for a later-stage reason, not for hidden `LOW` infeasibility

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/se2_sequence_generator.h \
        esv_planner/src/se2_sequence_generator.cpp \
        esv_planner/src/test_motion_sequence.cpp
git commit -m "fix: align low-risk classification with continuous safety"
```

### Task 3: Preserve per-segment safety through final stitching

**Files:**
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Tighten `test_esv_fixed_case_regression.cpp` so it distinguishes:
- segment-level accepted solutions
- final stitched trajectory

Fail when:
- segment-level trajectories are feasible,
- but `stitch()` degrades the final trajectory below `min_svsdf >= 0.0`.

**Step 2: Run test to verify it fails**

Run: `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: FAIL if stitching is the first point where fixed-case safety is lost.

**Step 3: Write minimal implementation**

In `trajectory_optimizer.cpp`:
- prefer exact concatenation of already-feasible segment trajectories;
- avoid unnecessary global re-fit when the concatenated result already satisfies collision and dynamics constraints;
- if re-fit is still required, keep the most conservative sampling strategy needed to preserve safety.

Do not change final candidate selection in this task.

**Step 4: Run test to verify it passes**

Run:
- `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression`
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: The fixed case now reaches an accepted `ESV` trajectory with `min_svsdf >= 0.0`, or the failure clearly shifts to a pre-stitch layer.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "fix: preserve segment safety through stitching"
```

### Task 4: Verify the fixed case online with the main ROS node

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: online `roslaunch esv_planner demo.launch use_map_server:=true`

**Step 1: Write the failing verification expectation**

Add node logs for:
- accepted `ESV` candidate count
- selected path index
- best trajectory `min_svsdf`
- whether fallback was used

Define the online failure condition as:
- no accepted `ESV` candidate
- or any fallback to `Hybrid A*`
- or final selected `min_svsdf < 0.0`

**Step 2: Run online verification to observe current failure**

Run the fixed case through `roslaunch esv_planner demo.launch use_map_server:=true` and publish:
- `/initialpose` with `(1.06, 7.55, -1.57)`
- `/move_base_simple/goal` with `(8.97, 3.63, 1.52)`

Expected: Either rejection or fallback before this task's code/log changes are complete.

**Step 3: Write minimal implementation**

Only add the logging and reporting needed to make online acceptance unambiguous. Do not change planning policy here.

**Step 4: Run verification to verify acceptance evidence is visible**

Run the same online command sequence again.

Expected: Logs now clearly show whether `ESV` succeeded without fallback.

**Step 5: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "chore: log fixed-case online ESV acceptance"
```

### Task 5: Shape-quality regression for oscillation-free output

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Add a simple fixed-case shape metric to the regression harness. Example constraints:
- total heading sign changes above a threshold fail,
- excessive lateral deviation from the simplified corridor fails,
- large waypoint-to-waypoint curvature oscillation fails.

Keep the metric simple and deterministic.

**Step 2: Run test to verify it fails**

Run: `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: FAIL if the current accepted ESV trajectory is safe but visibly wavy.

**Step 3: Write minimal implementation**

In `trajectory_optimizer.cpp`:
- reduce oscillation introduced by `R2` residuals and post-fit deformation;
- bias the final continuous shape toward monotone corridor-following behavior rather than local wiggle reduction alone.

Keep safety constraints from previous tasks unchanged.

**Step 4: Run test to verify it passes**

Run:
- `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression`
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`

Expected: The fixed case remains safe and becomes visibly smoother.

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_fixed_case_regression.cpp \
        esv_planner/src/trajectory_optimizer.cpp
git commit -m "fix: reduce fixed-case trajectory oscillation"
```

### Task 6: Runtime reduction after safety and shape are stable

**Files:**
- Modify: `esv_planner/src/topology_planner.cpp`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: fixed-case regression
- Test: online `roslaunch esv_planner demo.launch use_map_server:=true`

**Step 1: Write the failing runtime expectation**

Add timing output to the fixed-case regression harness and record the current baseline.

Define success as a measurable reduction in planner runtime without:
- introducing fallback,
- reducing safety below `min_svsdf >= 0.0`,
- reintroducing visible oscillation.

**Step 2: Run test to observe the current runtime baseline**

Run:
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- fixed-case online run through `roslaunch`

Expected: Capture baseline runtime numbers for offline and online flows.

**Step 3: Write minimal implementation**

Apply only targeted runtime reductions such as:
- earlier pruning of hopeless topology candidates,
- earlier rejection of continuously unsafe `LOW` chains,
- avoiding repeated optimizer/fallback work on candidates already dominated by accepted ones.

Do not weaken acceptance conditions.

**Step 4: Run tests to verify it passes**

Run:
- `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_motion_sequence`
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- `./devel/lib/esv_planner/test_motion_sequence`
- fixed-case online run through `roslaunch`

Expected: runtime is lower while safety and shape remain intact.

**Step 5: Commit**

```bash
git add esv_planner/src/topology_planner.cpp \
        esv_planner/src/se2_sequence_generator.cpp \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "perf: reduce fixed-case ESV planning time"
```

### Task 7: Re-open paper-gap work after the fixed case is stable

**Files:**
- Modify: `esv_planner/src/topology_planner.cpp`
- Modify: `esv_planner/src/svsdf_evaluator.cpp`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: all paper-alignment regressions

**Step 1: Write the failing test**

Choose the next explicit paper-gap regression after the fixed case is safe, smooth, and faster. Examples:
- topology waypoint orientation semantics,
- continuous collision consistency between sequence generation and final acceptance,
- direct optimization-variable mismatch.

**Step 2: Run test to verify it fails**

Run only the new targeted regression and confirm the failure is specific.

**Step 3: Write minimal implementation**

Close exactly one paper gap at a time.

**Step 4: Run test to verify it passes**

Run the new targeted regression plus:
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- online fixed-case verification

**Step 5: Commit**

```bash
git add [exact changed files]
git commit -m "fix: close next paper-alignment gap"
```
