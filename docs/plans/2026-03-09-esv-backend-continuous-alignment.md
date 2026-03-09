# ESV Back-End Continuous Alignment Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Remove online `Hybrid A*` fallback and move the ESV back-end from waypoint-first fitting toward paper-aligned continuous segment optimization while preserving the fixed-case safety and shape baselines.

**Architecture:** First make fallback and guard usage explicit in tests and optimizer results. Then remove online fallback from the node, refactor `optimizeSE2()` into support-state continuous optimization, and only afterward propagate the same structure to `optimizeR2()` and `stitch()`.

**Tech Stack:** ROS Noetic, C++, catkin, existing `esv_planner` regression tests, fixed-case offline harness, `roslaunch esv_planner demo.launch use_map_server:=true`.

---

### Task 1: Add no-fallback and no-guard fixed-case regression

**Files:**
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Extend the fixed-case regression so it explicitly fails when:
- the accepted result would require `Hybrid A*` online fallback semantics,
- the accepted trajectory depends on guard output rather than true continuous optimization,
- or the fixed case cannot distinguish optimizer source.

Add summary fields for:
- `used_guard`
- `source_mode`
- `continuous_source_ok`

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- build succeeds,
- regression fails because the current optimizer API cannot yet prove the accepted trajectory came from a continuous optimizer source.

**Step 3: Write minimal implementation**

Only add the data plumbing needed for the test:
- introduce a structured optimizer result type in the header;
- update the test harness to read it;
- do not change optimizer behavior yet.

**Step 4: Run test to verify it fails for the intended reason**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- the test now fails on explicit source/guard semantics instead of missing fields.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/test_esv_fixed_case_regression.cpp
git commit -m "test: require continuous-source fixed-case ESV output"
```

### Task 2: Remove `Hybrid A*` from the online publish path

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: online `roslaunch esv_planner demo.launch use_map_server:=true`

**Step 1: Write the failing verification expectation**

Define online success as:
- no `Falling back to Hybrid A* path` log,
- either a valid `ESV` trajectory is published,
- or the node explicitly reports ESV-layer failure without publishing fallback output.

**Step 2: Run verification to confirm current behavior**

Run the fixed case online and capture logs.

Expected:
- current code still contains the fallback branch, even if the fixed case does not trigger it every time.

**Step 3: Write minimal implementation**

In `esv_planner_node.cpp`:
- remove the online `Hybrid A*` publish fallback branch;
- keep diagnostics showing which ESV phase failed;
- do not yet change optimizer behavior.

**Step 4: Run verification to verify the fallback branch is gone**

Run:
```bash
roslaunch esv_planner demo.launch use_map_server:=true
```

Publish the fixed case:
- `/initialpose` with `(1.06, 7.55, -1.57)`
- `/move_base_simple/goal` with `(8.97, 3.63, 1.52)`

Expected:
- no `Falling back to Hybrid A* path` appears in logs.

**Step 5: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "refactor: remove Hybrid A* from online publish path"
```

### Task 3: Return structured optimizer results

**Files:**
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Test: `esv_planner/src/test_esv_pipeline_maze.cpp`

**Step 1: Write the failing test**

Tighten both regressions so they inspect optimizer result metadata:
- `success`
- `used_guard`
- `source_mode`
- `min_svsdf`
- `dynamics_ok`

Expected failure:
- current optimizer cannot yet return this structure consistently.

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

**Step 3: Write minimal implementation**

Refactor the optimizer API:
- add `OptimizerResult`;
- make `optimizeSE2()` and `optimizeR2()` return `OptimizerResult`;
- keep internal behavior unchanged for this task except for populating metadata truthfully.

**Step 4: Run test to verify it passes**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze esv_planner_node
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- both regressions pass with metadata now visible and correct.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp \
        esv_planner/src/test_esv_pipeline_maze.cpp \
        esv_planner/src/esv_planner_node.cpp
git commit -m "refactor: return structured ESV optimizer results"
```

### Task 4: Move `optimizeSE2()` to support-state continuous variables

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Add or Modify: `esv_planner/src/test_optimizer_continuous_se2.cpp`
- Modify: `esv_planner/CMakeLists.txt`
- Test: `esv_planner/src/test_optimizer_continuous_se2.cpp`
- Test: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Add a dedicated `SE2` optimizer regression that fails when:
- `optimizeSE2()` returns success only through guard output,
- or the accepted result still comes from dense-waypoint fitting semantics rather than the new support-state continuous path.

Keep the test focused on one behavior:
- a narrow-passage `SE2` segment must succeed via the continuous optimizer path.

**Step 2: Run test to verify it fails**

Run:
```bash
./devel/lib/esv_planner/test_optimizer_continuous_se2
```

Expected:
- current implementation still fails or reports guard-based success.

**Step 3: Write minimal implementation**

Refactor `optimizeSE2()` so it:
- extracts sparse support states from the incoming segment;
- optimizes support-state positions, yaw knots, and piece durations `T`;
- generates continuous quintic pieces from these optimized primary variables;
- reports guard usage only if the continuous path truly fails.

Do not refactor `optimizeR2()` in this task.

**Step 4: Run test to verify it passes**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_continuous_se2 test_esv_fixed_case_regression test_esv_pipeline_maze
./devel/lib/esv_planner/test_optimizer_continuous_se2
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- `SE2` regression passes,
- fixed case still passes,
- maze regression stays green,
- accepted fixed-case result no longer depends on guard output.

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h \
        esv_planner/src/trajectory_optimizer.cpp \
        esv_planner/src/test_optimizer_continuous_se2.cpp \
        esv_planner/src/test_esv_fixed_case_regression.cpp \
        esv_planner/CMakeLists.txt
git commit -m "feat: move SE2 optimization toward continuous support-state variables"
```

### Task 5: Verify fixed-case online without fallback and without guard dependence

**Files:**
- Modify: `esv_planner/src/esv_planner_node.cpp`
- Test: online `roslaunch esv_planner demo.launch use_map_server:=true`

**Step 1: Write the verification expectation**

The online fixed case must now show:
- no `Hybrid A*` fallback log,
- accepted `ESV` candidate,
- selected path metadata showing `used_guard=0`,
- `min_svsdf >= 0.0`.

**Step 2: Run verification**

Run:
```bash
roslaunch esv_planner demo.launch use_map_server:=true
```

Publish the fixed case:
- `/initialpose` with `(1.06, 7.55, -1.57)`
- `/move_base_simple/goal` with `(8.97, 3.63, 1.52)`

**Step 3: Write minimal implementation**

Only add logging/reporting required to surface:
- `source_mode`
- `used_guard`
- `selected min_svsdf`

Do not change planning policy in this task.

**Step 4: Run verification to verify acceptance evidence is visible**

Repeat the online run.

Expected:
- fixed case is accepted by ESV,
- no fallback branch exists,
- logs clearly show whether guard usage has been eliminated.

**Step 5: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "chore: log continuous-source ESV acceptance online"
```
