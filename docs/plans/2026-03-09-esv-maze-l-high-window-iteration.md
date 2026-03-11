# ESV Maze-L High-Window Iteration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reduce unnecessary `HIGH` segments for the `maze/L` benchmark, raise `accepted_candidates` to at least 2, and replace the outdated `test_motion_sequence` assumptions while preserving fixed-case ESV success without `Hybrid A*` fallback.

**Architecture:** Keep the strict final collision gate, but shrink `HIGH` windows by separating edge-failure handling from true state infeasibility inside `SE2SequenceGenerator`. Update the motion-sequence unit test to validate current paper-aligned semantics: small local repairs are preferred, continuous safety matters, and not every recoverable case must remain globally `LOW`.

**Tech Stack:** ROS Noetic, catkin, C++, `esv_planner` topology/sequence/optimizer pipeline, existing integration tests.

---

### Task 1: Rewrite Motion-Sequence Test Semantics

**Files:**
- Modify: `esv_planner/src/test_motion_sequence.cpp`
- Reference: `esv_planner/src/se2_sequence_generator.cpp`

**Step 1: Write the failing test**

Replace the old assertion set with checks that match current behavior:
- the synthetic path should still produce at least one local repair-like segment
- `HIGH` windows should be small and localized, not one giant fallback segment
- every adjacent pair inside `LOW` segments should remain continuously collision-free when sampled

**Step 2: Run test to verify it fails**

Run:

```bash
./devel/lib/esv_planner/test_motion_sequence
```

Expected:
- `FAIL` under the old assertions or missing new helper logic

**Step 3: Write minimal implementation**

Add helper sampling inside the test only. Do not change planner code yet.

**Step 4: Run test to verify it passes**

Run:

```bash
./devel/lib/esv_planner/test_motion_sequence
```

Expected:
- `PASS`

**Step 5: Commit**

```bash
git add esv_planner/src/test_motion_sequence.cpp
git commit -m "test: align motion-sequence regression with continuous safety"
```

### Task 2: Add A Focused Maze/L High-Window Regression

**Files:**
- Modify: `esv_planner/src/test_esv_pipeline_maze.cpp`
- Reference: `esv_planner/src/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing test**

Add explicit per-robot assertions for:
- `L` robot `accepted_candidates >= 2`
- `L` robot `traj_empty == 0`
- report the largest `HIGH` window size and total `HIGH` segment count

**Step 2: Run test to verify it fails**

Run:

```bash
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- `FAIL` for `L`

**Step 3: Write minimal implementation**

Only add diagnostics and assertions in the test.

**Step 4: Run test to verify it fails for the intended reason**

Expected:
- failure points to excessive `HIGH` windows / low candidate count, not build issues

**Step 5: Commit**

```bash
git add esv_planner/src/test_esv_pipeline_maze.cpp
git commit -m "test: add maze L high-window regression"
```

### Task 3: Shrink Edge-Driven HIGH Windows

**Files:**
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Modify: `esv_planner/include/esv_planner/se2_sequence_generator.h`
- Reference: `esv_planner/src/test_motion_sequence.cpp`
- Reference: `esv_planner/src/test_esv_pipeline_maze.cpp`

**Step 1: Write the failing test**

Use the updated motion-sequence and maze tests as the red state.

**Step 2: Run tests to verify they fail**

Run:

```bash
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- `motion_sequence` and/or `maze/L` fail because edge failures are still expanded too aggressively

**Step 3: Write minimal implementation**

Adjust `SE2SequenceGenerator` so that:
- edge-failure windows stay local
- short repaired windows rejoin neighboring `LOW` segments when possible
- only truly irrecoverable local windows remain `HIGH`

Do not touch optimizer selection or final gate in this task.

**Step 4: Run tests to verify they pass**

Run:

```bash
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- `motion_sequence` passes
- `maze/L` accepted candidates increase and `HIGH` usage drops

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/se2_sequence_generator.h esv_planner/src/se2_sequence_generator.cpp esv_planner/src/test_motion_sequence.cpp esv_planner/src/test_esv_pipeline_maze.cpp
git commit -m "fix: shrink unnecessary high-risk windows"
```

### Task 4: Re-Validate Fixed Case And Demo Behavior

**Files:**
- Reference only unless needed: `esv_planner/src/test_esv_fixed_case_regression.cpp`
- Reference only unless needed: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Run fixed-case regression**

Run:

```bash
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- `PASS`

**Step 2: Run online demo validation**

Run the fixed start/goal through:

```bash
roslaunch esv_planner demo.launch use_map_server:=true
```

Expected:
- ESV path accepted for the fixed case
- no `Hybrid A*` fallback for that case

**Step 3: Commit**

```bash
git add esv_planner/src/esv_planner_node.cpp
git commit -m "test: verify fixed-case ESV behavior after high-window tuning"
```
