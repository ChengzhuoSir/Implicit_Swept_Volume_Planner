# ESV SE2 c/T + LBFGS Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move the `SE2` optimizer from support-state-first optimization to a continuous coefficient-and-time (`c/T`) optimization path solved by `LBFGS++`.

**Architecture:** Keep support states only as initialization. Build an optimization vector from continuous quintic coefficients and log-durations, evaluate a continuous objective, and solve it with `LBFGS++`. Keep the migration scoped to `SE2` only for this stage.

**Tech Stack:** C++, ROS Noetic, Eigen, vendored `LBFGS++`, existing bounded continuous collision evaluator.

---

### Task 1: Add regression proving `SE2` still lacks true c/T optimization

**Files:**
- Create: `/home/chengzhuo/workspace/plan/src/esv_planner/test/test_optimizer_se2_ct_variables.cpp`
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/CMakeLists.txt`

**Step 1: Write the failing test**

Add a test that:
- extracts a representative fixed-case `HIGH` segment
- runs `optimizeSE2Detailed()`
- compares returned piece durations with a simple `allocateTime(...)`-equivalent geometric allocation
- fails if all durations are still effectively geometric-allocation outputs

**Step 2: Run test to verify it fails**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_se2_ct_variables && ./devel/lib/esv_planner/test_optimizer_se2_ct_variables
```

Expected:
- `FAIL`
- output showing the current result still mirrors geometric allocation or lacks an explicit c/T source marker

**Step 3: Commit red test only after confirming failure**

```bash
git add esv_planner/CMakeLists.txt esv_planner/test/test_optimizer_se2_ct_variables.cpp
git commit -m "test: add se2 c-t optimizer regression"
```

### Task 2: Add explicit SE2 c/T optimization data model

**Files:**
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/src/trajectory_optimizer.cpp`

**Step 1: Add internal structs**

Add internal helpers for:
- `Se2CtVariables`
- `Se2CtProblem`
- pack/unpack helpers for position coeffs, yaw coeffs, and `log(T)`

**Step 2: Build minimal implementation skeleton**

Add declarations/definitions for:
- `packSe2CtVariables(...)`
- `unpackSe2CtVariables(...)`
- `buildTrajectoryFromCtVariables(...)`

Return placeholders if needed, but keep compilation green.

**Step 3: Run focused build**

Run:
```bash
catkin_make --pkg esv_planner --make-args test_optimizer_se2_ct_variables
```

Expected:
- build passes
- test still fails for the intended red reason

**Step 4: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/esv_planner/src/trajectory_optimizer.cpp
```

Use the correct path in practice and commit:
```bash
git commit -m "refactor: add se2 c-t optimization scaffolding"
```

### Task 3: Implement SE2 c/T objective and LBFGS++ solve path

**Files:**
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/src/trajectory_optimizer.cpp`
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/include/esv_planner/trajectory_optimizer.h`

**Step 1: Implement objective evaluation**

Implement:
- smoothness term
- total time term
- continuous safety term via current continuous evaluator
- dynamics soft penalty

**Step 2: Implement LBFGS++ optimization entrypoint**

Add:
- `optimizeSE2CtLbfgs(...)`

This should:
- seed from the current quintic fit over support states
- optimize coefficients and `log(T)`
- rebuild a trajectory from the optimized variables

**Step 3: Wire `optimizeSE2Detailed()` to use c/T path first**

For covered `SE2` windows:
- call the new c/T optimizer first
- accept if it meets current hard gates
- only keep the legacy path as temporary comparison during migration

**Step 4: Run red test again**

Run:
```bash
./devel/lib/esv_planner/test_optimizer_se2_ct_variables
```

Expected:
- now `PASS`
- output shows durations are no longer trivial geometric allocation

**Step 5: Commit**

```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/esv_planner/src/trajectory_optimizer.cpp
git commit -m "feat: add se2 c-t lbfgs optimization path"
```

### Task 4: Prove accepted SE2 results come from the new c/T path

**Files:**
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/test/test_optimizer_se2_continuous_polynomial.cpp`
- Modify: `/home/chengzhuo/workspace/plan/src/esv_planner/test/test_esv_fixed_case_regression.cpp`

**Step 1: Extend tests**

Add assertions that:
- `source_mode` is still continuous
- covered `SE2` segments are produced by the new c/T path marker
- clearance remains `>= 0.1`

**Step 2: Run tests**

Run:
```bash
./devel/lib/esv_planner/test_optimizer_se2_continuous_polynomial
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

Expected:
- both `PASS`

**Step 3: Commit**

```bash
git add esv_planner/test/test_optimizer_se2_continuous_polynomial.cpp esv_planner/test/test_esv_fixed_case_regression.cpp
git commit -m "test: require se2 c-t path for accepted fixed-case results"
```

### Task 5: Full regression

**Files:**
- No code changes required unless regression fails

**Step 1: Run focused regressions**

```bash
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_pipeline_maze
```

Expected:
- all `PASS`

**Step 2: If any fail, fix minimally in `TrajectoryOptimizer` only**

Do not touch:
- `TopologyPlanner`
- `SE2SequenceGenerator`
- evaluator core

unless the regression proves the optimizer contract itself is wrong.

**Step 3: Commit regression fixes**

```bash
git add esv_planner/src/esv_planner/src/trajectory_optimizer.cpp
```

Use the correct path in practice and commit with a focused message.

