# ESV Upstream Paper Rebuild Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the current hybrid experimental core with an upstream-library-based, paper-aligned ESV pipeline while keeping the ROS shell and stable regression harnesses.

**Architecture:** Keep ROS/map/test infrastructure, then rebuild the algorithmic core in this order: geometry map -> body-frame frontend -> unified continuous feasibility -> continuous collision core -> SE2 optimizer -> R2 optimizer -> stitch cleanup. Remove legacy coefficient-space CT experiments and old optimizer-specific tests as each replacement lands.

**Tech Stack:** ROS Noetic, C++, Eigen 3.4, libigl, LBFGS++, existing catkin/CMake build, geometry/polygon map utilities.

---

### Task 1: Freeze the rebuild boundary

**Files:**
- Modify: `esv_planner/CMakeLists.txt`
- Modify: `esv_planner/test/*` (only to disable legacy optimizer tests from default build)
- Test: `esv_planner/test/test_esv_fixed_case_regression.cpp`

**Step 1: Write the failing boundary test list**
Create a list in comments / local notes of legacy tests to retire from default green path:
- `test_optimizer_se2_ct_variables.cpp`
- `test_optimizer_se2_maze_t_path1_seg3.cpp`
- `test_optimizer_se2_maze_t_path1_seg3_context.cpp`

**Step 2: Disable them from default build or mark as legacy experimental**
Ensure the default catkin target no longer depends on them for the primary rebuild path.

**Step 3: Verify fixed case still passes**
Run:
```bash
catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression
./devel/lib/esv_planner/test_esv_fixed_case_regression
```
Expected: PASS with `best_final_accepted_clearance=0.1`

**Step 4: Commit**
```bash
git add esv_planner/CMakeLists.txt esv_planner/test
git commit -m "chore: retire legacy optimizer experiment tests"
```

### Task 2: Introduce Minco parameterization skeleton

**Files:**
- Create: `esv_planner/include/esv_planner/minco_parameterization.h`
- Create: `esv_planner/src/minco_parameterization.cpp`
- Modify: `esv_planner/CMakeLists.txt`
- Test: `esv_planner/test/test_minco_parameterization.cpp`

**Step 1: Write the failing test**
Test should assert:
- input variables are `q, yaw, log(T)` rather than full coefficient blocks
- generated trajectory is `C2` continuous at internal boundaries

**Step 2: Run it and confirm failure**

**Step 3: Implement minimal mapping**
Add a skeleton that:
- packs/unpacks low-dimensional variables
- maps them to quintic pieces using the existing fitting infrastructure as a temporary bridge
- exposes continuity checks

**Step 4: Run test and confirm pass**

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/minco_parameterization.h esv_planner/src/minco_parameterization.cpp esv_planner/test/test_minco_parameterization.cpp esv_planner/CMakeLists.txt
git commit -m "feat: add minco parameterization skeleton"
```

### Task 3: Introduce unified continuous evaluator skeleton

**Files:**
- Create: `esv_planner/include/esv_planner/unified_continuous_evaluator.h`
- Create: `esv_planner/src/unified_continuous_evaluator.cpp`
- Modify: `esv_planner/include/esv_planner/continuous_collision_evaluator.h`
- Test: `esv_planner/test/test_unified_continuous_evaluator.cpp`

**Step 1: Write failing test**
The test should verify:
- high-frequency state query and trajectory query share the same evaluator interface
- `min_svsdf` returned by the unified evaluator is consistent for a simple known corridor case

**Step 2: Run test to confirm failure**

**Step 3: Implement minimal evaluator skeleton**
Use current geometry map + ESDF query as the first backend, but expose the new final interface that later code will target.

**Step 4: Run test to confirm pass**

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/unified_continuous_evaluator.h esv_planner/src/unified_continuous_evaluator.cpp esv_planner/include/esv_planner/continuous_collision_evaluator.h esv_planner/test/test_unified_continuous_evaluator.cpp
git commit -m "feat: add unified continuous evaluator skeleton"
```

### Task 4: Rewrite SE2 sequence generator around pure paper semantics

**Files:**
- Modify: `esv_planner/include/esv_planner/se2_sequence_generator.h`
- Modify: `esv_planner/src/se2_sequence_generator.cpp`
- Test: `esv_planner/test/test_motion_sequence.cpp`
- Test: `esv_planner/test/test_low_high_semantics.cpp`
- Test: `esv_planner/test/test_esv_fixed_case_regression.cpp`

**Step 1: Write failing expectations**
Update tests so they require:
- `generateCore`-style segmentation becomes the only main path
- no legacy finalize expansion behavior is needed for correctness
- fixed case remains green

**Step 2: Remove legacy finalize chain**
Delete or bypass the remaining compact/legacy postprocessing logic.

**Step 3: Make sequence depend only on unified continuous feasibility**
No optimizer-specific patch paths.

**Step 4: Run tests**
```bash
catkin_make --pkg esv_planner --make-args test_motion_sequence test_low_high_semantics test_esv_fixed_case_regression
./devel/lib/esv_planner/test_motion_sequence
./devel/lib/esv_planner/test_low_high_semantics
./devel/lib/esv_planner/test_esv_fixed_case_regression
```

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/se2_sequence_generator.h esv_planner/src/se2_sequence_generator.cpp esv_planner/test/test_motion_sequence.cpp esv_planner/test/test_low_high_semantics.cpp esv_planner/test/test_esv_fixed_case_regression.cpp
git commit -m "refactor: rewrite sequence layer around unified feasibility"
```

### Task 5: Replace optimizer entry point with low-dimensional SE2 parameterization

**Files:**
- Modify: `esv_planner/include/esv_planner/trajectory_optimizer.h`
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Modify: `esv_planner/include/esv_planner/minco_parameterization.h`
- Modify: `esv_planner/src/minco_parameterization.cpp`
- Test: `esv_planner/test/test_optimizer_se2_continuous_polynomial.cpp`
- Test: `esv_planner/test/test_esv_fixed_case_regression.cpp`

**Step 1: Write failing test**
Require that `SE2` no longer uses full coefficient blocks as the optimization variable family.

**Step 2: Remove legacy coefficient-space CT path**
Delete:
- `finiteDiffObjective`
- full coefficient block packing
- reduced/full CT switching
- continuity penalty driven full-coefficient optimization

**Step 3: Implement low-dimensional optimizer path**
Use `MincoParameterization` to optimize only `q, yaw, log(T)` with `LBFGS++`.

**Step 4: Run tests**
Expect fixed case to stay green.

**Step 5: Commit**
```bash
git add esv_planner/include/esv_planner/trajectory_optimizer.h esv_planner/src/trajectory_optimizer.cpp esv_planner/include/esv_planner/minco_parameterization.h esv_planner/src/minco_parameterization.cpp esv_planner/test/test_optimizer_se2_continuous_polynomial.cpp esv_planner/test/test_esv_fixed_case_regression.cpp
git commit -m "refactor: replace se2 optimizer with low-dimensional minco path"
```

### Task 6: Rebuild R2 optimizer on the same parameterization

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: `esv_planner/test/test_optimizer_r2_continuous_polynomial.cpp`
- Test: `esv_planner/test/test_esv_pipeline_maze.cpp`

**Step 1: Write failing test**
Require R2 to use the same low-dimensional continuous parameterization and no guard main path.

**Step 2: Implement minimal R2 replacement**

**Step 3: Run tests**

**Step 4: Commit**
```bash
git add esv_planner/src/trajectory_optimizer.cpp esv_planner/test/test_optimizer_r2_continuous_polynomial.cpp esv_planner/test/test_esv_pipeline_maze.cpp
git commit -m "refactor: rebuild r2 optimizer on minco parameterization"
```

### Task 7: Reduce stitch to a connector

**Files:**
- Modify: `esv_planner/src/trajectory_optimizer.cpp`
- Test: `esv_planner/test/test_stitch_continuity.cpp`
- Test: `esv_planner/test/test_esv_pipeline_maze.cpp`

**Step 1: Write failing test**
Require stitch to preserve segment-level safety and continuity without re-optimizing geometry.

**Step 2: Remove legacy seam/guard behavior**

**Step 3: Implement connector-only stitch**

**Step 4: Run tests**

**Step 5: Commit**
```bash
git add esv_planner/src/trajectory_optimizer.cpp esv_planner/test/test_stitch_continuity.cpp esv_planner/test/test_esv_pipeline_maze.cpp
git commit -m "refactor: reduce stitch to continuity connector"
```

### Task 8: Restore full regression set under the new core

**Files:**
- Modify: `esv_planner/test/test_esv_fixed_case_regression.cpp`
- Modify: `esv_planner/test/test_esv_second_case_regression.cpp`
- Modify: `esv_planner/test/test_esv_pipeline_maze.cpp`
- Modify: `esv_planner/src/esv_planner_node.cpp`

**Step 1: Run fixed case**
Expected:
- `best_final_accepted_clearance=0.1`
- `used_guard=0`
- `continuous_source_ok=1`

**Step 2: Run second case and maze**
Expected:
- accepted trajectory from ESV only
- online accepted trajectories satisfy `min_svsdf >= 0.1`

**Step 3: Run online check**
```bash
source /home/chengzhuo/workspace/plan/devel/setup.zsh
roslaunch esv_planner demo.launch use_map_server:=true
```
Verify logs show accepted ESV path and no `Hybrid A*` publish fallback.

**Step 4: Commit**
```bash
git add esv_planner/test/test_esv_fixed_case_regression.cpp esv_planner/test/test_esv_second_case_regression.cpp esv_planner/test/test_esv_pipeline_maze.cpp esv_planner/src/esv_planner_node.cpp
git commit -m "test: restore full regression set on rebuilt paper core"
```
