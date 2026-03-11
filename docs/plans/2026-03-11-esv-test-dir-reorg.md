# ESV Test Directory Reorganization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reorganize the `esv_planner` package so that tests live under `esv_planner/test/` while `esv_planner/src/` only contains core implementation, without breaking build or runtime verification.

**Architecture:** The package structure stays catkin-compatible and stable. The change is limited to moving test entry files, updating CMake source paths, and consolidating shared implementation source lists so existing executables keep the same names.

**Tech Stack:** ROS Noetic, catkin, CMake, C++14, zsh, git

---

### Task 1: Create Target Layout

**Files:**
- Create: `esv_planner/test/`
- Modify: `esv_planner/CMakeLists.txt`

**Step 1: Add the new test directory**

Run: `mkdir -p /home/chengzhuo/workspace/plan/src/esv_planner/test`
Expected: `esv_planner/test/` exists.

**Step 2: Identify all moved test entrypoints**

Run: `find /home/chengzhuo/workspace/plan/src/esv_planner/src -maxdepth 1 -name 'test_*.cpp' | sort`
Expected: all current package-local test entry files are listed.

### Task 2: Move Test Entrypoints

**Files:**
- Move: `esv_planner/src/test_*.cpp` -> `esv_planner/test/test_*.cpp`

**Step 1: Move files without changing contents**

Run: `git -C /home/chengzhuo/workspace/plan/src mv esv_planner/src/test_*.cpp esv_planner/test/`
Expected: `src/` no longer contains package-local test entrypoints.

**Step 2: Verify separation**

Run:
- `find /home/chengzhuo/workspace/plan/src/esv_planner/src -maxdepth 1 -name 'test_*.cpp'`
- `find /home/chengzhuo/workspace/plan/src/esv_planner/test -maxdepth 1 -name 'test_*.cpp' | sort`

Expected: first command prints nothing, second lists all moved tests.

### Task 3: Update Build Definitions

**Files:**
- Modify: `esv_planner/CMakeLists.txt`

**Step 1: Define shared implementation source lists**

Add reusable variables for core implementation source files, then reuse them in runtime and test executables.

**Step 2: Rewrite test target entry paths**

Replace `src/test_*.cpp` with `test/test_*.cpp` for all test targets while keeping target names unchanged.

**Step 3: Keep build behavior stable**

Do not alter dependency packages, link libraries, or target names unless the move requires it.

### Task 4: Rebuild And Verify

**Files:**
- Modify: `esv_planner/CMakeLists.txt` if follow-up path fixes are needed

**Step 1: Build package**

Run: `catkin_make --pkg esv_planner`
Expected: build exits 0.

**Step 2: Run key tests**

Run:
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- `timeout 180s ./devel/lib/esv_planner/test_esv_pipeline_maze`
- `./devel/lib/esv_planner/test_planner`
- `./devel/lib/esv_planner/test_low_high_semantics`
- `./devel/lib/esv_planner/test_stitch_seam_connector`
- `./devel/lib/esv_planner/test_continuous_svsdf_evaluator`
- `./devel/lib/esv_planner/test_geometry_map`
- `./devel/lib/esv_planner/test_body_frame_sdf`
- `./devel/lib/esv_planner/test_topology_body_push`

Expected: all exit 0.

**Step 3: Run launch smoke test**

Run: `timeout 20s roslaunch esv_planner demo.launch use_map_server:=true strict_rebuild_case:=secondary_case`
Expected: planner node starts and logs the configured strict rebuild case.

### Task 5: Inspect Final Layout And Commit

**Files:**
- Test: `git -C /home/chengzhuo/workspace/plan/src status --short`

**Step 1: Confirm final layout**

Run:
- `find /home/chengzhuo/workspace/plan/src/esv_planner/src -maxdepth 1 -type f | sort`
- `find /home/chengzhuo/workspace/plan/src/esv_planner/test -maxdepth 1 -type f | sort`

Expected: `src/` contains only core implementation and `test/` contains all package-local tests.

**Step 2: Commit**

```bash
git -C /home/chengzhuo/workspace/plan/src add -A
git -C /home/chengzhuo/workspace/plan/src commit -m "refactor: move esv planner tests into package test dir"
```
