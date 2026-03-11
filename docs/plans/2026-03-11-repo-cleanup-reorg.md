# Repository Cleanup And Reorganization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Remove non-essential files from the `src` repository, keep the validated ROS/catkin project runnable, and leave a cleaner top-level structure with only retained planning markdowns plus `plan.pdf` and `plan_zh.pdf`.

**Architecture:** Keep the existing catkin workspace and `esv_planner` package layout intact so build, tests, and launch paths do not move. Cleanup focuses on deleting generated artifacts, temporary developer metadata, unrelated helper scripts, and unused benchmark entrypoints that are not part of the validated runtime path.

**Tech Stack:** ROS catkin, CMake, C++14, zsh, git

---

### Task 1: Record Cleanup Scope

**Files:**
- Create: `docs/plans/2026-03-11-repo-cleanup-reorg.md`
- Test: `find /home/chengzhuo/workspace/plan/src -maxdepth 3 -type f | sort`

**Step 1: Write the inventory command**

```bash
find /home/chengzhuo/workspace/plan/src -maxdepth 3 -type f | sort
```

**Step 2: Run inventory and classify**

Run: `find /home/chengzhuo/workspace/plan/src -maxdepth 3 -type f | sort`
Expected: Root-level diffs/summaries, benchmark outputs, artifacts, and helper scripts are visible.

**Step 3: Freeze keep/delete policy**

Keep:
- `.git`, `.gitignore`, `CMakeLists.txt`
- `plan.pdf`, `plan_zh.pdf`
- `docs/plans/*.md`
- `esv_planner/{include,src,launch,config,maps,package.xml,CMakeLists.txt}`

Delete:
- Root temporary files and unrelated helper tools
- Benchmark output artifacts and harness logs
- Benchmark-only executable/tooling if no longer needed for validated runtime

**Step 4: Re-run inventory after cleanup**

Run: `find /home/chengzhuo/workspace/plan/src -maxdepth 3 -type f | sort`
Expected: Only retained structure remains.

### Task 2: Remove Non-Source Repository Clutter

**Files:**
- Delete: `diff_all.txt`
- Delete: `opt_diff.txt`
- Delete: `topo_diff.txt`
- Delete: `plan_summary.md`
- Delete: `tools/pdf_translate_overlay.py`
- Delete: `.claude/settings.local.json`
- Delete: `.worktrees/`
- Delete: `esv_planner/artifacts/`
- Delete: `esv_planner/benchmark_metrics.csv`
- Delete: `esv_planner/benchmark_trajectories.csv`
- Delete: `esv_planner/benchmark_summary.md`
- Delete: `esv_planner/diff_opt.txt`

**Step 1: Delete file clutter**

Run: `rm -f /home/chengzhuo/workspace/plan/src/{diff_all.txt,opt_diff.txt,topo_diff.txt,plan_summary.md}`
Expected: Root temporary text files are removed.

**Step 2: Delete unrelated helper/tooling clutter**

Run: `rm -rf /home/chengzhuo/workspace/plan/src/.claude /home/chengzhuo/workspace/plan/src/.worktrees /home/chengzhuo/workspace/plan/src/tools /home/chengzhuo/workspace/plan/src/esv_planner/artifacts`
Expected: Developer-local metadata and unrelated tooling directories are removed.

**Step 3: Delete generated benchmark outputs**

Run: `rm -f /home/chengzhuo/workspace/plan/src/esv_planner/{benchmark_metrics.csv,benchmark_trajectories.csv,benchmark_summary.md,diff_opt.txt}`
Expected: Output artifacts are removed.

### Task 3: Remove Benchmark-Only Code Path

**Files:**
- Modify: `esv_planner/CMakeLists.txt`
- Delete: `esv_planner/src/benchmark_planner.cpp`
- Delete: `esv_planner/tools/generate_maze_map.py`
- Delete: `esv_planner/tools/long_running_harness.py`
- Delete: `esv_planner/tools/plot_benchmark.py`
- Delete: `esv_planner/tools/run_benchmark.sh`
- Delete: `esv_planner/tools/summarize_benchmark.py`

**Step 1: Write the failing removal diff**

Remove the `benchmark_planner` target from `esv_planner/CMakeLists.txt`.

**Step 2: Delete benchmark-only sources**

Delete `esv_planner/src/benchmark_planner.cpp` and `esv_planner/tools/*`.

**Step 3: Rebuild package**

Run: `catkin_make --pkg esv_planner`
Expected: Build succeeds without benchmark target.

### Task 4: Verify Runtime Structure And Behavior

**Files:**
- Modify: `esv_planner/CMakeLists.txt` if build requires follow-up cleanup
- Test: `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze test_planner esv_planner_node`
- Test: `timeout 20s roslaunch esv_planner demo.launch use_map_server:=true strict_rebuild_case:=secondary_case`

**Step 1: Build validated targets**

Run: `catkin_make --pkg esv_planner --make-args test_esv_fixed_case_regression test_esv_pipeline_maze test_planner esv_planner_node`
Expected: Build exits 0.

**Step 2: Run key tests**

Run:
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- `timeout 180s ./devel/lib/esv_planner/test_esv_pipeline_maze`
- `./devel/lib/esv_planner/test_planner`

Expected: All exit 0.

**Step 3: Run launch smoke test**

Run: `timeout 20s roslaunch esv_planner demo.launch use_map_server:=true strict_rebuild_case:=secondary_case`
Expected: Launch starts successfully and logs the configured strict rebuild case.

### Task 5: Review Final Tree And Commit

**Files:**
- Test: `git -C /home/chengzhuo/workspace/plan/src status --short`

**Step 1: Inspect final structure**

Run:
- `find /home/chengzhuo/workspace/plan/src -maxdepth 2 -type d | sort`
- `find /home/chengzhuo/workspace/plan/src -maxdepth 2 -type f | sort`

Expected: Root contains only git metadata, retained docs, PDFs, and the `esv_planner` package.

**Step 2: Inspect diff**

Run: `git -C /home/chengzhuo/workspace/plan/src status --short`
Expected: Only intended cleanup/code changes are present.

**Step 3: Commit**

```bash
git -C /home/chengzhuo/workspace/plan/src add -A
git -C /home/chengzhuo/workspace/plan/src commit -m "chore: clean and reorganize esv planner repo"
```
