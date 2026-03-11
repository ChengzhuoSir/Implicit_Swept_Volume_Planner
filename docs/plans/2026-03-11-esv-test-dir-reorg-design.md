# ESV Test Directory Reorganization Design

**Date:** 2026-03-11

**Goal:** Move all package-local test entry files out of `esv_planner/src/` into `esv_planner/test/`, keep `esv_planner/src/` for core implementation only, and preserve current build, test, and launch behavior.

## Scope

- Keep package layout stable: `include/`, `src/`, `launch/`, `config/`, `maps/`
- Add `esv_planner/test/`
- Move every `test_*.cpp` entry file from `esv_planner/src/` to `esv_planner/test/`
- Keep headers in `esv_planner/include/esv_planner/`
- Keep executable target names unchanged
- Do not change test logic unless a path move exposes an implicit dependency

## Chosen Approach

Use a minimal-risk package-local reorganization:

- `esv_planner/src/` contains only non-test implementation `.cpp`
- `esv_planner/test/` contains all test entry `.cpp`
- `esv_planner/CMakeLists.txt` explicitly references `test/*.cpp`
- Shared implementation files are centralized in a reusable `ESV_CORE_SOURCES` CMake variable

This keeps the catkin package layout intact and limits behavior changes to filesystem paths plus CMake source lists.

## Alternatives Considered

1. Leave some tests in `src/`
   Rejected because the resulting layout remains mixed and does not satisfy the cleanup goal.

2. Fully re-split runtime code into additional subdirectories like `src/core/` and `test/integration/`
   Rejected because the path churn is larger and unnecessary for the current goal.

## Build Strategy

- Preserve all current executable names such as `test_planner`, `test_esv_pipeline_maze`, and `esv_planner_node`
- Replace duplicated core source lists with a shared `ESV_CORE_SOURCES` variable where practical
- Update test targets to point to `test/test_*.cpp`
- Keep link behavior and library dependencies unchanged

## Verification

Run fresh verification after reorganization:

- `catkin_make --pkg esv_planner`
- `./devel/lib/esv_planner/test_esv_fixed_case_regression`
- `timeout 180s ./devel/lib/esv_planner/test_esv_pipeline_maze`
- `./devel/lib/esv_planner/test_planner`
- `./devel/lib/esv_planner/test_low_high_semantics`
- `./devel/lib/esv_planner/test_stitch_seam_connector`
- `./devel/lib/esv_planner/test_continuous_svsdf_evaluator`
- `./devel/lib/esv_planner/test_geometry_map`
- `./devel/lib/esv_planner/test_body_frame_sdf`
- `./devel/lib/esv_planner/test_topology_body_push`
- `timeout 20s roslaunch esv_planner demo.launch use_map_server:=true strict_rebuild_case:=secondary_case`

## Success Criteria

- `esv_planner/src/` contains only core implementation files
- `esv_planner/test/` contains all package-local test entry files
- CMake builds all previous targets without renaming commands
- Key regression tests and ROS launch smoke test still pass
