# ESV Strict Paper Rebuild Design

**Date:** 2026-03-11

## Goal

Rebuild `esv_planner` so the main online pipeline follows the paper's method as closely as practical, instead of extending the current engineering approximation. The rebuilt system must publish only ESV trajectories, require `min_svsdf >= 0.1` for every accepted online trajectory, and use the paper's geometry, continuous-feasibility, and optimization semantics as the primary design constraints.

## Why A Rebuild Is Required

The current codebase has moved beyond the original rough prototype, but the main mathematical objects still do not match the paper:

1. The front-end still originates from a roadmap plus shortcut pipeline, with only partial body-frame geometry influence.
2. Continuous collision checking is still a bounded continuous approximation built on sampled state queries, not the paper's true swept-volume formulation.
3. The optimizer is still waypoint/support-state first, with continuous trajectory generation applied after state repair, rather than directly optimizing the continuous trajectory variables.

Those three mismatches explain why some cases still require targeted repair logic, why complex `HIGH` windows remain fragile, and why current behavior is closer to a paper-inspired system than a faithful reconstruction.

## Constraints

- Work only in the main workspace: `/home/chengzhuo/workspace/plan/src`
- Keep existing ROS integration, launch files, map loading, and visualization shell unless they block the new architecture
- Allow large code changes and new dependencies
- Default online acceptance rule: `min_svsdf >= 0.1`
- Final trajectory must come from the ESV pipeline only; no online `Hybrid A*` publish fallback
- Primary acceptance cases:
  - Fixed case: `(1.06, 7.55, -1.57) -> (8.97, 3.63, 1.52)`
  - Secondary regression case: `(0.72, 5.31, 0.29) -> (9.14, 5.74, 1.64)`
  - Maze regression: `maze/T`, `maze/L`

## Target Architecture

### 1. Geometry Map Layer

The occupancy grid remains an input format, not the runtime planning geometry.

Runtime geometry will be split into:
- obstacle boundary contours extracted from the grid
- a 2D segment/polygon representation for exact local geometry queries
- a spatial acceleration structure (KD-tree or BVH)
- a continuous ESDF helper for cheap optimizer-side queries

This layer exists so the planner no longer performs local-cell brute force inside high-frequency collision loops.

### 2. Body-Frame Geometry Front-End

The front-end will be rebuilt around robot-centric geometry queries.

Target behavior:
- convert local obstacle geometry into the robot body frame
- query signed distance and gradient against the robot geometry
- use that gradient to perform obstruction push-away and narrow-passage repairs
- keep orientation-aware topology waypoints as first-class data, not tangent-yaw placeholders

This layer will be implemented with `libigl`-backed signed distance / winding support for robot geometry, plus local obstacle contour queries from the geometry layer.

### 3. Unified Continuous Feasibility Layer

A single continuous-feasibility interface will define:
- state clearance
n- transition clearance
- segment feasibility
- final trajectory acceptance semantics

Every stage must use the same feasibility meaning:
- topology/path filtering
- `SafeYaw`
- recursive `SegAdjust`
- `LOW/HIGH` labeling
- optimizer safety objective
- final online acceptance

The current split between sampled evaluator logic, sequence heuristics, and final acceptance is explicitly removed.

### 4. Continuous Collision Layer

The next evaluator will be bounded and practical, but structurally aligned with the paper:
- continuous interval reasoning over trajectory pieces
- local obstacle set pruning from the geometry map
- no local occupied-cell brute force loops
- no unbounded recursive subdivision
- no high-frequency body-frame polygon scans inside every state evaluation

This will still be an approximation to the paper's implicit swept-volume CCA, but it will be designed as a unified continuous collision kernel rather than a fixed-step sampler.

### 5. Continuous Optimizer Layer

The optimizer will be rebuilt in two stages:

- `SE2` first
- `R2` second

Target state:
- the optimizer's main variables are continuous trajectory support variables and segment times
- continuous trajectory generation is part of the optimization path, not a post-fit after waypoint repair
- `stitch()` becomes a continuity connector and timing coordinator only
- ad hoc guard paths remain only as debug diagnostics, not accepted-source logic

This is the critical step needed to move from the current support-state repair system toward the paper's MINCO-style optimization intent.

## Stage Plan

### Stage A: Geometry Map Rebuild
- Add contour extraction from occupancy grid
- Add segment/polygon representation and local obstacle index
- Keep the old map interface only as input compatibility

### Stage B: Body-Frame Front-End Rebuild
- Add body-frame SDF module
- Replace topology push-away and shortcut obstruction repair with body-frame geometry
- Keep topology responsibilities narrow; do not let `shortenPaths()` become a local planner

### Stage C: Continuous Feasibility Rebuild
- Introduce a dedicated feasibility interface used by sequence, optimizer, and final validation
- Retire ad hoc local checks that do not match final acceptance semantics

### Stage D: Continuous Collision Rebuild
- Replace the current high-frequency sampled evaluator with the bounded continuous evaluator behind the unified interface
- Enforce `min_svsdf >= 0.1` consistently in all online accepted trajectories

### Stage E: `SE2` Continuous Optimizer Rebuild
- Replace waypoint-first `SE2` repair path with a continuous-variable path
- Use dedicated red tests for small and medium `HIGH` windows

### Stage F: `R2` Continuous Optimizer Rebuild
- Rebuild `R2` around the same continuous-variable architecture
- Prevent `LOW` segments from acting as a loose post-smoother

### Stage G: Stitch And Online Path Cleanup
- Reduce `stitch()` to continuity-preserving connection
- Remove non-paper online acceptance branches
- Keep only ESV-produced trajectories in the published path

### Stage H: Runtime Cleanup
- Optimize only after the rebuilt paper-aligned pipeline is stable
- Cache geometry queries
- prune weak topology candidates early
- avoid repeated `R2 -> SE2` re-solving

## Validation Strategy

Every stage must pass three levels of validation before moving on:

1. Focused unit/regression tests for the module being rebuilt
2. Fixed-case integration:
   - `(1.06, 7.55, -1.57) -> (8.97, 3.63, 1.52)`
3. Broader regressions:
   - `(0.72, 5.31, 0.29) -> (9.14, 5.74, 1.64)`
   - `maze/T`
   - `maze/L`

Global acceptance requirements:
- final trajectory must be produced by ESV
- no online `Hybrid A*` publish fallback
- accepted online trajectory must satisfy `min_svsdf >= 0.1`
- fixed case must remain green during the whole rebuild

## Dependencies

Allowed new dependencies:
- `libigl` for signed distance / generalized winding support
- one continuous optimization backend, likely `LBFGS++` or `Ceres`

Dependency introduction order:
1. `libigl`
2. geometry index support if needed
3. continuous optimization backend

## Git Strategy

All work stays in the main workspace, but commits are stage-scoped and bisectable.

Planned commit sequence:
1. `docs: add strict paper rebuild design`
2. `docs: add strict paper rebuild implementation plan`
3. `feat: add geometry map layer for strict paper rebuild`
4. `feat: add body-frame sdf frontend pipeline`
5. `refactor: unify continuous feasibility semantics`
6. `feat: replace sampled evaluator with bounded continuous kernel`
7. `refactor: rebuild se2 optimizer around continuous variables`
8. `refactor: rebuild r2 optimizer around continuous variables`
9. `refactor: reduce stitch to continuity connector`
10. `refactor: remove non-paper online fallback path`
11. `perf: optimize paper-aligned runtime hotspots`

## Risks

1. `maze/T` and `maze/L` may regress during intermediate stages. This is acceptable as long as the fixed case remains green and each stage improves architectural alignment.
2. `libigl` and the optimizer backend will increase build complexity.
3. The first fully unified continuous-feasibility stage will likely expose failures that the current mixed pipeline hides.
4. The optimizer rebuild is the highest-risk step and must be protected by small-window and medium-window red tests.

## Success Definition

This rebuild is successful when:
- the main planner path is structurally paper-aligned
- fixed case and secondary case both complete via ESV only
- accepted online trajectories satisfy `min_svsdf >= 0.1`
- `maze/T` and `maze/L` recover under the new architecture
- the final system no longer depends on support-state repair heuristics as the dominant success mechanism
