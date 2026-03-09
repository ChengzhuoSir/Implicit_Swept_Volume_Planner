# ESV Back-End Continuous Alignment Design

**Date:** 2026-03-09

**Primary Goal:**
Remove non-paper fallback behavior from the online ESV pipeline and move the back-end from waypoint-first fitting toward paper-aligned continuous trajectory optimization, using the fixed maze case as the hard acceptance harness.

**Fixed Acceptance Case:**
- Map: `esv_planner/maps/maze.pgm`
- Start: `(1.06, 7.55, -1.57)`
- Goal: `(8.97, 3.63, 1.52)`

**Hard Constraints:**
- The online node must not publish `Hybrid A*` fallback trajectories.
- The fixed case must still be solved by `ESV`.
- The accepted fixed-case trajectory must satisfy `min_svsdf >= 0.0`.
- The accepted fixed-case trajectory must remain dynamically feasible.
- The accepted fixed-case trajectory must not rely on shape-guard fallback as the normal success path.

## Why A Back-End Rebuild Is Required

The current branch has already stabilized:
- safety at the fixed case;
- no required `Hybrid A*` fallback for the accepted fixed-case path;
- reduced oscillation and bulge in the accepted output.

However, the current implementation still reaches that result through engineering guardrails rather than through a paper-aligned optimizer:

1. `TrajectoryOptimizer` still optimizes dense waypoint chains first and fits continuous polynomials second.
2. Shape-preserving guard logic still acts as an important safety net for accepted trajectories.
3. `stitch()` still has the power to reshape trajectories instead of only concatenating already-optimized continuous segments.
4. The online node still contains a non-paper fallback branch in its publish logic.

This means the current result quality is acceptable, but the method is still a prototype rather than a faithful realization of the paper's back-end.

## Accepted Direction

Use a **paper-first back-end realignment** strategy.

This means:
- remove `Hybrid A*` from the online publish path;
- make optimizer outputs explicit about whether they came from true continuous optimization or from guards;
- move segment optimization to a sparse continuous-variable representation;
- demote guards to diagnostics and development-time assertions, not normal success paths.

## Current Gap To The Paper

### Current behavior
- `optimizeSE2()` mutates dense interior waypoints, then fits a continuous trajectory.
- `optimizeR2()` does the same in position-only form.
- `stitch()` may still create a new global fit.
- guard trajectories such as polyline or rotate-translate can still become the returned segment result.

### Paper-aligned target
- the optimized object is the continuous trajectory parameterization itself;
- segment times are optimization variables, not post-hoc allocations only;
- `SE(2)` and `R²` segments optimize different variable sets but still produce continuous outputs directly;
- stitching is a composition step, not a second optimizer;
- non-paper guard trajectories are not part of the normal success path.

## New Back-End Architecture

### 1. Structured optimizer results

`TrajectoryOptimizer` should stop returning only `Trajectory`.

Each segment optimizer should return structured information:
- `success`
- `traj`
- `source_mode`
- `used_guard`
- `min_svsdf`
- `dynamics_ok`
- `cost`

This is required so tests and the online node can distinguish:
- true optimizer success
- guard-based rescue
- genuine optimizer failure

### 2. Continuous support-state optimization

The first paper-aligned step is not a full MINCO rewrite. It is a controlled shift from dense waypoint optimization to sparse continuous support states:

- choose sparse support states from the incoming motion segment;
- optimize support-state positions and yaw knots;
- optimize piece durations `T`;
- generate continuous quintic pieces directly from these primary variables.

This is already substantially closer to the paper than the current dense-waypoint-first pipeline.

### 3. Role split between `SE(2)` and `R²`

#### `SE(2)` segments
Optimize:
- `x`
- `y`
- `yaw`
- piece times `T`

Cost terms:
- smoothness
- safety
- time
- dynamics

#### `R²` segments
Optimize:
- `x`
- `y`
- piece times `T`

Yaw generation:
- derived from continuous tangent and boundary orientation constraints
- not from a disconnected post-fit heuristic

Cost terms:
- smoothness
- residual/corridor consistency
- time

This keeps the paper's division between harder `SE(2)` passages and lighter `R²` passages, but moves both into a continuous-variable setting.

### 4. Stitch as composition only

`stitch()` must stop acting like a second optimizer.

Its new role should be:
- concatenate already-optimized continuous pieces;
- check continuity at boundaries;
- optionally do lightweight global time reconciliation;
- never invent a new geometry unless explicitly invoked in a future dedicated global optimization stage.

### 5. Guards downgraded to diagnostics

Existing guards:
- polyline fallback
- rotate-translate fallback
- shape guard fallback

must remain available for diagnosis during development, but not as the normal accepted source of a published trajectory.

The desired end state is:
- accepted fixed-case trajectory comes from continuous optimization,
- `used_guard == false`,
- guards exist only for debug output and development-time assertions.

## Main-Flow Changes

### Online node

The node must:
- stop publishing `Hybrid A*` as fallback;
- report `ESV` failure modes clearly when no candidate is acceptable;
- only publish trajectories produced by the ESV pipeline itself.

`Hybrid A*` remains useful for:
- offline comparison,
- debugging,
- benchmark baselines,

but it must not remain in the online publish path.

### Tests

The fixed-case regression must be strengthened so that success now means:
- accepted `ESV` candidate exists;
- no `Hybrid A*` publish fallback is used;
- accepted candidate came from continuous optimization, not guard rescue;
- shape and safety metrics remain acceptable.

## Candidate Approaches

### Approach A: Structured-result + SE(2)-first continuous refactor (recommended)

1. Add structured optimizer results and no-fallback tests.
2. Remove `Hybrid A*` from the online publish path.
3. Refactor `optimizeSE2()` first into support-state + time variables.
4. Refactor `optimizeR2()` second.
5. Simplify `stitch()` into composition.

**Why recommend it:**
- highest paper alignment per unit risk;
- keeps the fixed case as a controlled harness;
- preserves working safety checks while replacing the core optimization semantics.

### Approach B: Remove fallback first, delay optimizer changes

**Why not enough:**
- it would make the node purer, but the optimizer would still depend on non-paper guards internally.

### Approach C: Full optimizer rewrite in one shot

**Why not first:**
- highest regression risk;
- too hard to isolate which change restores or breaks the fixed case.

## Accepted Plan

Proceed with **Approach A**.

The next execution wave will:
1. make fallback and guard usage measurable and testable;
2. remove `Hybrid A*` from the online publish path;
3. rework `optimizeSE2()` into a support-state continuous optimizer;
4. then carry the same pattern into `optimizeR2()` and `stitch()`.

## Verification Workflow

Every iteration in this phase must follow:

1. Add or tighten a regression for fallback/guard misuse.
2. Run it and capture the intended failure.
3. Modify exactly one logical layer:
   - node path selection,
   - optimizer result structure,
   - `SE2` optimizer,
   - `R2` optimizer,
   - or `stitch()`.
4. Re-run the targeted regression.
5. Re-run the fixed-case regression.
6. Re-run `test_esv_pipeline_maze`.
7. Re-run `roslaunch esv_planner demo.launch use_map_server:=true` with the fixed start/goal.
8. Commit only if:
   - no online `Hybrid A*` fallback remains,
   - fixed case still succeeds with `ESV`,
   - no safety regression appears.

## First Success Criterion

The first milestone of this phase is:
- the online node no longer contains the `Hybrid A*` publish fallback;
- the fixed case still succeeds with `ESV`;
- the accepted fixed-case trajectory is produced by the continuous optimizer path rather than by a guard trajectory.

Only after that is stable should the next milestone target deeper `R²` and `stitch()` alignment.
