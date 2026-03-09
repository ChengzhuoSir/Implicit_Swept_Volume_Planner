# ESV Fixed-Case Regression Design

**Date:** 2026-03-09

**Target Case:**
- Map: `esv_planner/maps/maze.pgm`
- Start: `(0.77, 5.53, 0.07)`
- Goal: `(9.15, 3.04, 1.58)`

**Primary Goal:**
Make the current ESV pipeline publish a valid ESV trajectory for the target case instead of falling back to `Hybrid A*`, while preserving the strict collision gate (`min_svsdf >= 0.0`).

## Current Observations

The target case no longer fails silently. The current main branch logs show the true failure mode:

1. `Stage 1` succeeds and returns multiple topological paths.
2. Most candidate paths are classified as a single `LOW` segment.
3. The optimized continuous trajectories repeatedly fail with `min_svsdf = -0.05`.
4. Upgrading `LOW` segments to `SE(2)` via fallback still fails at the same clearance.
5. The node correctly falls back to `Hybrid A*` because the strict gate now rejects all penetrating ESV candidates.

This means the failure is not a roadmap-generation failure. It is a continuous-trajectory feasibility failure in the current optimization/stitching pipeline.

## Root Cause Hypothesis

The most likely root-cause chain for the target case is:

1. The topology path is valid at the discrete waypoint level.
2. `SE2SequenceGenerator` classifies too much of the path as `LOW`, so the first optimization pass is `R²` instead of a stronger `SE(2)` solve.
3. `optimizeR2()` prioritizes smoothness and residual matching, but it does not strongly preserve obstacle clearance.
4. Even when fallback upgrades the path to `SE(2)`, the current optimizer is still waypoint-centric and the fitted continuous trajectory remains too close to the obstacle boundary.
5. `stitch()` can further degrade already-tight trajectories when producing the final polynomial representation.

The repeated `-0.05` clearance strongly suggests a one-cell penetration induced by the continuous representation rather than random numerical noise.

## Why The Recent Changes Exposed This

Recent main-branch fixes changed system behavior in a way that is correct but stricter:

- `accepted_by_current_gate` now requires `min_svsdf >= 0.0`.
- `selectBest()` now filters out penetrating candidates.
- The node now falls back to `Hybrid A*` instead of publishing a bad ESV trajectory.

Before these fixes, the same target case could appear to “work” while still producing a trajectory with negative clearance. The current behavior is a truthful failure report.

## Recommended Approach

Use this target case as a fixed regression case and improve the algorithm iteratively with evidence at each stage.

### Approach A: Fixed-case, layer-by-layer regression loop (recommended)

Add a dedicated regression harness for the exact start/goal pair and log clearance at each stage:
- topology-path sampling clearance
- post-`optimizeR2()` clearance
- post-`optimizeSE2()` clearance
- post-`stitch()` clearance

Then make one targeted change at a time and re-run the same case.

**Why this is recommended:**
- It isolates the real failing layer.
- It prevents parameter thrashing.
- It gives a stable acceptance criterion for each iteration.

### Approach B: Parameter tuning first

Adjust `lambda_safety`, `num_samples`, `knn`, `step_size`, or `max_iterations` first.

**Why this is not recommended first:**
- The current failure pattern is structural, not obviously parametric.
- Parameter-only changes are likely to move the symptom rather than fix the cause.

### Approach C: Large optimizer rewrite immediately

Rework the entire continuous optimizer and stitching process up front.

**Why this is not recommended first:**
- It is the highest-risk option.
- It makes it difficult to identify which change actually fixed the target case.

## Accepted Design

Proceed with Approach A.

The next implementation phase will:

1. Create a dedicated regression test for the fixed target case.
2. Instrument the pipeline to report per-stage clearance and dynamics.
3. Use TDD to identify whether the first hard failure is in:
   - `R²` optimization,
   - `SE(2)` optimization,
   - or final `stitch()`.
4. Implement the smallest algorithm change that makes the target case publish a valid ESV trajectory.
5. After the fixed case turns green, re-run `maze + T/L` regression and `demo.launch` validation.

## Verification Workflow

Every iteration must use the same loop:

1. Reproduce the fixed case in an automated test.
2. Make the test fail for the intended reason.
3. Apply one minimal code change.
4. Re-run the fixed-case test.
5. Re-run `roslaunch esv_planner demo.launch use_map_server:=true` with the same start/goal.
6. Re-run `maze + T/L` regression before accepting the change.
7. Commit only after the fixed case and regressions are understood.

## First-Stage Success Criterion

The first milestone is not “beat A* everywhere”.

The first milestone is:
- for the fixed target case,
- the final published trajectory is an ESV trajectory,
- not a `Hybrid A*` fallback,
- and it satisfies:
  - `min_svsdf >= 0.0`
  - dynamics within limits.

Once that is achieved, the next stage will optimize for quality and superiority over `Hybrid A*`.
