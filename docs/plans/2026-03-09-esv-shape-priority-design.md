# ESV Shape-Priority Design

**Date:** 2026-03-09

**Primary Goal:**
Improve the accepted fixed-case ESV trajectory so that it remains paper-aligned, collision-free, and dynamically feasible while removing obvious oscillation, unnecessary bulges, and avoidable path inflation.

**Fixed Acceptance Case:**
- Map: `esv_planner/maps/maze.pgm`
- Start: `(1.06, 7.55, -1.57)`
- Goal: `(8.97, 3.63, 1.52)`

**Inherited Hard Constraints:**
- The final trajectory must be produced by `ESV`, not by `Hybrid A*` fallback.
- The final trajectory must satisfy `min_svsdf >= 0.0`.
- The final trajectory must remain dynamically feasible.
- All work must happen in the main workspace: `/home/chengzhuo/workspace/plan/src`.

## Why Shape Is The Next Paper Gap

The current branch now meets the safety-first baseline for the fixed case: an `ESV` candidate is accepted without falling back to `Hybrid A*`. However, the accepted trajectory is still not paper-like in shape.

The current implementation can still:
- introduce unnecessary bends when converting waypoint chains into polynomial pieces;
- create visible bulges during final global stitching;
- prefer trajectories that are safe but visually over-conservative or inflated.

This is a paper gap because the paper's benefit is not only feasibility. It also relies on producing smooth, intentional trajectories through better sequence semantics and continuous optimization. A safety-only trajectory that is visibly wavy is still below the paper target.

## Root-Cause Assessment

The main sources of the current shape defects are downstream of topology generation:

1. **`optimizeR2()` and `optimizeSE2()` still optimize waypoints first, then fit polynomials.**
   This can preserve safety but distort shape when the continuous fit is reconstructed from a discretized chain.

2. **`stitch()` can still deform already-feasible segment trajectories.**
   Even though exact concatenation is preferred when feasible, fallback re-fitting still allows visible bulges if the reconstruction is too coarse or the objective is not shape-aware enough.

3. **`selectBest()` still does not optimize for paper-like shape strongly enough.**
   It currently filters by feasibility and prefers large clearance, but it does not explicitly penalize waviness, lateral detours, or unnecessary length growth.

4. **The regression harness does not yet measure shape.**
   The fixed-case regression currently proves safety, but it cannot fail on an accepted trajectory that is safe yet visibly worse than the intended motion sequence.

## Accepted Direction

Use a **shape-regression-first** approach.

This means:
- first make shape measurable and testable on the fixed case;
- then reduce unnecessary deformation in the final trajectory construction;
- only then tune or reshape optimizer objectives if the metrics still indicate paper-gap behavior.

This keeps the work paper-aligned: the target is not arbitrary visual smoothing, but preserving the motion-sequence geometry the optimizer has already validated.

## Candidate Approaches

### Approach A: Shape Regression + Minimal-Deformation Back End (recommended)

1. Add fixed-case shape metrics to the regression harness.
2. Make `stitch()` preserve segment geometry more aggressively.
3. Make `selectBest()` choose among safe candidates using a combined shape-aware objective.

**Why recommend it:**
- It directly targets the two layers now responsible for visible shape degradation.
- It preserves the established safety baseline.
- It matches the paper direction better than ad hoc smoothing.

### Approach B: Selection-Only Tuning

Only modify final candidate ranking so safer and straighter trajectories win.

**Why not enough:**
- If all candidates are already deformed in `stitch()`, ranking alone cannot recover a good shape.

### Approach C: Immediate Optimizer Rewrite

Directly rework `optimizeR2()` and `optimizeSE2()` objectives first.

**Why not first:**
- This is the most invasive option.
- It risks breaking the current fixed-case safety baseline before shape is even measurable.

## Shape Metrics To Add

The fixed-case regression will add deterministic metrics for accepted trajectories:

1. **`length_ratio`**
   - Definition: final accepted trajectory arc length divided by the arc length of the accepted segment-chain reference.
   - Purpose: catch inflated or needlessly long trajectories.

2. **`heading_oscillation_count`**
   - Definition: count of meaningful signed heading-curvature reversals along a sampled accepted trajectory, after ignoring very small noise.
   - Purpose: catch visible wavy motion rather than purposeful turning.

3. **`max_lateral_bulge`**
   - Definition: maximum perpendicular deviation of the accepted trajectory from the accepted segment-chain reference polyline.
   - Purpose: catch stitch-induced bowing or polynomial detours.

These metrics are intentionally simple, deterministic, and tied to the accepted segment-chain geometry rather than to `Hybrid A*`.

## Design For The First Shape Iteration

### Part 1: Add Shape Regression

Extend `test_esv_fixed_case_regression.cpp` so it records, for the best accepted candidate:
- accepted segment-chain length;
- final trajectory sampled length;
- heading oscillation count;
- maximum lateral bulge.

The first test iteration should fail on metrics only after confirming the trajectory is still accepted and safe. Safety remains the first gate.

### Part 2: Reduce Stitch Deformation

Tighten `trajectory_optimizer.cpp` so that `stitch()`:
- prefers exact concatenation whenever already feasible;
- when reconstruction is necessary, samples the accepted segment trajectories densely enough to avoid bulge-prone re-fitting;
- avoids coarse spacing choices that inflate the path in narrow passages.

### Part 3: Shape-Aware Final Selection

Adjust final selection so that among safe, dynamically feasible candidates, preference is given to:
- lower bulge,
- fewer oscillations,
- lower length inflation,
- and only then secondary smoothness/timing costs.

This keeps safety as a hard gate while moving quality closer to the paper's intended result.

## Verification Workflow

Every shape iteration must follow this loop:

1. Tighten the fixed-case regression with one explicit shape expectation.
2. Run the test and observe the failure.
3. Modify the smallest relevant back-end logic.
4. Re-run the fixed-case regression.
5. Re-run `test_esv_pipeline_maze` to ensure accepted candidate quality does not regress elsewhere.
6. Re-run `roslaunch esv_planner demo.launch use_map_server:=true` on the fixed case.
7. Commit only if:
   - `ESV` still succeeds,
   - no fallback is used,
   - `min_svsdf >= 0.0`,
   - shape metrics improve or meet the threshold.

## First Iteration Success Criteria

The first shape milestone is:
- fixed-case `ESV` acceptance remains intact;
- accepted trajectory shape metrics are now visible in regression output;
- at least one shape defect is reduced through back-end trajectory construction rather than through case-specific logic.

This does not yet require runtime improvements. Runtime remains a later phase.
