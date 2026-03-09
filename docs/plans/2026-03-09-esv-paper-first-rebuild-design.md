# ESV Paper-First Rebuild Design

**Date:** 2026-03-09

**Primary Goal:**
Rebuild the current ESV pipeline in the main workspace so that subsequent implementation decisions are driven by the paper rather than by the existing prototype, using a fixed regression case as the acceptance target.

**Fixed Acceptance Case:**
- Map: `esv_planner/maps/maze.pgm`
- Start: `(1.06, 7.55, -1.57)`
- Goal: `(8.97, 3.63, 1.52)`

**Hard Constraints:**
- The final trajectory must be produced by `ESV`, not by `Hybrid A*` fallback.
- The final trajectory must be collision-free with `min_svsdf >= 0.0`.
- The final trajectory must not exhibit obvious oscillation or unnecessary waviness.
- Planning time must improve in later stages, but only after safety and shape are stable.
- All work must happen in the main workspace: `/home/chengzhuo/workspace/plan/src`.

## Why A Paper-First Rebuild Is Required

The current branch is no longer failing because of a single bug. The deeper issue is that the implemented pipeline is still a paper-inspired prototype rather than a faithful reproduction of the paper's algorithmic structure.

The largest remaining gaps are:

1. **Front-end topology generation is still PRM-centric rather than paper-centric.**
   The current topology planner is based on Halton PRM, Dijkstra, and geometric shortcutting. This gives a useful engineering scaffold but it is not yet the paper's intended front-end semantics.

2. **`LOW/HIGH` sequence generation is still an engineering approximation.**
   The current `SE2SequenceGenerator` is closer to the paper than earlier versions, but it still relies on repair and compaction logic that is not the paper's clean semantic definition.

3. **Continuous collision semantics are not yet unified.**
   Different stages still reason about feasibility through slightly different approximations. This creates cases where early stages accept a motion chain that the final trajectory later rejects.

4. **The optimizer is still waypoint-first, not trajectory-variable-first.**
   The current `TrajectoryOptimizer` mutates waypoints and then fits continuous polynomials. This is precisely why safety, shape, and final stitched behavior can drift away from the intended motion sequence.

5. **Final candidate selection is not aligned with the paper's quality objective.**
   Safety is now enforced more strictly than before, but final trajectory choice still does not fully reflect the paper's end-to-end objective.

## Accepted Direction

Use a **paper-first rebuild strategy**.

This means:
- the paper is treated as the primary algorithm specification;
- the existing code is treated as an implementation scaffold only;
- whenever the current code and the paper disagree, implementation should move toward the paper rather than preserving the current engineering shortcut.

The fixed maze case is used as an acceptance harness, not as the source of algorithm design.

## Paper-First Working Rules

1. **Paper semantics override current convenience.**
   If an existing shortcut conflicts with the paper, the shortcut should be removed or isolated.

2. **Every stage must explicitly identify and reduce a paper gap.**
   Each implementation phase must state:
   - the paper's method,
   - the current implementation's behavior,
   - the exact gap being closed.

3. **The fixed case is for regression, not special-casing.**
   The fixed case must never be solved through ad hoc case-specific logic.

4. **Main workspace only.**
   No additional worktree is used. Git history in the main workspace is the source of truth.

5. **Controlled delegation.**
   A subagent may be used for tightly scoped coding or test tasks, but the main agent retains responsibility for:
   - paper comparison,
   - task decomposition,
   - result interpretation,
   - acceptance decisions,
   - commit boundaries.

## Staged Acceptance Order

The implementation order is intentionally strict:

### Stage A: Safety First

Success means:
- the fixed case produces an `ESV` trajectory,
- no `Hybrid A*` fallback is used,
- `min_svsdf >= 0.0`,
- the trajectory remains dynamically feasible.

This stage is the first hard gate. No shape or timing work is accepted if safety is unstable.

### Stage B: Shape Quality

Success means:
- the fixed-case trajectory no longer exhibits obvious oscillation,
- turns are purposeful rather than wavy,
- final stitching no longer reintroduces avoidable bulges or detours,
- safety from Stage A remains intact.

### Stage C: Runtime

Success means:
- the fixed case remains safe and visually stable,
- planning time is reduced by eliminating unnecessary candidate work and repeated revalidation,
- runtime improvements do not come from weakening safety or shape requirements.

## Four Algorithm-Rebuild Phases

### Phase 1: Front-End Paper Alignment

Target modules:
- `esv_planner/src/topology_planner.cpp`
- `esv_planner/include/esv_planner/common.h`

Intent:
- move shortcut and waypoint generation closer to the paper's robot-shape-aware front end;
- ensure topology waypoints are meaningful for downstream orientation reasoning rather than merely carrying tangent yaw placeholders.

### Phase 2: Sequence Generation Paper Alignment

Target modules:
- `esv_planner/src/se2_sequence_generator.cpp`
- `esv_planner/include/esv_planner/se2_sequence_generator.h`

Intent:
- make `SafeYaw`, recursive `SegAdjust`, and `LOW/HIGH` decisions match the paper's semantics more directly;
- eliminate mismatches between local passage reasoning and final continuous feasibility.

### Phase 3: Continuous Collision Semantics Alignment

Target modules:
- `esv_planner/src/svsdf_evaluator.cpp`
- any tests that validate segment or trajectory feasibility

Intent:
- unify the continuous feasibility semantics used by:
  - sequence generation,
  - optimizer acceptance,
  - final trajectory publication.

### Phase 4: Back-End Optimization Alignment

Target modules:
- `esv_planner/src/trajectory_optimizer.cpp`
- `esv_planner/include/esv_planner/trajectory_optimizer.h`

Intent:
- reduce dependence on waypoint-first optimization plus post-fit deformation;
- move the implementation closer to the paper's continuous optimization logic;
- ensure final selection prefers trajectories that satisfy the paper's quality goals rather than merely maximizing clearance.

## Main-Agent / Subagent Workflow

### Main Agent Responsibilities
- compare implementation against the paper;
- define the current phase boundary;
- decide which paper gap is being reduced this round;
- review diffs and verification evidence;
- decide whether the phase is accepted or iteration continues;
- create commits.

### Subagent Responsibilities
- implement a narrowly scoped task in one module or one test area;
- run only the requested verification commands;
- return a concise summary of changes and outputs;
- avoid broad refactors or design changes not explicitly assigned.

This structure keeps context stable while allowing limited parallel execution without losing algorithmic discipline.

## Verification Workflow

Every iteration must follow this loop:

1. Add or tighten a regression test for the current paper gap.
2. Run the test and observe the failure.
3. Modify one module or one tightly related set of files.
4. Re-run the targeted regression.
5. Re-run the fixed-case offline regression.
6. Re-run `roslaunch esv_planner demo.launch use_map_server:=true` with the fixed start/goal.
7. Accept and commit only if the phase objective is met without violating previous hard constraints.

## First Phase To Execute

Start with **Stage A / Phase 2+4 overlap**, because the immediate hard failure is still safety at the fixed case:
- `LOW/HIGH` classification must match continuous safety semantics more strictly;
- `stitch()` must stop invalidating otherwise acceptable segment outputs;
- the fixed case must end with an accepted `ESV` candidate and no fallback.

This first phase does not yet attempt to optimize for runtime. It only aims to establish a correct paper-first safety baseline.
