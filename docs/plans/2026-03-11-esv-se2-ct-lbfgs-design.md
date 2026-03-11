# ESV SE2 c/T + LBFGS Design

## Goal

Replace the current waypoint-first `SE2` optimizer path with a paper-aligned optimization entrypoint that uses continuous trajectory coefficients and segment times as the primary optimization variables, solved by `LBFGS++`.

## Context

Current `TrajectoryOptimizer::optimizeSE2Detailed()` still operates on support states / waypoints and then generates a quintic trajectory afterward. This means the system has improved continuous outputs, but it still does not match the paper's core optimizer semantics.

The current codebase now has the prerequisites needed to take the next step:
- `libigl` and `LBFGS++` are vendored and buildable.
- The fixed case and maze regressions are green under the current bounded continuous safety semantics.
- `SE2/R2` primary outputs already use quintic continuous trajectories instead of piecewise-linear support-state chords.

## Design Decision

Use a staged paper-first migration:

1. Keep support states only as initialization.
2. Pack a continuous `SE2` problem into an optimization vector containing:
   - position polynomial coefficients
   - yaw polynomial coefficients
   - log-duration variables for each piece
3. Build the trajectory directly from those variables.
4. Optimize with `LBFGS++` against a continuous objective.
5. Accept trajectories only from this continuous source path for the covered window class.

This is more aggressive than time-only refinement and closer to the paper than another support-state repair layer.

## Variable Model

For a single `SE2` high-risk segment with `N` pieces:

- Position: quintic piece coefficients `c_pos[k]` for each piece
- Yaw: quintic piece coefficients `c_yaw[k]` for each piece
- Time: `tau[k] = log(T[k])`

`T[k] = exp(tau[k])` keeps all piece durations positive.

The initial guess is generated from the current continuous quintic fit over the support-state chain. That fit is used only to seed the optimization.

## Objective

For `SE2`, optimize:

`J = lambda_smooth * J_smooth + lambda_time * J_time + lambda_safety * J_safe + lambda_dynamics * J_dyn`

Where:
- `J_smooth`: integrated high-order derivative cost over all pieces
- `J_time`: total duration
- `J_safe`: continuous collision penalty from the current continuous evaluator
- `J_dyn`: velocity / acceleration / yaw-rate soft penalties

This preserves the current bounded continuous evaluator for now, but moves the optimization variables toward the paper's coefficient-space formulation.

## Migration Boundaries

This stage changes only the `SE2` optimizer path.

In scope:
- `optimizeSE2Detailed()`
- supporting pack/unpack/build/evaluate helpers
- regression tests proving durations are no longer simple geometric allocation

Out of scope for this stage:
- `R2` optimizer migration
- `stitch()` redesign
- replacing the bounded evaluator with a more implicit swept-volume evaluator

## Acceptance Criteria

Required before this stage is considered complete:
- new `SE2` c/T regression is green
- fixed case regression remains green
- maze regression remains green
- accepted trajectories for covered `SE2` windows come from the new continuous c/T path
- piece durations are no longer just `allocateTime(...)` geometry allocation

## Risks

1. Numerical instability in coefficient-space optimization
2. High finite-difference cost if gradients are not yet analytic
3. Regression on medium-window `HIGH` segments if optimization is under-regularized

## Mitigations

1. Start with covered windows only: `SE2` windows up to a bounded size
2. Seed from the existing quintic fit, not from scratch
3. Keep the old path available only as a temporary comparison path during implementation, not as the final accepted source for covered windows
4. Add regression tests first and gate rollout on fixed case + maze
