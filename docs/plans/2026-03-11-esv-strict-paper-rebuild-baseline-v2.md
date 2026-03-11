# ESV Strict Paper Rebuild Baseline v2

Recorded on 2026-03-11 in `/home/chengzhuo/workspace/plan/src` before strict paper-first core replacement.

## Fixed Case
- Start: `(1.06, 7.55, -1.57)`
- Goal: `(8.97, 3.63, 1.52)`
- `topo_paths=1`
- `accepted_candidates=1`
- `best_topo_clearance=0.15`
- `best_sequence_chain_clearance=0.25`
- `best_segment_optimized_clearance=0.25`
- `best_stitched_clearance=0.1`
- `best_final_accepted_clearance=0.1`
- `used_guard=0`
- `continuous_source_ok=1`
- Notable structure: `11` segments total, `5` `HIGH` segments reached `0.1~0.1207` in `SE2`.

## Second Case
- Start: `(0.72, 5.31, 0.29)`
- Goal: `(9.14, 5.74, 1.64)`
- `topo_paths=1`
- `accepted_candidates=1`
- `best_topo_clearance=0.1`
- `best_sequence_chain_clearance=0.1`
- `best_segment_optimized_clearance=0.1207`
- `best_stitched_clearance=0.1`
- `best_final_accepted_clearance=0.1`
- `used_guard=0`
- `continuous_source_ok=1`
- Shape metrics are weaker than fixed case:
  - `selected_length_ratio=1.0068`
  - `selected_max_bulge=0.188986`
  - `selected_heading_oscillations=17`
- This case is suitable as the second acceptance harness because it stresses multiple `HIGH` windows while still reaching a valid online result.

## Maze Regression
### T-shape
- `topo_paths=2`
- `accepted_candidates=2`
- `accepted_high_risk_segments=3`
- `final_traj_valid=1`
- Both accepted candidates reached `stitched_clearance >= 0.1`

### L-shape
- `topo_paths=3`
- `accepted_candidates=2`
- `accepted_high_risk_segments=2`
- `final_traj_valid=1`
- Accepted paths reached `stitched_clearance >= 0.1`

## Observations
- Current system is functionally usable under the bounded continuous evaluator.
- It is still not paper-faithful in three core areas:
  1. continuous collision is bounded adaptive sampling, not implicit swept-volume CCA
  2. optimizer is still support-state / waypoint-first, not `c,T` MINCO joint optimization
  3. front-end remains roadmap + shortcut driven, not fully body-frame-SDF-dominated
- The baseline is therefore stable enough to start the strict paper-first core rebuild without first fixing general functionality.
