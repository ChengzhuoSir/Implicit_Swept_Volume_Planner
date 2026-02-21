#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
SRC_DIR="$ROOT_DIR/src/esv_planner"
TOOLS_DIR="$SRC_DIR/tools"

METRICS_CSV="${1:-$SRC_DIR/benchmark_metrics.csv}"
TRAJ_CSV="${2:-$SRC_DIR/benchmark_trajectories.csv}"
RUNS="${RUNS:-10}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
# shellcheck disable=SC1091
source "$ROOT_DIR/devel/setup.bash"

python3 "$TOOLS_DIR/generate_maze_map.py"
catkin_make

"$ROOT_DIR/devel/lib/esv_planner/benchmark_planner" \
  --runs "$RUNS" \
  --csv "$METRICS_CSV" \
  --traj_csv "$TRAJ_CSV" \
  --map_office "$SRC_DIR/maps/office.pgm" \
  --map_maze "$SRC_DIR/maps/maze.pgm"

python3 "$TOOLS_DIR/summarize_benchmark.py" "$METRICS_CSV" "$SRC_DIR/benchmark_summary.md"
python3 "$TOOLS_DIR/plot_benchmark.py" --metrics_csv "$METRICS_CSV" --traj_csv "$TRAJ_CSV" --out_dir "$SRC_DIR/artifacts/plots"
