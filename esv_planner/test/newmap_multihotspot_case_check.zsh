#!/bin/zsh
set -euo pipefail

cd /home/chengzhuo/workspace/plan
source devel/setup.zsh

export ROS_HOME="${ROS_HOME:-/tmp/roshome_newmap_multihotspot}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_newmap_multihotspot}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

launch_log="$(mktemp /tmp/newmap_multihotspot.XXXXXX.log)"

cleanup() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    wait "$launch_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

roslaunch esv_planner demo.launch use_map_server:=true use_start_goal_params:=false >"$launch_log" 2>&1 &
launch_pid=$!

for _ in {1..60}; do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

for _ in {1..60}; do
  if rostopic list 2>/dev/null | grep -q '/esv_planner/planning_stats'; then
    break
  fi
  sleep 1
done

stats_file="$(mktemp /tmp/newmap_multihotspot_stats.XXXXXX.txt)"
traj_file="$(mktemp /tmp/newmap_multihotspot_traj.XXXXXX.txt)"

timeout 120s rostopic echo -n 1 /esv_planner/planning_stats >"$stats_file" &
stats_pid=$!
timeout 120s rostopic echo -n 1 /esv_planner/trajectory >"$traj_file" &
traj_pid=$!

rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 3.190, y: 45.895, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8040048575, w: 0.5946227284}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}' \
  >/dev/null

sleep 1

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -13.372, y: -17.087, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9764839034, w: 0.2155903209}}}' \
  >/dev/null

wait "$stats_pid" || true
wait "$traj_pid" || true

python3 - "$stats_file" "$traj_file" "$launch_log" "$ROS_LOG_DIR/latest/rosout.log" <<'PY'
import math
import pathlib
import re
import sys

stats_text = pathlib.Path(sys.argv[1]).read_text()
traj_text = pathlib.Path(sys.argv[2]).read_text()
launch_log = pathlib.Path(sys.argv[3]).read_text()
rosout_path = pathlib.Path(sys.argv[4])
rosout_log = rosout_path.read_text() if rosout_path.exists() else ""


def extract_reason(text: str) -> str:
    reason = ""
    for line in text.splitlines():
        if any(
            token in line
            for token in (
                "Planning done:",
                "Planning failed after",
                "Local bottleneck rebuild",
                "Candidate infeasible",
                "Path degraded",
                "Using best degraded candidate",
            )
        ):
            reason = line.split("]: ", 1)[-1]
    return reason


match = re.search(r"data:\s*\[([^\]]+)\]", stats_text)
if not match:
    reason = extract_reason(rosout_log) or extract_reason(launch_log)
    print("FAIL no planning_stats data reason=%s" % reason)
    sys.exit(1)

values = [v.strip() for v in match.group(1).split(",")]
clearance = float(values[3])
support = float(values[5])
poses = traj_text.count("position:")
reason = extract_reason(rosout_log) or extract_reason(launch_log)

if not math.isfinite(clearance) or support <= 0 or poses <= 0:
    print(
        "FAIL clearance=%s support=%.0f poses=%d reason=%s"
        % (values[3], support, poses, reason)
    )
    sys.exit(1)

if clearance <= -0.051:
    print(
        "FAIL degraded clearance=%.3f support=%.0f poses=%d reason=%s"
        % (clearance, support, poses, reason)
    )
    sys.exit(1)

print("PASS clearance=%.3f support=%.0f poses=%d reason=%s" %
      (clearance, support, poses, reason))
PY
