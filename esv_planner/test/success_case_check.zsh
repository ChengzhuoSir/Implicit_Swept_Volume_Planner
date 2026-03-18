#!/bin/zsh
set -euo pipefail

cd /home/chengzhuo/workspace/plan
source /home/chengzhuo/workspace/plan/devel/setup.zsh

export ROS_HOME="${ROS_HOME:-/tmp/roshome_success_case_check}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_success_case_check}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

launch_log="$(mktemp /tmp/success_case_check.XXXXXX.log)"

cleanup() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    wait "$launch_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

roslaunch esv_planner demo.launch use_map_server:=true >"$launch_log" 2>&1 &
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

stats_file="$(mktemp /tmp/success_case_stats.XXXXXX.txt)"
traj_file="$(mktemp /tmp/success_case_traj.XXXXXX.txt)"

timeout 30s rostopic echo -n 1 /esv_planner/planning_stats >"$stats_file" &
stats_pid=$!
timeout 30s rostopic echo -n 1 /esv_planner/trajectory >"$traj_file" &
traj_pid=$!

rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 3.14, y: 45.44, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.843214154, w: 0.5375870763}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}' \
  >/dev/null

sleep 1

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -0.52, y: 15.15, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.2474039593, w: 0.9689124217}}}' \
  >/dev/null

wait "$stats_pid" || true
wait "$traj_pid" || true

python3 - "$stats_file" "$traj_file" "$launch_log" <<'PY'
import math
import pathlib
import re
import sys

stats_text = pathlib.Path(sys.argv[1]).read_text()
traj_text = pathlib.Path(sys.argv[2]).read_text()
launch_log = pathlib.Path(sys.argv[3]).read_text().splitlines()

match = re.search(r"data:\s*\[([^\]]+)\]", stats_text)
if not match:
    print("FAIL no planning_stats data")
    raise SystemExit(1)

values = [v.strip() for v in match.group(1).split(",")]
clearance = float(values[3])
support = float(values[5])
poses = traj_text.count("position:")

reason = ""
for line in launch_log:
    if "Planning done:" in line or "Planning failed after" in line:
        reason = line.split("]: ", 1)[-1]

if not math.isfinite(clearance) or support <= 0 or poses <= 0:
    print(
        "FAIL clearance=%s support=%.0f poses=%d reason=%s"
        % (values[3], support, poses, reason)
    )
    raise SystemExit(1)

print("PASS clearance=%.3f support=%.0f poses=%d reason=%s" %
      (clearance, support, poses, reason))
PY
