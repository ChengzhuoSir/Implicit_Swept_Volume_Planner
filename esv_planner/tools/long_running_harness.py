#!/usr/bin/env python3
"""Controller harness for long-running agent workflows.

Inspired by staged harness patterns:
- explicit task list with role ownership
- durable progress logging
- command execution with per-task artifacts
- round-by-round validation gates
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
from pathlib import Path


def run_cmd(cmd: str, cwd: Path, log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as lf:
        lf.write(f"$ {cmd}\n\n")
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            shell=True,
            stdout=lf,
            stderr=subprocess.STDOUT,
            check=False,
        )
        lf.write(f"\n[exit_code] {proc.returncode}\n")
        return proc.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="/home/chengzhuo/workspace/plan")
    parser.add_argument("--runs", type=int, default=10)
    args = parser.parse_args()

    ws = Path(args.workspace).resolve()
    src = ws / "src"
    esv = src / "esv_planner"
    artifacts = esv / "artifacts" / "harness_logs"
    state_path = src / "claude-progress.json"
    feature_path = src / "feature-list.json"

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    tasks = [
        {
            "id": "analysis_controller_round",
            "role": "controller",
            "cmd": "catkin_make",
            "desc": "build before delegation",
        },
        {
            "id": "coding_subagent_round",
            "role": "coding_subagent",
            "cmd": "python3 src/esv_planner/tools/generate_maze_map.py",
            "desc": "prepare benchmark assets",
        },
        {
            "id": "testing_subagent_round",
            "role": "testing_subagent",
            "cmd": (
                f"RUNS={args.runs} bash src/esv_planner/tools/run_benchmark.sh "
                "src/esv_planner/benchmark_metrics.csv "
                "src/esv_planner/benchmark_trajectories.csv"
            ),
            "desc": "execute benchmark and summarize",
        },
    ]

    state = {
        "timestamp": stamp,
        "tasks": [],
        "status": "running",
    }

    for task in tasks:
        log_path = artifacts / f"{stamp}_{task['id']}.log"
        code = run_cmd(task["cmd"], ws, log_path)
        state["tasks"].append(
            {
                "id": task["id"],
                "role": task["role"],
                "desc": task["desc"],
                "cmd": task["cmd"],
                "log": str(log_path),
                "exit_code": code,
                "status": "done" if code == 0 else "failed",
            }
        )
        if code != 0:
            state["status"] = "failed"
            break

    if state["status"] != "failed":
        state["status"] = "done"

    artifacts.mkdir(parents=True, exist_ok=True)
    run_state = artifacts / f"{stamp}_state.json"
    run_state.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")

    # Update lightweight feature/progress signals expected by long-running harness.
    if feature_path.exists():
        try:
            features = json.loads(feature_path.read_text(encoding="utf-8"))
        except Exception:
            features = {}
    else:
        features = {}
    features["last_harness_run"] = {
        "timestamp": stamp,
        "status": state["status"],
        "state_file": str(run_state),
    }
    feature_path.write_text(json.dumps(features, ensure_ascii=False, indent=2), encoding="utf-8")

    if state_path.exists():
        try:
            progress = json.loads(state_path.read_text(encoding="utf-8"))
        except Exception:
            progress = {}
        progress["last_harness_run"] = {
            "timestamp": stamp,
            "status": state["status"],
            "artifact_state": str(run_state),
        }
        state_path.write_text(json.dumps(progress, ensure_ascii=False, indent=2), encoding="utf-8")

    print(json.dumps(state, ensure_ascii=False, indent=2))
    print(f"state: {run_state}")
    return 0 if state["status"] == "done" else 1


if __name__ == "__main__":
    raise SystemExit(main())
