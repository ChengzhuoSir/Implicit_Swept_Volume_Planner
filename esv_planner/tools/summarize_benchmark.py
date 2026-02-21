#!/usr/bin/env python3
"""Summarize benchmark CSV and output a compact markdown report."""

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


def fnum(v, nd=3):
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "nan"
    return f"{v:.{nd}f}"


def mean(vals):
    if not vals:
        return float("nan")
    return sum(vals) / len(vals)


def main():
    if len(sys.argv) < 2:
        print("usage: summarize_benchmark.py <metrics.csv> [summary.md]")
        return 1
    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else csv_path.with_name("benchmark_summary.md")
    if not csv_path.exists():
        print(f"missing csv: {csv_path}")
        return 1

    rows = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            r["success"] = int(r["success"])
            for k in ["runtime_ms", "duration_s", "length_m", "min_svsdf", "max_vel", "max_acc"]:
                r[k] = float(r[k])
            rows.append(r)

    grouped = defaultdict(list)
    for r in rows:
        key = (r["scenario"], r["robot"], r["method"])
        grouped[key].append(r)

    lines = []
    lines.append("# Benchmark Summary")
    lines.append("")
    lines.append("| scenario | robot | method | success_rate | runtime_ms | duration_s | length_m | min_svsdf |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|")

    for key in sorted(grouped.keys()):
        vals = grouped[key]
        sr = mean([v["success"] for v in vals])
        rt = mean([v["runtime_ms"] for v in vals])
        du = mean([v["duration_s"] for v in vals])
        ln = mean([v["length_m"] for v in vals])
        ms = mean([v["min_svsdf"] for v in vals])
        lines.append(
            f"| {key[0]} | {key[1]} | {key[2]} | {fnum(sr, 3)} | {fnum(rt, 2)} |"
            f" {fnum(du, 2)} | {fnum(ln, 2)} | {fnum(ms, 3)} |"
        )

    lines.append("")
    lines.append("## ESV vs Baselines")
    lines.append("")
    lines.append("| scenario | robot | compare_to | success_delta | runtime_speedup_x | length_reduction_m |")
    lines.append("|---|---|---|---:|---:|---:|")

    for scenario in sorted(set(r["scenario"] for r in rows)):
        for robot in sorted(set(r["robot"] for r in rows)):
            esv_vals = grouped.get((scenario, robot, "ESV"), [])
            if not esv_vals:
                continue
            esv_sr = mean([v["success"] for v in esv_vals])
            esv_rt = mean([v["runtime_ms"] for v in esv_vals])
            for base in ["SVSDF_like", "RC_ESDF_like"]:
                bvals = grouped.get((scenario, robot, base), [])
                if not bvals:
                    continue
                b_sr = mean([v["success"] for v in bvals])
                b_rt = mean([v["runtime_ms"] for v in bvals])
                b_len = mean([v["length_m"] for v in bvals])
                esv_len = mean([v["length_m"] for v in esv_vals])
                sd = esv_sr - b_sr
                sp = (b_rt / esv_rt) if esv_rt > 1e-9 else float("nan")
                lr = b_len - esv_len
                lines.append(
                    f"| {scenario} | {robot} | {base} | {fnum(sd, 3)} | {fnum(sp, 2)} | {fnum(lr, 2)} |"
                )

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\nSaved: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
