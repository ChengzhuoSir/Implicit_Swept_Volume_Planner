#!/usr/bin/env python3
"""Plot benchmark comparisons and trajectory overlays.

Outputs:
- metrics_comparison.png
- relative_improvement.png
- trajectory_overlay.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


METHOD_ORDER = ["ESV", "SVSDF_like", "RC_ESDF_like"]
METHOD_LABEL = {
    "ESV": "ESV (proposed)",
    "SVSDF_like": "SVSDF-like",
    "RC_ESDF_like": "RC-ESDF-like",
}
METHOD_COLOR = {
    "ESV": "#1f77b4",
    "SVSDF_like": "#ff7f0e",
    "RC_ESDF_like": "#2ca02c",
}


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def load_data(metrics_csv: Path, traj_csv: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    mdf = pd.read_csv(metrics_csv)
    tdf = pd.read_csv(traj_csv)
    return mdf, tdf


def grouped_metrics(mdf: pd.DataFrame) -> pd.DataFrame:
    agg = (
        mdf.groupby(["scenario", "robot", "method"], as_index=False)
        .agg(
            success_rate=("success", "mean"),
            runtime_ms=("runtime_ms", "mean"),
            length_m=("length_m", "mean"),
            duration_s=("duration_s", "mean"),
            min_svsdf=("min_svsdf", "mean"),
            max_acc=("max_acc", "mean"),
        )
        .sort_values(["scenario", "robot", "method"])
    )
    return agg


def plot_metrics_bars(agg: pd.DataFrame, out_png: Path) -> None:
    groups = sorted({(r.scenario, r.robot) for r in agg.itertuples()})
    labels = [f"{s}-{r}" for s, r in groups]
    x = np.arange(len(groups))
    width = 0.25

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    metric_cfg = [
        ("success_rate", "Success Rate", (0.0, 1.05)),
        ("runtime_ms", "Runtime (ms)", None),
        ("length_m", "Trajectory Length (m)", None),
        ("max_acc", "Max Acceleration (m/s^2)", None),
    ]

    for ax, (metric, title, ylim) in zip(axes.flat, metric_cfg):
        for i, method in enumerate(METHOD_ORDER):
            vals = []
            for sc, rb in groups:
                row = agg[(agg["scenario"] == sc) & (agg["robot"] == rb) & (agg["method"] == method)]
                vals.append(float(row[metric].iloc[0]) if not row.empty else np.nan)
            ax.bar(x + (i - 1) * width, vals, width=width, label=METHOD_LABEL[method],
                   color=METHOD_COLOR[method], alpha=0.9)
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=15)
        ax.grid(axis="y", alpha=0.3, linestyle="--")
        if ylim is not None:
            ax.set_ylim(*ylim)

    axes[0, 0].legend(loc="lower right", fontsize=9)
    fig.suptitle("Planner Comparison Metrics", fontsize=14)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def build_relative_df(agg: pd.DataFrame) -> pd.DataFrame:
    rows = []
    keys = sorted({(r.scenario, r.robot) for r in agg.itertuples()})
    for sc, rb in keys:
        esv = agg[(agg["scenario"] == sc) & (agg["robot"] == rb) & (agg["method"] == "ESV")]
        if esv.empty:
            continue
        esv = esv.iloc[0]
        for base in ["SVSDF_like", "RC_ESDF_like"]:
            br = agg[(agg["scenario"] == sc) & (agg["robot"] == rb) & (agg["method"] == base)]
            if br.empty:
                continue
            br = br.iloc[0]
            speedup = (br["runtime_ms"] / esv["runtime_ms"]) if esv["runtime_ms"] > 1e-9 else np.nan
            rows.append(
                {
                    "scenario": sc,
                    "robot": rb,
                    "base": base,
                    "success_delta": esv["success_rate"] - br["success_rate"],
                    "runtime_speedup_x": speedup,
                    "length_reduction_m": br["length_m"] - esv["length_m"],
                }
            )
    return pd.DataFrame(rows)


def plot_relative(rel: pd.DataFrame, out_png: Path) -> None:
    if rel.empty:
        return

    rel = rel.copy()
    rel["group"] = rel["scenario"] + "-" + rel["robot"] + " vs " + rel["base"].map(METHOD_LABEL)
    x = np.arange(len(rel))

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), constrained_layout=True)
    axes[0].bar(x, rel["success_delta"], color="#1f77b4")
    axes[0].set_title("Success Delta (ESV - Baseline)")
    axes[0].axhline(0.0, color="k", linewidth=1)
    axes[0].grid(axis="y", linestyle="--", alpha=0.3)

    axes[1].bar(x, rel["runtime_speedup_x"], color="#ff7f0e")
    axes[1].set_title("Runtime Speedup (Baseline / ESV)")
    axes[1].axhline(1.0, color="k", linewidth=1)
    axes[1].grid(axis="y", linestyle="--", alpha=0.3)

    axes[2].bar(x, rel["length_reduction_m"], color="#2ca02c")
    axes[2].set_title("Length Reduction (Baseline - ESV, m)")
    axes[2].axhline(0.0, color="k", linewidth=1)
    axes[2].grid(axis="y", linestyle="--", alpha=0.3)

    axes[2].set_xticks(x)
    axes[2].set_xticklabels(rel["group"], rotation=28, ha="right")
    axes[0].set_xticks([])
    axes[1].set_xticks([])

    fig.suptitle("ESV Relative Improvements", fontsize=14)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def pick_curve(df: pd.DataFrame, scenario: str, robot: str, method: str) -> pd.DataFrame:
    sub = df[(df["scenario"] == scenario) & (df["robot"] == robot) & (df["method"] == method)]
    if sub.empty:
        return sub
    run = int(sub["run"].min())
    sub = sub[sub["run"] == run]
    if method == "ESV":
        pref = sub[sub["topo_mode"] == "multi"]
        return pref if not pref.empty else sub
    pref = sub[sub["topo_mode"] == "A"]
    return pref if not pref.empty else sub


def plot_trajectory_overlay(tdf: pd.DataFrame, out_png: Path) -> None:
    combos = [("office", "T"), ("office", "L"), ("maze", "T"), ("maze", "L")]
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)

    for ax, (scenario, robot) in zip(axes.flat, combos):
        any_curve = False
        for method in METHOD_ORDER:
            curve = pick_curve(tdf, scenario, robot, method)
            if curve.empty:
                continue
            curve = curve.sort_values("t")
            ax.plot(
                curve["x"].to_numpy(),
                curve["y"].to_numpy(),
                color=METHOD_COLOR[method],
                linewidth=2.0,
                label=METHOD_LABEL[method],
            )
            # Mark start/end for same-origin comparison.
            ax.scatter(curve["x"].iloc[0], curve["y"].iloc[0], color=METHOD_COLOR[method], marker="o", s=28)
            ax.scatter(curve["x"].iloc[-1], curve["y"].iloc[-1], color=METHOD_COLOR[method], marker="x", s=32)
            any_curve = True

        ax.set_title(f"{scenario}-{robot} (same start/goal)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle="--", alpha=0.3)
        if any_curve:
            ax.legend(fontsize=8, loc="best")

    fig.suptitle("Trajectory Overlay by Method", fontsize=14)
    fig.savefig(out_png, dpi=170)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics_csv", default="/home/chengzhuo/workspace/plan/src/esv_planner/benchmark_metrics.csv")
    parser.add_argument("--traj_csv", default="/home/chengzhuo/workspace/plan/src/esv_planner/benchmark_trajectories.csv")
    parser.add_argument("--out_dir", default="/home/chengzhuo/workspace/plan/src/esv_planner/artifacts/plots")
    args = parser.parse_args()

    metrics_csv = Path(args.metrics_csv)
    traj_csv = Path(args.traj_csv)
    out_dir = Path(args.out_dir)
    ensure_dir(out_dir)

    mdf, tdf = load_data(metrics_csv, traj_csv)
    agg = grouped_metrics(mdf)
    rel = build_relative_df(agg)

    metrics_png = out_dir / "metrics_comparison.png"
    relative_png = out_dir / "relative_improvement.png"
    traj_png = out_dir / "trajectory_overlay.png"

    plot_metrics_bars(agg, metrics_png)
    plot_relative(rel, relative_png)
    plot_trajectory_overlay(tdf, traj_png)

    print(f"saved: {metrics_png}")
    print(f"saved: {relative_png}")
    print(f"saved: {traj_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
