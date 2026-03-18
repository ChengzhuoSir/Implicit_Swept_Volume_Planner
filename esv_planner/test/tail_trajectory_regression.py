#!/usr/bin/env python3

import csv
import math
import sys


def load_rows(path):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(
                (
                    float(row["t"]),
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw"]),
                )
            )
    return rows


def tail_detour_ratio(rows, tail_fraction=0.4):
    if len(rows) < 4:
        raise ValueError("need at least four trajectory samples")

    start_index = max(1, int(len(rows) * (1.0 - tail_fraction)))
    tail = rows[start_index:]
    path_length = 0.0
    for i in range(1, len(tail)):
        path_length += math.hypot(tail[i][1] - tail[i - 1][1], tail[i][2] - tail[i - 1][2])

    chord = math.hypot(tail[-1][1] - tail[0][1], tail[-1][2] - tail[0][2])
    if chord < 1e-9:
        return float("inf")
    return path_length / chord


def main():
    if len(sys.argv) != 2:
        print("usage: tail_trajectory_regression.py <traj_data.csv>", file=sys.stderr)
        return 2

    rows = load_rows(sys.argv[1])
    ratio = tail_detour_ratio(rows)
    threshold = 1.45
    if ratio > threshold:
        print(
            "FAIL tail detour ratio %.3f exceeds threshold %.3f" % (ratio, threshold),
            file=sys.stderr,
        )
        return 1

    print("PASS tail detour ratio %.3f within threshold %.3f" % (ratio, threshold))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
