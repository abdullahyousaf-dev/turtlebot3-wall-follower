"""Run both controllers in the maze, print metrics, save a figure and a GIF.

    python sim/run_sim.py               # 3 minutes of simulated time each
    python sim/run_sim.py --seconds 60
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from simulator import ROBOT_RADIUS, clearance, maze, simulate  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
LABELS = {"v1": "v1 - course version (front avoidance)", "v2": "v2 - right-wall PD follower"}


def metrics(run, walls, desired=0.45):
    c = np.array([clearance(p, walls) for p in run.poses[:, :2]])
    rd = run.right_dist
    following = np.isfinite(rd) & (rd < 0.9)
    return {
        "right_wall_error_cm": round(float(np.abs(rd[following] - desired).mean() * 100), 1),
        "collisions": run.collisions,
        "min_clearance_m": round(run.min_clearance - ROBOT_RADIUS, 3),
        "path_length_m": round(run.path_length, 1),
        "maze_coverage": round(run.coverage, 2),
        "time_0.3_to_0.7m_from_wall": round(float(((c > 0.3) & (c < 0.7)).mean()), 2),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180)
    args = ap.parse_args()
    walls = maze()
    runs = {k: simulate(k, seconds=args.seconds) for k in LABELS}
    table = {k: metrics(r, walls) for k, r in runs.items()}

    keys = list(next(iter(table.values())))
    print(f"{'metric':<30}" + "".join(f"{k:>10}" for k in table))
    for m in keys:
        print(f"{m:<30}" + "".join(f"{table[k][m]:>10}" for k in table))

    media = ROOT / "media"
    media.mkdir(exist_ok=True)
    (media / "sim_metrics.json").write_text(json.dumps(table, indent=2))

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    for ax, (k, r) in zip(axes, runs.items()):
        for a, b in walls:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="0.2", lw=3)
        sc = ax.scatter(r.poses[:, 0], r.poses[:, 1], c=np.arange(len(r.poses)) * 0.1, s=2, cmap="viridis")
        ax.plot(*r.poses[0, :2], "go", ms=8, label="start")
        ax.set_aspect("equal")
        ax.set_title(f"{LABELS[k]}\ncoverage {table[k]['maze_coverage']:.0%}, collisions {r.collisions}")
        ax.set_xticks([]), ax.set_yticks([])
        ax.legend(loc="upper right", fontsize=8)
    fig.colorbar(sc, ax=axes, label="time (s)", shrink=0.8)
    fig.savefig(media / "sim_paths.png", dpi=120, bbox_inches="tight")
    print("\nSaved media/sim_paths.png and media/sim_metrics.json")


if __name__ == "__main__":
    main()
