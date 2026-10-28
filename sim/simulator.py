"""Minimal 2-D TurtleBot3 simulator: wall segments, a 360-beam LiDAR, unicycle motion.

Lets the wall-following controllers run and be measured on a laptop without ROS
or Gazebo. The maze is modelled on the Gazebo world used in the project videos.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros" / "wall_follow_turtlebot" / "scripts"))
from wall_follower_core import CONTROLLERS  # noqa: E402

ROBOT_RADIUS = 0.105  # TurtleBot3 Burger
LIDAR_MIN, LIDAR_MAX = 0.12, 3.5  # LDS-01


def maze() -> np.ndarray:
    """Wall segments (M, 2, 2) in metres: 8 x 5 m outer box plus inner walls."""
    w = [
        ((0, 0), (8, 0)), ((8, 0), (8, 5)), ((8, 5), (0, 5)), ((0, 5), (0, 0)),
        ((1.7, 5), (1.7, 1.5)), ((1.0, 1.5), (3.4, 1.5)),
        ((3.3, 1.5), (3.3, 4.0)), ((3.3, 2.6), (4.1, 2.6)),
        ((5.4, 5), (5.4, 2.0)),
        ((6.3, 3.9), (8, 3.9)),
        ((6.2, 0.4), (6.2, 2.4)), ((6.2, 2.4), (6.9, 2.4)),
    ]
    return np.array(w, dtype=float)


def lidar(pose, walls: np.ndarray, n: int = 360, noise: float = 0.01, rng=None) -> np.ndarray:
    """Ray-cast n beams from the robot. Index 0 = straight ahead, counter-clockwise."""
    x, y, th = pose
    ang = th + np.deg2rad(np.arange(n) * 360.0 / n)
    d = np.c_[np.cos(ang), np.sin(ang)]  # (n, 2)
    a, e = walls[:, 0], walls[:, 1] - walls[:, 0]  # (m, 2)
    ap = a - [x, y]
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (ap[None, :, 0] * e[None, :, 1] - ap[None, :, 1] * e[None, :, 0]) / denom
        s = (ap[None, :, 0] * d[:, None, 1] - ap[None, :, 1] * d[:, None, 0]) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 0) & (s >= 0) & (s <= 1)
    r = np.where(hit, t, np.inf).min(axis=1)
    if rng is not None:
        r = r + rng.normal(0, noise, n)
    return np.where((r < LIDAR_MIN) | (r > LIDAR_MAX), np.inf, r)


def clearance(p, walls: np.ndarray) -> float:
    """Distance from point p to the nearest wall."""
    a, b = walls[:, 0], walls[:, 1]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / (ab * ab).sum(1), 0, 1)
    return float(np.linalg.norm(a + t[:, None] * ab - p, axis=1).min())


@dataclass
class Run:
    poses: np.ndarray
    collisions: int
    min_clearance: float
    path_length: float
    coverage: float
    right_dist: np.ndarray = field(default_factory=lambda: np.array([]))


def simulate(controller: str = "v1", seconds: float = 180.0, dt: float = 0.1,
             start=(0.5, 0.5, np.pi / 2), seed: int = 0, walls: np.ndarray | None = None) -> Run:
    walls = maze() if walls is None else walls
    rng = np.random.default_rng(seed)
    ctrl = CONTROLLERS[controller]()
    pose = np.array(start, dtype=float)
    poses, right, collisions, in_contact, min_c = [pose.copy()], [], 0, False, np.inf
    for _ in range(int(seconds / dt)):
        scan = lidar(pose, walls, rng=rng)
        v, w = ctrl.step(scan, dt)
        right.append(np.nanmin(np.where(np.isfinite(scan[260:281]), scan[260:281], np.nan))
                     if np.isfinite(scan[260:281]).any() else np.inf)
        nxt = pose + [v * np.cos(pose[2]) * dt, v * np.sin(pose[2]) * dt, w * dt]
        c = clearance(nxt[:2], walls)
        min_c = min(min_c, c)
        if c < ROBOT_RADIUS:  # bumped: count it once, rotate in place only
            if not in_contact:
                collisions += 1
            in_contact = True
            nxt[:2] = pose[:2]
        else:
            in_contact = False
        pose = nxt
        poses.append(pose.copy())
    P = np.array(poses)
    cells = {(int(px // 0.5), int(py // 0.5)) for px, py in P[:, :2]}
    return Run(P, collisions, min_c, float(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum()),
               len(cells) / (16 * 10), np.array(right))
