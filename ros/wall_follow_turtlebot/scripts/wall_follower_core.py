"""Wall-following controllers with no ROS dependency, so the same code runs in the
ROS node and in the 2-D simulator.

Scan convention (TurtleBot3 LDS-01): 360 ranges, index 0 straight ahead, angles
increasing counter-clockwise, so index 90 = left, 270 = right. Missing returns
are inf.
"""
from __future__ import annotations

import numpy as np


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, i_limit: float = 1.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.prev_error = None
        self.integral = 0.0

    def __call__(self, error: float, dt: float) -> float:
        if dt <= 0:  # first message, or duplicated timestamps
            dt = 1e-3
        self.integral = float(np.clip(self.integral + error * dt, -self.i_limit, self.i_limit))
        deriv = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


def _sector_min(ranges: np.ndarray, start_deg: int, end_deg: int) -> float:
    idx = np.arange(start_deg, end_deg + 1) % len(ranges)
    r = ranges[idx]
    r = r[np.isfinite(r) & (r > 0)]
    return float(r.min()) if len(r) else np.inf


class FrontAvoidController:
    """v1: the controller from the course project, unchanged in behaviour.

    Drive straight while the +-20 deg front sector is clear. When something is
    closer than the threshold, slow down and let a PD term on the front distance
    turn the robot (always to the right), which makes it slide along walls on its left."""

    def __init__(self, threshold=0.55, speed=0.2, body_offset=0.18, kp=1.0, ki=0.0, kd=0.1):
        self.threshold, self.speed, self.body_offset = threshold, speed, body_offset
        self.pid = PID(kp, ki, kd)

    def step(self, ranges, dt):
        r = np.asarray(ranges, dtype=float) - self.body_offset
        front = min(_sector_min(r, 0, 20), _sector_min(r, 340, 359))
        if front > self.threshold:
            return self.speed, 0.0
        error = self.threshold - front
        return self.speed / 2.0, self.pid(0.0 - error, dt)


class RightWallController:
    """v2: proper right-hand wall following.

    Keeps a set distance to the wall on the right using the right (-90 deg) and
    front-right (-45 deg) beams, turns left on the spot at inside corners, and
    curves right to find the wall again at outside corners or open space."""

    def __init__(self, desired=0.45, speed=0.2, front_stop=0.5, lost=1.0, kp=2.2, kd=1.4, max_w=1.8):
        self.desired, self.speed, self.front_stop, self.lost, self.max_w = desired, speed, front_stop, lost, max_w
        self.pid = PID(kp, 0.0, kd)
        self.state = "find"

    def step(self, ranges, dt):
        r = np.asarray(ranges, dtype=float)
        front = min(_sector_min(r, 0, 15), _sector_min(r, 345, 359))
        right = _sector_min(r, 255, 285)
        front_right = _sector_min(r, 300, 330)
        side = min(right, front_right * np.cos(np.deg2rad(45)) * 1.4)

        if front < self.front_stop:  # inside corner or wall ahead
            self.state = "turn"
            return 0.0, self.max_w * 0.6
        if side > self.lost:  # no wall on the right: arc right to pick one up
            self.state = "find"
            return self.speed * 0.8, -0.6
        self.state = "follow"
        error = self.desired - side  # positive = too close, turn left
        w = float(np.clip(self.pid(error, dt), -self.max_w, self.max_w))
        v = self.speed * (1.0 - min(abs(error) / 0.4, 0.6))
        return v, w


CONTROLLERS = {"v1": FrontAvoidController, "v2": RightWallController}
