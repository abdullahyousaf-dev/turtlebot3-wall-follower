import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "ros" / "wall_follow_turtlebot" / "scripts"))

from simulator import clearance, lidar, maze, simulate  # noqa: E402
from wall_follower_core import PID, FrontAvoidController, RightWallController  # noqa: E402

BOX = np.array([((0, 0), (4, 0)), ((4, 0), (4, 4)), ((4, 4), (0, 4)), ((0, 4), (0, 0))], float)


def test_lidar_reads_distance_to_box_walls():
    r = lidar((1.0, 2.0, 0.0), BOX)
    assert abs(r[0] - 3.0) < 1e-9  # ahead (+x)
    assert abs(r[90] - 2.0) < 1e-9  # left (+y)
    assert abs(r[180] - 1.0) < 1e-9  # behind
    assert abs(r[270] - 2.0) < 1e-9  # right


def test_clearance():
    assert abs(clearance(np.array([1.0, 2.0]), BOX) - 1.0) < 1e-12


def test_pid_first_call_has_no_divide_by_zero():
    pid = PID(1.0, 0.0, 0.5)
    assert np.isfinite(pid(0.3, 0.0))


def test_v1_drives_straight_when_clear_and_turns_right_near_wall():
    c = FrontAvoidController()
    clear = np.full(360, 3.0)
    assert c.step(clear, 0.1) == (0.2, 0.0)
    near = clear.copy()
    near[:10] = 0.5
    v, w = c.step(near, 0.1)
    assert v == 0.1 and w < 0


def test_v2_turns_away_when_too_close_to_right_wall():
    c = RightWallController()
    r = np.full(360, 3.0)
    r[250:335] = 0.25  # wall close on the right
    _, w = c.step(r, 0.1)
    assert w > 0


def test_both_controllers_run_collision_free_in_maze():
    for name in ("v1", "v2"):
        run = simulate(name, seconds=60, walls=maze())
        assert run.collisions == 0, name
