"""Unit tests for hazard_memory.HazardMemory. No rclpy, no simulator --
run with `pytest src/bot_script/test/` or `colcon test --packages-select bot_script`.
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_script.hazard_memory import HazardMemory  # noqa: E402


def test_empty_memory_has_no_repulsion():
    mem = HazardMemory()
    assert mem.repulsion(0.0, 0.0, radius=1.0) == (0.0, 0.0)
    assert len(mem) == 0


def test_far_away_hazard_has_no_effect():
    mem = HazardMemory()
    mem.record(10.0, 10.0)
    rx, ry = mem.repulsion(0.0, 0.0, radius=1.0)
    assert rx == 0.0 and ry == 0.0


def test_nearby_hazard_pushes_away():
    mem = HazardMemory()
    mem.record(1.0, 0.0)  # hazard directly ahead (+x) of the query point
    rx, ry = mem.repulsion(0.0, 0.0, radius=2.0)
    assert rx < 0.0   # push back toward -x, away from the hazard
    assert abs(ry) < 1e-9


def test_closer_hazard_pushes_harder():
    mem = HazardMemory()
    mem.record(0.5, 0.0)
    near_rx, _ = mem.repulsion(0.0, 0.0, radius=2.0)
    mem2 = HazardMemory()
    mem2.record(1.9, 0.0)
    far_rx, _ = mem2.repulsion(0.0, 0.0, radius=2.0)
    assert abs(near_rx) > abs(far_rx)


def test_multiple_hazards_combine():
    mem = HazardMemory()
    mem.record(1.0, 0.0)
    mem.record(-1.0, 0.0)
    rx, ry = mem.repulsion(0.0, 0.0, radius=2.0)
    # symmetric hazards on both sides should roughly cancel out
    assert abs(rx) < 1e-9
    assert abs(ry) < 1e-9


def test_max_points_evicts_oldest():
    mem = HazardMemory(max_points=3)
    for i in range(5):
        mem.record(float(i), 0.0)
    assert len(mem) == 3


def test_zero_radius_never_triggers():
    mem = HazardMemory()
    mem.record(0.0, 0.0)
    assert mem.repulsion(0.0, 0.0, radius=0.0) == (0.0, 0.0)


def test_iteration_yields_recorded_points():
    mem = HazardMemory()
    mem.record(1.0, 2.0)
    mem.record(3.0, 4.0)
    assert list(mem) == [(1.0, 2.0), (3.0, 4.0)]
