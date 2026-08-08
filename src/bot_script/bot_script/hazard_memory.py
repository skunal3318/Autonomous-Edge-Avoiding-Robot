"""Persistent memory of world-frame locations where the robot has had to
AVOID or RECOVER, used to steer future exploration and turn-direction
choices away from spots it has already learned are dangerous.

Without this, the state machine in edge_avoider.py treats every edge
encounter as brand new: a fixed reverse/turn maneuver can point the robot
right back at the same edge it just backed away from, especially once its
narrow forward-facing LIDAR cone loses sight of that specific patch of
ground. Recording *where* trouble happened and biasing steering away from
it is what turns "react and forget" into "react and remember."

Pure Python, no ROS dependency, so it's unit-tested directly
(see test/test_hazard_memory.py) without spinning up rclpy.
"""
import math
from collections import deque
from typing import Iterator, Tuple


class HazardMemory:
    def __init__(self, max_points: int = 200):
        self._points = deque(maxlen=max_points)

    def record(self, x: float, y: float) -> None:
        self._points.append((x, y))

    def repulsion(self, x: float, y: float, radius: float) -> Tuple[float, float]:
        """World-frame vector pointing away from every remembered hazard
        within `radius` of (x, y), weighted by inverse distance so closer
        hazards push harder. (0.0, 0.0) if none are within range."""
        if radius <= 0:
            return 0.0, 0.0
        rx = ry = 0.0
        for hx, hy in self._points:
            dx, dy = x - hx, y - hy
            dist = math.hypot(dx, dy)
            if dist < 1e-6 or dist > radius:
                continue
            weight = (radius - dist) / radius
            rx += (dx / dist) * weight
            ry += (dy / dist) * weight
        return rx, ry

    def __len__(self) -> int:
        return len(self._points)

    def __iter__(self) -> Iterator[Tuple[float, float]]:
        return iter(self._points)
