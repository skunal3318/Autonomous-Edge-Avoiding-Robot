import math
from collections import deque
from typing import Iterator, Tuple


class HazardMemory:
    def __init__(self, max_points: int = 200):
        self._points = deque(maxlen=max_points)

    def record(self, x: float, y: float) -> None:
        self._points.append((x, y))

    def repulsion(self, x: float, y: float, radius: float) -> Tuple[float, float]:
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
