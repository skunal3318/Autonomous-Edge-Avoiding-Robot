from collections import deque
from dataclasses import dataclass


@dataclass
class _Sample:
    t: float
    x: float
    y: float


class StuckDetector:
    def __init__(self, window_sec: float = 6.0, min_displacement: float = 0.3):
        self.window_sec = window_sec
        self.min_displacement = min_displacement
        self._history = deque()

    def update(self, t: float, x: float, y: float) -> None:
        self._history.append(_Sample(t, x, y))
        while self._history and (t - self._history[0].t) > self.window_sec:
            self._history.popleft()

    def is_stuck(self) -> bool:
        if len(self._history) < 2:
            return False
        oldest = self._history[0]
        newest = self._history[-1]
        if (newest.t - oldest.t) < self.window_sec * 0.8:
            return False 
        dx = newest.x - oldest.x
        dy = newest.y - oldest.y
        displacement = (dx * dx + dy * dy) ** 0.5
        return displacement < self.min_displacement

    def reset(self) -> None:
        self._history.clear()
