import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_script.stuck_detector import StuckDetector 


def test_not_stuck_with_insufficient_history():
    d = StuckDetector(window_sec=6.0, min_displacement=0.3)
    d.update(0.0, 0.0, 0.0)
    assert not d.is_stuck()


def test_not_stuck_before_window_fills():
    d = StuckDetector(window_sec=6.0, min_displacement=0.3)
    d.update(0.0, 0.0, 0.0)
    d.update(1.0, 0.0, 0.0) 
    assert not d.is_stuck()


def test_stuck_when_oscillating_in_place():
    d = StuckDetector(window_sec=6.0, min_displacement=0.3)
    t = 0.0
    x = 0.0
    for _ in range(30):
        x = 0.1 if x == 0.0 else 0.0  
        t += 0.25
        d.update(t, x, 0.0)
    assert d.is_stuck()


def test_not_stuck_when_making_progress():
    d = StuckDetector(window_sec=6.0, min_displacement=0.3)
    t = 0.0
    x = 0.0
    for _ in range(30):
        x += 0.05
        t += 0.25
        d.update(t, x, 0.0)
    assert not d.is_stuck()


def test_reset_clears_history():
    d = StuckDetector(window_sec=6.0, min_displacement=0.3)
    for i in range(20):
        d.update(i * 0.3, 0.0, 0.0)
    assert d.is_stuck()
    d.reset()
    assert not d.is_stuck()
