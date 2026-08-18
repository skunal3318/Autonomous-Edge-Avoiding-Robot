import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_script.scan_utils import analyze_scan, bin_ranges, is_cliff 


def flat_floor_scan(n=20, reading=0.5):
    return [reading] * n


def edge_ahead_scan(n=20, floor=0.5, edge=1.0):
    ranges = [floor] * n
    mid = n // 2
    ranges[mid - 1:mid + 1] = [edge, edge]
    return ranges


def edge_on_right_scan(n=20, floor=0.5, edge=1.0):
    ranges = [floor] * n
    ranges[int(n * 0.75):] = [edge] * (n - int(n * 0.75))
    return ranges


def edge_on_left_scan(n=20, floor=0.5, edge=1.0):
    ranges = [floor] * n
    ranges[:int(n * 0.25)] = [edge] * int(n * 0.25)
    return ranges


def test_flat_floor_no_danger():
    d = analyze_scan(flat_floor_scan(), edge_threshold=0.65)
    assert not d.danger
    assert not d.hard_stop
    assert d.strongest_side == 'none'
    assert d.angular_bias == 0.0


def test_edge_dead_ahead_triggers_hard_stop():
    d = analyze_scan(edge_ahead_scan(), edge_threshold=0.65, num_bins=5, center_bins=1)
    assert d.danger
    assert d.hard_stop


def test_edge_on_right_biases_left_without_hard_stop():
    d = analyze_scan(edge_on_right_scan(), edge_threshold=0.65, num_bins=5, center_bins=1)
    assert d.danger
    assert d.strongest_side == 'right'
    assert d.angular_bias > 0  


def test_edge_on_left_biases_right():
    d = analyze_scan(edge_on_left_scan(), edge_threshold=0.65, num_bins=5, center_bins=1)
    assert d.danger
    assert d.strongest_side == 'left'
    assert d.angular_bias < 0


def test_empty_scan_is_safe_default():
    d = analyze_scan([], edge_threshold=0.65)
    assert not d.danger
    assert not d.hard_stop


def test_nan_readings_are_ignored():
    ranges = [0.5] * 20
    ranges[10] = float('nan')
    ranges[11] = float('inf')
    d = analyze_scan(ranges, edge_threshold=0.65)
    assert not d.danger 


def test_bin_ranges_uses_max_within_sector():
    ranges = [0.1, 0.9, 0.1, 0.1]
    bins = bin_ranges(ranges, num_bins=2)
    assert bins[0] == 0.9  
    assert bins[1] == 0.1


def test_bin_ranges_handles_empty_input():
    assert bin_ranges([], num_bins=4) == [0.0, 0.0, 0.0, 0.0]


def test_no_return_sector_is_treated_as_edge_not_floor():
    ranges = [0.5] * 20
    mid = 10
    ranges[mid - 2:mid + 2] = [float('inf')] * 4
    d = analyze_scan(ranges, edge_threshold=0.65, num_bins=5, center_bins=1)
    assert d.danger
    assert d.hard_stop


def test_bin_ranges_all_non_finite_sector_is_inf():
    bins = bin_ranges([float('inf'), float('nan'), 0.5, 0.5], num_bins=2)
    assert bins[0] == float('inf')
    assert bins[1] == 0.5


def test_threshold_boundary_is_strict_greater_than():
    ranges = [0.65] * 20  
    d = analyze_scan(ranges, edge_threshold=0.65)
    assert not d.danger
    ranges2 = [0.6501] * 20
    d2 = analyze_scan(ranges2, edge_threshold=0.65)
    assert d2.danger


def test_is_cliff_true_on_no_return():
    assert is_cliff(float('inf'), threshold=0.2)
    assert is_cliff(float('nan'), threshold=0.2)


def test_is_cliff_true_beyond_threshold():
    assert is_cliff(0.21, threshold=0.2)


def test_is_cliff_false_on_floor():
    assert not is_cliff(0.12, threshold=0.2)
    assert not is_cliff(0.2, threshold=0.2) 
