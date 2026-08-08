"""Pure, ROS-free helper functions that turn a LaserScan's ranges into an
edge-avoidance steering signal. Kept separate from the ROS node so this
logic can be unit-tested (see test/test_scan_utils.py) without rclpy or
a running simulator.
"""
from dataclasses import dataclass
from typing import List, Sequence
import math


@dataclass
class SteeringDecision:
    """Result of analyzing one LaserScan for edge-avoidance purposes."""
    hard_stop: bool       # True: an edge is under the robot's center bins -> must react now
    danger: bool           # True: an edge was found somewhere in the scan
    angular_bias: float    # -1..1. Positive = edge weighted to the right -> steer left (+z)
    strongest_side: str    # 'left' | 'right' | 'center' | 'none'


def bin_ranges(ranges: Sequence[float], num_bins: int) -> List[float]:
    """Split a scan into `num_bins` contiguous left-to-right sectors and
    return each sector's max range (non-finite readings dropped). Max,
    not mean/min, because a single beam finding no floor is already
    enough to flag that sector as an edge.
    """
    if not ranges or num_bins <= 0:
        return [0.0] * max(num_bins, 0)
    n = len(ranges)
    bins = []
    for b in range(num_bins):
        lo = (b * n) // num_bins
        hi = max(((b + 1) * n) // num_bins, lo + 1)
        sector = [r for r in ranges[lo:hi] if math.isfinite(r)]
        if sector:
            bins.append(max(sector))
        else:
            # Every beam in this sector came back non-finite (inf/nan) --
            # meaning no floor was found within the sensor's max range at
            # all, the clearest possible "no floor" signal (a clean
            # drop-off, not a glancing edge). Falling back to 0.0 here
            # previously read as "floor right under the sensor," which
            # silently defeated edge detection at the exact moment it
            # mattered most. math.inf guarantees it is flagged.
            bins.append(math.inf)
    return bins


def analyze_scan(ranges: Sequence[float], edge_threshold: float,
                  num_bins: int = 5, center_bins: int = 1) -> SteeringDecision:
    """Divide the scan into `num_bins` left-to-right sectors. Any sector
    whose max range exceeds `edge_threshold` is flagged as "no floor".
    The `center_bins` innermost sectors control `hard_stop` (an edge
    dead ahead, needs an immediate reactive maneuver); every flagged
    sector contributes to a proportional `angular_bias` so the robot
    can veer away from a peripheral edge without needing to stop at all.
    """
    bins = bin_ranges(ranges, num_bins)
    n = len(bins)
    if n == 0:
        return SteeringDecision(False, False, 0.0, 'none')
    mid = n / 2.0

    flags = [r > edge_threshold for r in bins]
    danger = any(flags)

    weighted = 0.0
    weight_total = 0.0
    for i, flagged in enumerate(flags):
        if not flagged:
            continue
        offset = (i + 0.5 - mid) / mid  # -1 (far left) .. +1 (far right)
        weighted += offset
        weight_total += 1.0
    angular_bias = (weighted / weight_total) if weight_total else 0.0

    half_center = max(center_bins, 1) // 2
    center_idx = n // 2
    center_lo = max(center_idx - half_center, 0)
    center_hi = min(center_idx + max(center_bins - half_center, 1), n)
    hard_stop = any(flags[center_lo:center_hi])

    if not danger:
        strongest_side = 'none'
    elif angular_bias > 0.15:
        strongest_side = 'right'
    elif angular_bias < -0.15:
        strongest_side = 'left'
    else:
        strongest_side = 'center'

    return SteeringDecision(hard_stop=hard_stop, danger=danger,
                             angular_bias=angular_bias, strongest_side=strongest_side)


def is_cliff(range_value: float, threshold: float) -> bool:
    """True if a single-beam range reading indicates no floor within
    `threshold` of the sensor. Used for the downward-facing corner
    IR/cliff sensors (bot.urdf.xacro's ir_front_left/right,
    ir_rear_left/right), which each report one range rather than a full
    scan -- the same "non-finite or too far = no floor" rule as
    bin_ranges(), just for a single reading instead of a sector."""
    return (not math.isfinite(range_value)) or range_value > threshold
