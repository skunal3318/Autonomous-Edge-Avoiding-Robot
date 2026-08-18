from dataclasses import dataclass
from typing import List, Sequence
import math


@dataclass
class SteeringDecision:
    hard_stop: bool       
    danger: bool           
    angular_bias: float    
    strongest_side: str    


def bin_ranges(ranges: Sequence[float], num_bins: int) -> List[float]:
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
            bins.append(math.inf)
    return bins


def analyze_scan(ranges: Sequence[float], edge_threshold: float,
                  num_bins: int = 5, center_bins: int = 1) -> SteeringDecision:
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
        offset = (i + 0.5 - mid) / mid 
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
    return (not math.isfinite(range_value)) or range_value > threshold
