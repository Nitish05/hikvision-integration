"""Pure-function safety primitives for the FR5 streaming bridge.

Position is handled here (per-axis Euler-style math is correct for XYZ).
Rotation lives in quat.py — Euler clamps on rotation are unsafe near the
±180 seam and ill-defined for combined rotations.
"""

from __future__ import annotations

import math
from typing import List, Sequence


Pos = List[float]   # [x, y, z]
Pose = List[float]  # [x, y, z, rx, ry, rz] — used at the FR5 send boundary


def ema_pos(prev: Pos, sample: Pos, alpha: float) -> Pos:
    """Exponential moving average on a 3-vector. alpha=1.0 disables filtering."""
    return [alpha * sample[i] + (1.0 - alpha) * prev[i] for i in range(3)]


def clamp_pos_delta(target: Pos, last: Pos, max_mm: float) -> Pos:
    """Clamp per-axis change from last target. Catches trilat-recovery jumps
    and sudden handle motion."""
    out = [0.0, 0.0, 0.0]
    for i in range(3):
        d = target[i] - last[i]
        if d > max_mm:
            d = max_mm
        elif d < -max_mm:
            d = -max_mm
        out[i] = last[i] + d
    return out


def clamp_pos_box(target: Pos, anchor: Pos, box_mm: Sequence[float]) -> Pos:
    """Hard AABB clamp around the anchor position."""
    out = list(target)
    for i in range(3):
        lo, hi = anchor[i] - box_mm[i], anchor[i] + box_mm[i]
        if out[i] < lo:
            out[i] = lo
        elif out[i] > hi:
            out[i] = hi
    return out


def is_finite(p: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in p)


def check_reach(target: Pos, anchor: Pos, max_radius_mm: float) -> bool:
    """Spherical reachability guard. Catches diagonal excursions the
    axis-aligned position box lets through."""
    dx = target[0] - anchor[0]
    dy = target[1] - anchor[1]
    dz = target[2] - anchor[2]
    return (dx * dx + dy * dy + dz * dz) <= max_radius_mm * max_radius_mm
