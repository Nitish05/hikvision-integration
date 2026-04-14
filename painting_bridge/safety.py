"""Pure-function safety primitives for the FR5 streaming bridge.

All functions take and return plain lists of six floats so they compose
cleanly and are trivial to unit-test.
"""

from __future__ import annotations

import math
from typing import List, Sequence


Pose = List[float]  # [x, y, z, rx, ry, rz]


def ema(prev: Pose, sample: Pose, alpha_xyz: float, alpha_rot: float) -> Pose:
    """Exponential moving average. alpha=1.0 disables filtering on that channel."""
    out = [0.0] * 6
    for i in range(3):
        out[i] = alpha_xyz * sample[i] + (1.0 - alpha_xyz) * prev[i]
    for i in range(3, 6):
        out[i] = alpha_rot * sample[i] + (1.0 - alpha_rot) * prev[i]
    return out


def clamp_delta(target: Pose, last: Pose,
                max_mm: float, max_deg: float) -> Pose:
    """Clamp per-axis change from last target. Catches trilat recovery jumps,
    yaw wraps, and sudden handle motion."""
    out = [0.0] * 6
    for i in range(3):
        d = target[i] - last[i]
        if d > max_mm:
            d = max_mm
        elif d < -max_mm:
            d = -max_mm
        out[i] = last[i] + d
    for i in range(3, 6):
        d = target[i] - last[i]
        if d > max_deg:
            d = max_deg
        elif d < -max_deg:
            d = -max_deg
        out[i] = last[i] + d
    return out


def clamp_workspace(target: Pose, anchor: Pose,
                    box_mm: Sequence[float],
                    rot_deg: Sequence[float]) -> Pose:
    """Hard AABB clamp around the anchor pose. Prevents reaching outside the
    FR5 envelope or into known fixtures."""
    out = list(target)
    for i in range(3):
        lo, hi = anchor[i] - box_mm[i], anchor[i] + box_mm[i]
        if out[i] < lo:
            out[i] = lo
        elif out[i] > hi:
            out[i] = hi
    for i in range(3):
        j = 3 + i
        lo, hi = anchor[j] - rot_deg[i], anchor[j] + rot_deg[i]
        if out[j] < lo:
            out[j] = lo
        elif out[j] > hi:
            out[j] = hi
    return out


def wrap_deg(angle: float) -> float:
    """Wrap degrees to (-180, 180]."""
    a = ((angle + 180.0) % 360.0) - 180.0
    if a == -180.0:
        a = 180.0
    return a


def normalize_orientation(target: Pose, anchor: Pose) -> Pose:
    """Rewrite target[3:6] as anchor + wrapped_delta so unwrapped firmware
    angles (rollCont/yawCont can exceed +/-180) don't confuse the IK."""
    out = list(target)
    for i in range(3, 6):
        out[i] = anchor[i] + wrap_deg(target[i] - anchor[i])
    return out


def is_finite(p: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in p)


def check_reach(target: Pose, anchor: Pose, max_radius_mm: float) -> bool:
    """Spherical reachability guard. Returns True if target[:3] is within
    max_radius_mm of anchor[:3]. Catches diagonal excursions that the
    axis-aligned workspace box lets through."""
    dx = target[0] - anchor[0]
    dy = target[1] - anchor[1]
    dz = target[2] - anchor[2]
    return (dx * dx + dy * dy + dz * dz) <= max_radius_mm * max_radius_mm
