"""Quaternion math for the painting bridge.

Convention: unit quaternion q = (w, x, y, z), Hamilton product, active
rotation (q * v * q^-1 rotates v). Tait-Bryan extraction matches the
firmware's quatToEulerDeg (XYZ-extrinsic / ZYX-intrinsic, aerospace RPY)
and Fairino's [rx, ry, rz] convention so a round-trip through quaternion
preserves end-effector orientation.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Quat = Tuple[float, float, float, float]  # (w, x, y, z)
QuatList = List[float]

IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)


def quat_mul(p: Sequence[float], q: Sequence[float]) -> Quat:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def quat_conj(q: Sequence[float]) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def quat_norm(q: Sequence[float]) -> float:
    return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def quat_normalize(q: Sequence[float]) -> Quat:
    n = quat_norm(q)
    if n < 1e-12:
        return IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_canonicalize(q: Sequence[float]) -> Quat:
    """Force w >= 0 so subsequent angle math takes the short arc."""
    if q[0] < 0.0:
        return (-q[0], -q[1], -q[2], -q[3])
    return (q[0], q[1], q[2], q[3])


def quat_from_zyx_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> Quat:
    """Aerospace RPY (XYZ-extrinsic / ZYX-intrinsic) to unit quaternion.
    rx = roll about X, ry = pitch about Y, rz = yaw about Z."""
    cr = math.cos(math.radians(rx_deg) * 0.5)
    sr = math.sin(math.radians(rx_deg) * 0.5)
    cp = math.cos(math.radians(ry_deg) * 0.5)
    sp = math.sin(math.radians(ry_deg) * 0.5)
    cy = math.cos(math.radians(rz_deg) * 0.5)
    sy = math.sin(math.radians(rz_deg) * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_to_zyx_deg(q: Sequence[float]) -> Tuple[float, float, float]:
    """Inverse of quat_from_zyx_deg. Returns (rx, ry, rz) in degrees, with
    ry in [-90, 90] (pitch saturates at gimbal lock; yaw absorbs the rest)."""
    w, x, y, z = q
    sinp = 2.0 * (w * y - z * x)
    if sinp >= 1.0:
        ry = 90.0
    elif sinp <= -1.0:
        ry = -90.0
    else:
        ry = math.degrees(math.asin(sinp))
    rx = math.degrees(math.atan2(2.0 * (w * x + y * z),
                                 1.0 - 2.0 * (x * x + y * y)))
    rz = math.degrees(math.atan2(2.0 * (w * z + x * y),
                                 1.0 - 2.0 * (y * y + z * z)))
    return (rx, ry, rz)


def quat_axis_angle_deg(q: Sequence[float]) -> float:
    """Geodesic rotation magnitude of q in degrees (always >= 0, <= 180)."""
    w = max(-1.0, min(1.0, q[0]))
    if w < 0.0:
        w = -w  # short-arc
    return 2.0 * math.degrees(math.acos(w))


def quat_scale_angle(q: Sequence[float], alpha: float) -> Quat:
    """Scale a unit quaternion's rotation angle by alpha (0..1 typical).
    Equivalent to slerp(IDENTITY, q, alpha) but cheaper. Always uses the
    short arc."""
    qc = quat_canonicalize(q)
    w = max(-1.0, min(1.0, qc[0]))
    angle = 2.0 * math.acos(w)
    if angle < 1e-9:
        return IDENTITY
    s = math.sin(angle * 0.5)
    new_half = 0.5 * alpha * angle
    s2 = math.sin(new_half)
    return (math.cos(new_half), qc[1] / s * s2, qc[2] / s * s2, qc[3] / s * s2)


def quat_slerp(q0: Sequence[float], q1: Sequence[float], t: float) -> Quat:
    """Spherical linear interpolation. t=0 -> q0, t=1 -> q1."""
    if t <= 0.0:
        return tuple(q0)  # type: ignore
    if t >= 1.0:
        return tuple(q1)  # type: ignore
    dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    if dot > 0.9995:
        # Near-parallel: fall back to NLERP for numerical stability.
        out = (q0[0] + t * (q1[0] - q0[0]),
               q0[1] + t * (q1[1] - q0[1]),
               q0[2] + t * (q1[2] - q0[2]),
               q0[3] + t * (q1[3] - q0[3]))
        return quat_normalize(out)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_t = math.sin(theta)
    a = math.sin((1.0 - t) * theta) / sin_t
    b = math.sin(t * theta) / sin_t
    return (a * q0[0] + b * q1[0],
            a * q0[1] + b * q1[1],
            a * q0[2] + b * q1[2],
            a * q0[3] + b * q1[3])


def quat_clamp_delta(q_target: Sequence[float], q_last: Sequence[float],
                     max_deg: float) -> Quat:
    """Limit the rotation from q_last to q_target to max_deg (geodesic).
    Mirrors safety.clamp_delta for position — caps angular speed per cycle."""
    q_delta = quat_mul(quat_conj(q_last), q_target)
    angle = quat_axis_angle_deg(q_delta)
    if angle <= max_deg or angle < 1e-9:
        return tuple(q_target)  # type: ignore
    q_clamped = quat_scale_angle(q_delta, max_deg / angle)
    return quat_mul(q_last, q_clamped)


def quat_clamp_box(q_target: Sequence[float], q_anchor: Sequence[float],
                   max_deg: float) -> Quat:
    """Limit the angular distance from q_anchor to max_deg (geodesic).
    Replaces the per-axis Euler clamp_workspace for rotation; one scalar
    radius is the natural quat analogue of an angular box."""
    q_delta = quat_mul(quat_conj(q_anchor), q_target)
    angle = quat_axis_angle_deg(q_delta)
    if angle <= max_deg or angle < 1e-9:
        return tuple(q_target)  # type: ignore
    q_clamped = quat_scale_angle(q_delta, max_deg / angle)
    return quat_mul(q_anchor, q_clamped)


def is_finite_q(q: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in q)
