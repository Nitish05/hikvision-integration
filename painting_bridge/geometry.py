"""Pure-Python rotation utilities for the FR5 streaming bridge.

All matrices are 3x3 list-of-lists. All angles are in degrees on the
public interface. RPY uses FR5/Fairino convention: extrinsic XYZ-fixed,
i.e. R = Rz(rz) @ Ry(ry) @ Rx(rx).
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

R3 = List[List[float]]


def R_eye() -> R3:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def R_mul(A: R3, B: R3) -> R3:
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def R_inv(R: R3) -> R3:
    # Orthonormal -> inverse is transpose.
    return [[R[j][i] for j in range(3)] for i in range(3)]


def quat_to_R(qw: float, qx: float, qy: float, qz: float) -> R3:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return R_eye()
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ]


def rpy_xyz_deg_to_R(rpy: Sequence[float]) -> R3:
    rx, ry, rz = (math.radians(a) for a in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz @ Ry @ Rx
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ]


def R_to_rpy_xyz_deg(R: R3) -> List[float]:
    """Decompose R into [rx, ry, rz] degrees using R = Rz @ Ry @ Rx.

    ry in [-90, 90]. At gimbal lock (|R[2][0]| ≈ 1) returns rz=0 and folds
    the underdetermined freedom into rx.
    """
    sy_neg = R[2][0]
    if sy_neg > 1.0:
        sy_neg = 1.0
    elif sy_neg < -1.0:
        sy_neg = -1.0
    if abs(sy_neg) > 1.0 - 1e-6:
        # Gimbal lock: ry = ±90°, rx and rz are coupled. Convention: rz = 0.
        ry = -math.asin(sy_neg)
        rz = 0.0
        # When ry = +90 (sy_neg = -1): R[0][1] = -sin(rx-rz), R[1][1] = cos(rx-rz)
        # When ry = -90 (sy_neg = +1): R[0][1] =  sin(rx+rz), R[1][1] = cos(rx+rz)
        if sy_neg < 0:
            rx = math.atan2(R[0][1], R[1][1])
        else:
            rx = math.atan2(-R[0][1], R[1][1])
    else:
        ry = math.asin(-sy_neg)
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _wrap_nearest(angle: float, ref: float) -> float:
    """Add a multiple of 360 to angle so it lands closest to ref."""
    d = angle - ref
    k = round(d / 360.0)
    return angle - 360.0 * k


def R_to_rpy_xyz_deg_continuous(R: R3, last_rpy: Sequence[float]) -> List[float]:
    """Decompose R but pick the branch closest to last_rpy (continuity).

    Both Tait-Bryan solutions are evaluated:
        primary:   (rx,        ry,         rz)
        alternate: (rx + 180,  180 - ry,   rz + 180)   (mod wrap)

    Each component is then unwrapped (±360k) to be nearest the corresponding
    component of last_rpy. The branch with smaller L-infinity distance to
    last_rpy wins. This keeps clamps in continuous-angle space across the
    ±180 seam and across the gimbal-lock branch flip.
    """
    primary = R_to_rpy_xyz_deg(R)
    alternate = [primary[0] + 180.0, 180.0 - primary[1], primary[2] + 180.0]

    def _unwrap_to_ref(rpy: Sequence[float]) -> List[float]:
        return [_wrap_nearest(rpy[i], last_rpy[i]) for i in range(3)]

    p = _unwrap_to_ref(primary)
    a = _unwrap_to_ref(alternate)

    def _linf(rpy: Sequence[float]) -> float:
        return max(abs(rpy[i] - last_rpy[i]) for i in range(3))

    return p if _linf(p) <= _linf(a) else a


def _R_axis_angle(R: R3) -> Tuple[List[float], float]:
    """Extract (axis, theta_rad) from R. axis is unit-length.

    For theta near 0 returns ([1,0,0], 0). For theta near pi falls back to
    eigen-style recovery from the symmetric part R+I.
    """
    trace = R[0][0] + R[1][1] + R[2][2]
    cos_t = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    theta = math.acos(cos_t)
    if theta < 1e-9:
        return [1.0, 0.0, 0.0], 0.0
    if math.pi - theta < 1e-6:
        # theta ≈ pi: use diagonal of R+I; pick column with largest diagonal.
        M = [[R[i][j] + (1.0 if i == j else 0.0) for j in range(3)]
             for i in range(3)]
        diag = [M[0][0], M[1][1], M[2][2]]
        k = max(range(3), key=lambda i: diag[i])
        col = [M[0][k], M[1][k], M[2][k]]
        n = math.sqrt(sum(c * c for c in col))
        if n < 1e-12:
            return [1.0, 0.0, 0.0], 0.0
        axis = [c / n for c in col]
        return axis, theta
    s = 1.0 / (2.0 * math.sin(theta))
    axis = [
        (R[2][1] - R[1][2]) * s,
        (R[0][2] - R[2][0]) * s,
        (R[1][0] - R[0][1]) * s,
    ]
    return axis, theta


def _axis_angle_to_R(axis: Sequence[float], theta: float) -> R3:
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    x, y, z = axis
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def scale_axis_angle(R: R3, s: float) -> R3:
    """Scale rotation R by factor s along its axis-angle representation."""
    axis, theta = _R_axis_angle(R)
    if theta < 1e-9:
        return R_eye()
    return _axis_angle_to_R(axis, s * theta)
