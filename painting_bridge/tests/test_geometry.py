import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from geometry import (  # noqa: E402
    R_eye,
    R_inv,
    R_mul,
    R_to_rpy_xyz_deg,
    R_to_rpy_xyz_deg_continuous,
    quat_to_R,
    rpy_xyz_deg_to_R,
    scale_axis_angle,
)


def _rand_quat(rng):
    while True:
        w, x, y, z = (rng.uniform(-1, 1) for _ in range(4))
        n = math.sqrt(w * w + x * x + y * y + z * z)
        if n > 1e-3:
            return w / n, x / n, y / n, z / n


def _R_close(A, B, tol=1e-9):
    return all(abs(A[i][j] - B[i][j]) < tol
               for i in range(3) for j in range(3))


def _approx(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_quat_to_R_orthonormal():
    rng = random.Random(0)
    for _ in range(50):
        R = quat_to_R(*_rand_quat(rng))
        RtR = R_mul(R_inv(R), R)
        assert _R_close(RtR, R_eye(), 1e-12)


def test_rpy_to_R_to_rpy_roundtrip():
    rng = random.Random(1)
    for _ in range(1000):
        rx = rng.uniform(-180, 180)
        ry = rng.uniform(-89, 89)  # avoid gimbal lock
        rz = rng.uniform(-180, 180)
        R = rpy_xyz_deg_to_R([rx, ry, rz])
        out = R_to_rpy_xyz_deg(R)
        # Reconstruct and check matrix equality (Euler may have alternate
        # representations but the matrix is unique).
        R2 = rpy_xyz_deg_to_R(out)
        assert _R_close(R, R2, 1e-9)


def test_continuity_across_pos_seam():
    # True rz sweeps 175 -> 180 -> 185 (continuous). Build R from each true
    # angle; the decomposer's primary branch will wrap (185 -> -175). The
    # continuous decomposer must keep tracking 185.
    last = [0.0, 0.0, 175.0]
    for true_rz in (175.0, 178.0, 180.0, 182.0, 185.0, 188.0, 190.0):
        R = rpy_xyz_deg_to_R([0.0, 0.0, true_rz])
        out = R_to_rpy_xyz_deg_continuous(R, last)
        assert abs(out[2] - true_rz) < 1e-6, (true_rz, out)
        last = out


def test_continuity_across_neg_seam():
    last = [0.0, 0.0, -175.0]
    for true_rz in (-175.0, -178.0, -180.0, -182.0, -185.0, -188.0, -190.0):
        R = rpy_xyz_deg_to_R([0.0, 0.0, true_rz])
        out = R_to_rpy_xyz_deg_continuous(R, last)
        assert abs(out[2] - true_rz) < 1e-6, (true_rz, out)
        last = out


def test_continuity_chooses_alternate_branch_at_gimbal():
    # Walk past gimbal lock from below: ry = 88 -> 89 -> 89.95 -> 89.999.
    # The primary decomposition near gimbal pins rz=0 and folds into rx, but
    # if last_rpy says rx=0 and rz=170, the alternate branch (rx+180,
    # 180-ry, rz+180) should win.
    rx_true, rz_true = 5.0, 170.0
    last = [rx_true, 88.0, rz_true]
    for ry_true in (88.5, 89.0, 89.5, 89.9):
        R = rpy_xyz_deg_to_R([rx_true, ry_true, rz_true])
        out = R_to_rpy_xyz_deg_continuous(R, last)
        # At least one branch must reproduce R; the chosen branch must stay
        # near last (no 180-deg jumps in rx or rz).
        R2 = rpy_xyz_deg_to_R(out)
        assert _R_close(R, R2, 1e-7)
        assert abs(out[0] - rx_true) < 1.0, (ry_true, out)
        assert abs(out[2] - rz_true) < 1.0, (ry_true, out)
        last = out


def test_axis_angle_scale_half_compose_full():
    rng = random.Random(2)
    for _ in range(50):
        # Build a random rotation from axis-angle to keep theta in (0.1, pi-0.1).
        ax = [rng.uniform(-1, 1) for _ in range(3)]
        n = math.sqrt(sum(c * c for c in ax))
        ax = [c / n for c in ax]
        theta = rng.uniform(0.1, math.pi - 0.1)
        c, s = math.cos(theta), math.sin(theta)
        C = 1.0 - c
        x, y, z = ax
        R = [
            [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
        half = scale_axis_angle(R, 0.5)
        full = R_mul(half, half)
        assert _R_close(full, R, 1e-9)


def test_axis_angle_scale_zero_is_identity():
    R = rpy_xyz_deg_to_R([30.0, 20.0, 50.0])
    out = scale_axis_angle(R, 0.0)
    assert _R_close(out, R_eye(), 1e-12)


def test_axis_angle_scale_one_is_self():
    R = rpy_xyz_deg_to_R([30.0, 20.0, 50.0])
    out = scale_axis_angle(R, 1.0)
    assert _R_close(out, R, 1e-9)


def test_axis_angle_scale_near_identity_no_nan():
    # Tiny perturbation: theta ~ 1e-12. Must not divide by zero.
    R = [
        [1.0,   1e-12, 0.0],
        [-1e-12, 1.0,  0.0],
        [0.0,   0.0,   1.0],
    ]
    out = scale_axis_angle(R, 0.5)
    for i in range(3):
        for j in range(3):
            assert math.isfinite(out[i][j])


def test_mount_similarity_swaps_axes():
    # imu_mount_rpy = [0,0,90]: a +90 deg rotation about IMU X must become
    # a +90 deg rotation about base Y after R_im2base * R_rel_imu * R_im2base^-1.
    R_im2base = rpy_xyz_deg_to_R([0.0, 0.0, 90.0])
    R_rel_imu = rpy_xyz_deg_to_R([90.0, 0.0, 0.0])  # +90 about IMU X
    R_rel_base = R_mul(R_mul(R_im2base, R_rel_imu), R_inv(R_im2base))
    expected = rpy_xyz_deg_to_R([0.0, 90.0, 0.0])  # +90 about base Y
    assert _R_close(R_rel_base, expected, 1e-9)


def test_mount_similarity_identity_passes_through():
    R_im2base = rpy_xyz_deg_to_R([0.0, 0.0, 0.0])
    R_rel_imu = rpy_xyz_deg_to_R([10.0, 20.0, 30.0])
    R_rel_base = R_mul(R_mul(R_im2base, R_rel_imu), R_inv(R_im2base))
    assert _R_close(R_rel_base, R_rel_imu, 1e-12)
