import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from safety import (  # noqa: E402
    check_reach,
    clamp_delta,
    clamp_workspace,
    ema,
    is_finite,
    normalize_orientation,
    wrap_deg,
)


def approx(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_ema_identity_when_alpha_one():
    prev = [1, 2, 3, 10, 20, 30]
    s = [5, 6, 7, 40, 50, 60]
    assert ema(prev, s, 1.0, 1.0) == s


def test_ema_hold_when_alpha_zero():
    prev = [1, 2, 3, 10, 20, 30]
    s = [99, 99, 99, 99, 99, 99]
    assert ema(prev, s, 0.0, 0.0) == prev


def test_ema_mixes_channels_independently():
    prev = [0] * 6
    s = [10, 10, 10, 100, 100, 100]
    out = ema(prev, s, 0.5, 0.25)
    assert approx(out[:3], [5, 5, 5])
    assert approx(out[3:], [25, 25, 25])


def test_clamp_delta_allows_small_motion():
    last = [0] * 6
    tgt = [1.0, -1.0, 0.5, 0.5, -0.5, 0.0]
    assert clamp_delta(tgt, last, 2.0, 1.0) == tgt


def test_clamp_delta_limits_huge_jump():
    last = [0] * 6
    tgt = [500, -500, 0, 90, -90, 0]
    out = clamp_delta(tgt, last, 2.0, 1.0)
    assert out == [2.0, -2.0, 0.0, 1.0, -1.0, 0.0]


def test_clamp_workspace_inside_box_is_noop():
    anchor = [100, 100, 100, 0, 0, 0]
    tgt = [110, 90, 100, 10, -10, 0]
    assert clamp_workspace(tgt, anchor, [50, 50, 50], [45, 45, 45]) == tgt


def test_clamp_workspace_clips_to_box():
    anchor = [0, 0, 0, 0, 0, 0]
    tgt = [1000, -1000, 0, 200, -200, 0]
    out = clamp_workspace(tgt, anchor, [50, 50, 50], [45, 45, 45])
    assert out == [50, -50, 0, 45, -45, 0]


def test_wrap_deg_range():
    assert math.isclose(wrap_deg(0), 0)
    assert math.isclose(wrap_deg(180), 180)
    assert math.isclose(wrap_deg(181), -179)
    assert math.isclose(wrap_deg(-181), 179)
    assert math.isclose(wrap_deg(720), 0)
    assert math.isclose(wrap_deg(-540), 180)


def test_normalize_orientation_unwraps_large_continuous_angle():
    # Target absolute orientation is wrapped to [-180, 180].
    tgt = [0, 0, 0, 10 + 725, 0, 0]  # 735 deg == 15 deg
    out = normalize_orientation(tgt)
    assert math.isclose(out[3], 15.0, abs_tol=1e-6)


def test_normalize_orientation_wraps_values_across_seam():
    # Boundary case: a continuously composed target near the +180 seam.
    # Example: tcp_start_rz=170 plus handle rotation yields target rz=195,
    # which is the same physical orientation as rz=-165 for the FR5 IK.
    tgt = [0, 0, 0, 0, 0, 195.0]
    out = normalize_orientation(tgt)
    assert math.isclose(out[5], -165.0, abs_tol=1e-6)


def test_check_reach_at_anchor():
    anchor = [100, 200, 300, 0, 0, 0]
    assert check_reach(anchor, anchor, 100.0)


def test_check_reach_on_boundary():
    anchor = [0, 0, 0, 0, 0, 0]
    # Exactly 100 mm away on Z axis — should be inclusive.
    assert check_reach([0, 0, 100, 0, 0, 0], anchor, 100.0)


def test_check_reach_just_outside():
    anchor = [0, 0, 0, 0, 0, 0]
    # 100.1 mm diagonal.
    assert not check_reach([50, 50, 70.8, 0, 0, 0], anchor, 100.0)


def test_check_reach_ignores_orientation():
    anchor = [0, 0, 0, 0, 0, 0]
    # Large orientation delta, zero translation — should pass.
    assert check_reach([0, 0, 0, 180, -180, 180], anchor, 10.0)


def test_is_finite():
    assert is_finite([1, 2, 3, 4, 5, 6])
    assert not is_finite([1, float("nan"), 3, 4, 5, 6])
    assert not is_finite([1, 2, float("inf"), 4, 5, 6])


if __name__ == "__main__":
    import traceback
    failed = 0
    g = dict(globals())
    for name, fn in g.items():
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception:
                failed += 1
                print(f"FAIL {name}")
                traceback.print_exc()
    print(f"\n{len([n for n in g if n.startswith('test_')]) - failed}/"
          f"{len([n for n in g if n.startswith('test_')])} passed")
    sys.exit(1 if failed else 0)
