import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from safety import (  # noqa: E402
    check_reach,
    clamp_pos_box,
    clamp_pos_delta,
    ema_pos,
    is_finite,
)


def approx(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_ema_pos_identity_when_alpha_one():
    prev = [1, 2, 3]
    sample = [5, 6, 7]
    assert ema_pos(prev, sample, 1.0) == sample


def test_ema_pos_hold_when_alpha_zero():
    prev = [1, 2, 3]
    sample = [99, 99, 99]
    assert ema_pos(prev, sample, 0.0) == prev


def test_ema_pos_mixes_channels():
    assert approx(ema_pos([0, 0, 0], [10, 20, 30], 0.5), [5, 10, 15])


def test_clamp_pos_delta_allows_small_motion():
    last = [0, 0, 0]
    target = [1.0, -1.0, 0.5]
    assert clamp_pos_delta(target, last, 2.0) == target


def test_clamp_pos_delta_limits_huge_jump():
    last = [0, 0, 0]
    target = [500, -500, 0]
    assert clamp_pos_delta(target, last, 2.0) == [2.0, -2.0, 0.0]


def test_clamp_pos_box_inside_box_is_noop():
    anchor = [100, 100, 100]
    target = [110, 90, 100]
    assert clamp_pos_box(target, anchor, [50, 50, 50]) == target


def test_clamp_pos_box_clips_to_box():
    anchor = [0, 0, 0]
    target = [1000, -1000, 0]
    assert clamp_pos_box(target, anchor, [50, 50, 50]) == [50, -50, 0]


def test_check_reach_at_anchor():
    anchor = [100, 200, 300]
    assert check_reach(anchor, anchor, 100.0)


def test_check_reach_on_boundary():
    anchor = [0, 0, 0]
    assert check_reach([0, 0, 100], anchor, 100.0)


def test_check_reach_just_outside():
    anchor = [0, 0, 0]
    assert not check_reach([50, 50, 70.8], anchor, 100.0)


def test_is_finite():
    assert is_finite([1, 2, 3])
    assert not is_finite([1, float("nan"), 3])
    assert not is_finite([1, 2, float("inf")])


if __name__ == "__main__":
    import traceback
    failed = 0
    tests = [(n, fn) for n, fn in globals().items()
             if n.startswith("test_") and callable(fn)]
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception:
            failed += 1
            print(f"FAIL {name}")
            traceback.print_exc()
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
