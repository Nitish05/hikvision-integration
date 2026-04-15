"""Tests for recording.format_row / format_header and recorder.py round-trip.

The round-trip test reimplements recorder.py's load_positions_from_file_v2
parser inline so we don't need to import its Tk-based module.
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from recording import (  # noqa: E402
    FRTYPE,
    TRAILER,
    format_header,
    format_row,
)


def test_format_row_exact_bytes():
    # 6 joints, 6 tcp, trigger=1 -> expect 15 comma-separated fields + \n
    joints = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    tcp = [100.0, 200.0, 300.0, 10.0, 20.0, 30.0]
    row = format_row(joints, tcp, 1)
    assert row == (
        "10.0000,20.0000,30.0000,40.0000,50.0000,60.0000,"
        "100.0000,200.0000,300.0000,10.0000,20.0000,30.0000,"
        f"{FRTYPE},1,{TRAILER}\n"
    )


def test_format_row_precision_four_decimals():
    joints = [1.12345, 0, 0, 0, 0, 0]
    tcp = [0.99999, 0, 0, 0, 0, 0]
    row = format_row(joints, tcp, 0)
    parts = row.rstrip("\n").split(",")
    assert parts[0] == "1.1235"     # rounded
    assert parts[6] == "1.0000"     # rounded up


def test_format_row_trigger_zero_and_nonbool():
    row0 = format_row([0]*6, [0]*6, 0)
    row1 = format_row([0]*6, [0]*6, 1)
    assert row0.rstrip("\n").split(",")[13] == "0"
    assert row1.rstrip("\n").split(",")[13] == "1"


def test_format_row_rejects_wrong_arity():
    try:
        format_row([0]*5, [0]*6, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError")


def test_format_header_fields():
    h = format_header(42, 0.010, tool=0, diconfig=1, doconfig=1)
    # Header: N(10-wide),1,period_ms,tool,diconfig,doconfig
    assert h.startswith("0000000042,1,10,0,1,1\n"), f"got: {h!r}"


def test_format_header_fixed_width_for_seek_rewrite():
    # Any legal N must produce the same-length header string so seek(0)+write
    # leaves subsequent row bytes intact.
    h0 = format_header(0, 0.010)
    h1 = format_header(1, 0.010)
    hbig = format_header(999999, 0.010)
    assert len(h0) == len(h1) == len(hbig)


def test_round_trip_recorderpy_compatibility(tmp_path=None):
    """Write a small trajectory with the recorder's format and parse it with
    a faithful reimplementation of recorder.py:load_positions_from_file_v2.
    The xyz_rxryrz and trigger fields must round-trip exactly."""
    samples = [
        # joints,                        tcp,                             trigger
        ([10, 20, 30, 40, 50, 60],      [100, 200, 300, 10, 20, 30],     0),
        ([11, 21, 31, 41, 51, 61],      [101, 201, 301, 11, 21, 31],     1),
        ([12.1234, 22, 32, 42, 52, 62], [102.5, 202, 302, 12, 22, 32],   1),
    ]
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as tf:
        path = tf.name
        tf.write(format_header(len(samples), 0.010))
        for j, t, trig in samples:
            tf.write(format_row(j, t, trig))

    # --- reimplement recorder.py:258-272 parser ---
    parsed = []
    with open(path, "r") as f:
        lines = f.readlines()
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = line.strip().split(",")
        assert len(parts) == 15, f"expected 15 fields, got {len(parts)}"
        xyz_rxryrz = list(map(float, parts[6:12]))
        trigger = int(float(parts[13]))
        parsed.append((xyz_rxryrz, trigger))

    os.unlink(path)

    assert len(parsed) == 3
    for (expected_joints, expected_tcp, expected_trig), (got_tcp, got_trig) \
            in zip(samples, parsed):
        for a, b in zip(expected_tcp, got_tcp):
            assert abs(a - b) < 1e-3
        assert got_trig == expected_trig


if __name__ == "__main__":
    import traceback
    failed = 0
    g = dict(globals())
    total = 0
    for name, fn in g.items():
        if name.startswith("test_") and callable(fn):
            total += 1
            try:
                fn()
                print(f"PASS {name}")
            except Exception:
                failed += 1
                print(f"FAIL {name}")
                traceback.print_exc()
    print(f"\n{total - failed}/{total} passed")
    sys.exit(1 if failed else 0)
