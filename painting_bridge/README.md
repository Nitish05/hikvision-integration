# painting_bridge

Real-time Cartesian streaming from the Teensy painting handle (BNO055 + 3×
draw-wire trilateration) to a Fairino FR5 via `ServoCart`. The handle's
6-DOF pose drives the FR5 TCP at 125 Hz, anchored so the arm never jumps
on startup.

## Layout

| File | Role |
|---|---|
| `bridge.py` | Entry point. Orchestrates reader thread + servo loop. |
| `teensy_reader.py` | Serial auto-detect + Teleplot parser. |
| `fr5_servo.py` | `fairino.Robot.RPC` wrapper + `MockRobot` for dry-run. |
| `safety.py` | EMA, delta clamp, workspace box, orientation wrap. |
| `config.yaml` | Tunables (scales, limits, filter alpha). |
| `tests/test_safety.py` | Unit tests for pure-function safety primitives. |

## Install

```
pip install pyserial pyyaml
```
(Fairino SDK is already in `../fairino/`.)

## Run

### Dry-run (no robot, Mock only)
```
python bridge.py --dry-run
```
Expect logs like `preflight OK`, `anchored`, and `sent=125 skipped=0 clamped=0`
streaming at ~1 Hz. Wiggle the handle; clamps should fire on fast motion.

### Live
```
python bridge.py
```
Default IP `192.168.58.2`. Override with `--serial /dev/ttyACM0` if needed.

## Safety checklist — run through every session

1. Robot **E-stop within reach**.
2. Arm in a pose with ≥150 mm clearance in every direction from fixtures.
3. First live test: `config.yaml` → `scale_xyz: 0.1`, `workspace_box_mm: [50, 50, 50]`.
4. Handle **stationary** when you start the bridge. On `ServoMoveStart` the
   arm must not move — if it does, stop and investigate before proceeding.
5. Nudge handle 10 mm in +X → arm should move 1 mm in base +X. If direction
   is wrong, flip a sign in the firmware `ENC_SIGN[]` or add a mapping
   rotation to a future config field.
6. Press `z` on the Teensy (or send `'z'` over the serial monitor) to
   re-anchor without moving the arm.
7. Only then raise `scale_xyz` and widen the workspace box.

## Safety invariants enforced by the code

- **No startup jump** — first `ServoCart` target equals the robot's current
  TCP pose (delta = 0). See `bridge.py:run()` after `preflight()`.
- **Bounded velocity** — per-cycle delta clamp in `safety.clamp_delta`.
  Default 2 mm/cycle at 125 Hz = 250 mm/s ceiling.
- **Hard workspace box** — `safety.clamp_workspace` AABB around the anchor.
- **Stream-loss watchdog** — if no Teensy sample in 50 ms, the loop holds
  and treats recovery as a new anchor (no accumulated drift or jump).
- **Trilat-fail gate** — requires 2 consecutive clean samples after any
  trilateration failure before resuming motion.
- **Orientation unwrap** — firmware emits `rollCont`/`yawCont` unwrapped;
  bridge re-wraps deltas to ±180° around the anchor for clean IK.
- **Graceful shutdown** — SIGINT/SIGTERM trigger `StopMotion` → `ServoMoveEnd`.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `No Teensy found` | Device not enumerated | `ls /dev/serial/by-id/`; plug in or `--serial` |
| `no clean (trilat-OK) sample within 5 s` | Draw-wires slack / anchor geometry bad | Re-tension, re-verify `ANCHOR*` constants in `main.cpp` |
| `ServoCart err=<N>` | Robot refused target | Check pendant for faults, verify auto mode + enabled, consult Fairino error table |
| Clamps firing constantly | Handle noisy or scale too high | Lower `scale_xyz`; raise `ema_alpha_xyz` toward 0.15 |
| Arm drifts at rest | BNO055 yaw drift via rotation scale | Set `scale_rot: 0` for translation-only teleop |

## Firmware dependency

`main.cpp` must emit `>rezero:1` inside `captureZero()` (added alongside
this bridge). Without it, the bridge cannot distinguish a user-initiated
rezero from a normal sample.

## Tests

```
cd tests && python test_safety.py
```
