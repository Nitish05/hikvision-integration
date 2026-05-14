# painting_bridge

Real-time Cartesian streaming from the Teensy painting handle (BNO055 + 3×
draw-wire trilateration) to a Fairino FR5 via `ServoJ` + `GetInverseKinRef`.
The handle's 6-DOF pose drives the FR5 TCP at 100 Hz, anchored so the arm
never jumps on startup.

> For the operator-facing two-machine workflow (record on Linux with this
> bridge, play back on Windows with `hikvision.py`) see [`../HANDOFF.md`](../HANDOFF.md).
> For setup, network, and the full troubleshooting table see [`../README.md`](../README.md).

## Layout

| File | Role |
|---|---|
| `bridge.py` | Entry point. Orchestrates reader thread + servo loop + recorder hotkey. |
| `teensy_reader.py` | Serial auto-detect + Teleplot parser (`>x:`, `>q*:`, `>button:`, `>rezero:`, `>trilatFail:`). |
| `fr5_servo.py` | `fairino.Robot.RPC` wrapper + `MockRobot` for `--dry-run`. ServoJ path with `GetInverseKinRef` + fresh-joint retry. |
| `quat.py` | Quaternion math (Hamilton product, Tait-Bryan ⇄ quat, axis-angle scaling). Used to compose handle→TCP rotations in the body frame. |
| `safety.py` | EMA, per-axis delta clamp, workspace AABB + spherical reach gate. |
| `recording.py` | `TrajectoryRecorder` daemon thread; writes `recorder.py`-compatible `.txt` from cached state reads. |
| `config.yaml` | Tunables (scales, limits, filter alpha, recording period). |
| `tests/test_safety.py` | Unit tests for pure-function safety primitives. |
| `tests/test_recording.py` | Unit tests for the trajectory file writer (header + row formatting). |

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
Expect logs like `preflight OK`, `anchored`, and `sent=99 skipped=0 clamped=0`
streaming at ~1 Hz. Wiggle the handle; clamps should fire on fast motion.

### Live
```
python bridge.py
```
Robot IP is read from `config.yaml` (`robot.ip`, default `192.168.57.2` — the
SDK's factory default is `192.168.58.2`, this rig is on the .57 subnet).
Serial port is auto-detected by Teensy VID; override with `--serial /dev/ttyACM0`
if needed.

### CLI flags

| Flag | Default | Purpose |
|---|---|---|
| `--config <path>` | `painting_bridge/config.yaml` | Override config file |
| `--serial <dev>` | (auto-detect) | Force a specific serial device |
| `--dry-run` | off | Use `MockRobot` — no controller needed |
| `--log-level` | `INFO` | `DEBUG` / `INFO` / `WARNING` / `ERROR` |

### Recording a trajectory (in-session)

While the bridge is running, press **`r` + Enter** in the terminal to start
recording, **`r` + Enter** again to stop. Output lands at
`recordings/run_YYYYMMDD_HHMMSS.txt` in `recorder.py`-compatible format
(header + 15-field rows). The recorder is a daemon thread that samples
cached state — it never touches the servo loop. See the format spec below.

Files written here are designed to be played back from Windows via
`.venv/hikvision.py` (see [`../HANDOFF.md`](../HANDOFF.md)). Do **not** run
that GUI from Linux.

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

- **No startup jump** — first `ServoJ` target equals the IK of the robot's
  current TCP pose (delta = 0). See `bridge.py:run()` after `preflight()`.
- **Bounded velocity** — per-cycle delta clamp in `safety.clamp_delta`.
  Default `2.5` mm/cycle at 100 Hz = 250 mm/s ceiling
  (`safety.max_delta_mm_per_cycle` in `config.yaml`); `1.25` deg/cycle = 125 deg/s.
- **Hard workspace box** — `safety.clamp_workspace` AABB around the anchor
  (`safety.workspace_box_mm`, default ±400 mm per axis) plus a spherical
  reach gate (`safety.workspace_reach_radius_mm`, default 500 mm).
- **Stream-loss watchdog** — if no Teensy sample in 50 ms, the loop holds
  and treats recovery as a new anchor (no accumulated drift or jump).
- **Trilat-fail gate** — requires 2 consecutive clean samples after any
  trilateration failure before resuming motion.
- **Quaternion rotation composition** — handle→TCP rotations are composed
  as quaternions in the body frame at the TCP; Tait-Bryan angles are only
  computed at the final `ServoJ` send. See `quat.py` and the trade-off
  notes in `../README.md`.
- **Joint-delta clamp before ServoJ** — the IK output is rejected if any
  joint changes more than the configured limit per cycle (mitigates wrist
  singularity / IK flips that would otherwise trip the controller's
  "Shaft collision fault").
- **IK fresh-joint retry** — on `GetInverseKinRef` failure the wrapper
  retries once with a fresh `GetActualJointPosDegree()` seed before
  giving up; counters `retry_ok` / `retry_fail` appear in the stats line.
- **Graceful shutdown** — SIGINT/SIGTERM trigger `StopMotion` →
  `ServoMoveEnd` → `SetDO(0, 0)` (paint valve off).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `No Teensy found` | Device not enumerated | `ls /dev/serial/by-id/`; plug in or `--serial` |
| `no clean (trilat-OK) sample within 5 s` | Draw-wires slack / anchor geometry bad | Re-tension, re-verify `ANCHOR*` constants in `main.cpp` |
| `ServoJ err=112` | IK solver couldn't reach the target (most common with `use_servoj: true`) | Non-fatal; bridge holds. Re-home to mid-range; set `scale_rot: 0`; check the `retry_ok`/`retry_fail` counters |
| `ServoCart err=112` | Same as above, on the legacy `use_servoj: false` path | Switch to ServoJ in `config.yaml` (`use_servoj: true`) |
| `"Shaft collision fault"` on the pendant | Joint-space jump (IK flip / wrist singularity) | Mitigated by the joint-delta clamp before ServoJ; re-home off the singularity, raise `scale_rot` only after translation is dialled in |
| `err=-4` on every RPC | Network timeout / wrong route | `ping 192.168.57.2`; `ip route get 192.168.57.2` must go via the wired NIC |
| Clamps firing constantly | Handle noisy or scale too high | Lower `scale_xyz`; raise `ema_alpha_xyz` toward 0.15 |
| Arm drifts at rest | BNO055 yaw drift via rotation scale | Set `scale_rot: 0` for translation-only teleop |
| Arm moves violently during **playback** | You are running `hikvision.py` from Linux | E-stop. Move playback to Windows. See [`../HANDOFF.md`](../HANDOFF.md). |

## Firmware dependency

`main.cpp` must emit `>rezero:1` inside `captureZero()` (added alongside
this bridge). Without it, the bridge cannot distinguish a user-initiated
rezero from a normal sample.

## Tests

```
cd tests && python test_safety.py
cd tests && python test_recording.py
```
