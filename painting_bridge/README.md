# painting_bridge

Real-time Cartesian streaming bridge for the Fairino FR5 painting rig.

The bridge can use any of three input sources:

- **Teensy handle**: BNO055 + 3 draw-wire encoders over USB serial.
- **Quest controller**: OpenVR pose stream from a Quest controller, using the same controller-coordinate convention as `quest.py`.
- **Camera (AprilTag)**: a handheld AprilTag tracked relative to a fixed reference tag (see `../camera_tracking/`).

All sources feed the same servo loop. The bridge anchors the robot at its current TCP pose, treats the input device as a delta source, and sends `tcp_start + input_delta` through `ServoJ` + `GetInverseKinRef`.

For the operator-facing two-machine workflow, see [`../HANDOFF.md`](../HANDOFF.md). For setup, network, hardware pinout, and broader troubleshooting, see [`../README.md`](../README.md).

## Layout

| File | Role |
|---|---|
| `bridge.py` | Main entry point. Prompts for input source, starts the reader, runs the servo loop, handles recording and shutdown. |
| `teensy_reader.py` | Serial auto-detect + Teleplot parser for the Teensy handle. Auto-detect works on Windows COM ports and Linux/macOS serial devices. |
| `quest_reader.py` | OpenVR reader for Quest controller poses. Converts controller data to bridge `HandleSample` deltas. |
| `camera_reader.py` | AprilTag reader. Tracks a handheld tag relative to a fixed reference tag (uses `../camera_tracking/`) and publishes `HandleSample`. |
| `windows_hotkey_reader.py` | Windows global Space listener. Space is hold-to-spray for the solenoid even when the terminal is not focused. |
| `fr5_servo.py` | `fairino.Robot.RPC` wrapper + `MockRobot` for `--dry-run`. Uses ServoJ with controller-side IK via `GetInverseKinRef`. |
| `quat.py` | Quaternion math for bridge rotation composition and clamps. |
| `safety.py` | Position EMA, workspace clamp, per-cycle delta clamp, reach gate, finite-value checks. |
| `recording.py` | Background trajectory recorder. Writes recorder.py-compatible `.txt` files from cached robot state. |
| `config.yaml` | Tunables for serial, Quest, robot, mapping, safety, filtering, solenoid, and recording. |
| `tests/` | Unit tests for safety and recording helpers. |

## Install

Minimum bridge dependencies:

```bash
# From the repo root, not from painting_bridge/
pip install -r requirements.txt
```

Quest input also needs the OpenVR Python package and SteamVR/OpenVR running:

```bash
pip install openvr
```

Windows camera support uses `pygrabber` to list DirectShow camera names and
`pynput` for the global Space trigger; both are installed by `requirements.txt`
on Windows. Linux keypad support uses `evdev`, installed by `requirements.txt`
only on Linux.

The Fairino SDK is already vendored in `../fairino/`; `fr5_servo.py` imports it from the repo root.

## Running

From the repo root on Windows:

```powershell
.\.venv\Scripts\python.exe .\painting_bridge\bridge.py --dry-run
```

From inside `painting_bridge/`:

```powershell
..\.venv\Scripts\python.exe .\bridge.py --dry-run
```

On Linux/macOS with the venv activated:

```bash
cd painting_bridge
python bridge.py --dry-run
```

When `--source` is omitted, the bridge asks at runtime:

```text
Select control source:
  1. Teensy handle
  2. Quest controller
  3. Camera (AprilTag)
Press Enter for Teensy.
>
```

Accepted choices:

- `Enter`, `1`, `t`, `teensy`
- `2`, `q`, `quest`
- `3`, `c`, `camera`

You can bypass the prompt:

```bash
python bridge.py --dry-run --source teensy
python bridge.py --dry-run --source quest
python bridge.py --dry-run --source camera
```

Live robot mode is the same command without `--dry-run`:

```bash
python bridge.py --source teensy
python bridge.py --source quest
python bridge.py --source camera
```

## Source Behavior

### Teensy Handle

The Teensy firmware emits Teleplot-style serial keys:

```text
>x:...
>y:...
>z:...
>qw:...
>qx:...
>qy:...
>qz:...
>button:0/1
>rezero:1
>trilatFail:N
```

`teensy_reader.py` publishes complete samples after it has received `x/y/z` and `qw/qx/qy/qz`. The pin-6 switch maps to the bridge button/solenoid state. A firmware rezero (`z` sent to the Teensy serial console) marks the next sample with `rezero=True`, causing the bridge to re-anchor without moving the robot.

Serial auto-detect now uses PySerial `list_ports` first, so it works on:

- Windows: `COM5`, `COM6`, etc.
- Linux/macOS: `/dev/ttyACM0`, `/dev/cu.usbmodem...`, etc.
- Linux stable fallback: `/dev/serial/by-id/*`

Override auto-detect if needed:

```bash
python bridge.py --source teensy --serial COM5
python bridge.py --source teensy --serial /dev/ttyACM0
```

### Quest Controller

`quest_reader.py` reads OpenVR poses directly. SteamVR must be running, and the headset plus selected controller must be awake/tracked.

Coordinate convention:

- `+X`: right from the headset reference frame
- `+Y`: away/forward from the headset reference frame
- `+Z`: up

Startup behavior:

1. The first valid headset pose locks a headset reference frame.
2. The first valid selected-controller pose becomes the controller anchor.
3. The reader publishes controller deltas from that anchor, not raw Quest coordinates.
4. The bridge adds those deltas to the robot TCP anchor, just like the Teensy source.

This is important: the robot target is not `QuestXYZ`. It is:

```text
robot_target = robot_tcp_at_anchor + scale_xyz * (quest_controller_now - quest_controller_anchor)
```

Quest trigger maps to the bridge button/solenoid state.

B/Y reset behavior:

- Hold `B` or `Y`: Quest samples are marked not clean, so the bridge holds the robot.
- Move while holding `B/Y`: the robot still does not move.
- Release `B/Y`: the Quest controller anchor resets to the release pose, and the bridge re-anchors the robot TCP without a jump.

Quest config:

```yaml
quest:
  controller: right        # right, left, or first tracked controller
  wait_timeout_s: 60.0     # time allowed for OpenVR to publish first poses
```

If startup is waiting on tracking, logs will say whether it is waiting for headset pose or controller pose.

### Camera (AprilTag)

`camera_reader.py` tracks a handheld **moving** AprilTag relative to a fixed
**reference** AprilTag, using the calibration and detector in
`../camera_tracking/`. Camera device, intrinsics, and the tag ids/sizes are
read from `../camera_tracking/config.yaml` (pointed to by `camera.ct_config`).

Camera auto-detect is platform-specific:

- Linux: `camera.device: auto` matches the Arducam OV9281 through `v4l2-ctl`.
- Windows: `camera.device: auto` lists DirectShow camera names and prefers
  external-looking devices such as Arducam, Logitech, USB, or Webcam entries
  over built-in Surface/Intel cameras. Set `camera.device: 2` (or another
  index) to force a camera.

Manual exposure is platform-specific too. Linux uses `exposure_time` in
`v4l2-ctl` units. Windows uses DirectShow/OpenCV UVC values via
`windows_auto_exposure` and `windows_exposure`; tune `windows_exposure` on the
actual Arducam if the tags blur or the image is too dark.

The camera open log is the source of truth. On Windows it prints all DirectShow
devices and the chosen index, then the negotiated resolution/FPS. With the
Arducam, keep `fourcc: MJPG` so high-FPS USB capture uses MJPEG.

Coordinate convention — set by where the reference tag is placed:

- `+X`: left/right     `+Y`: away from the operator     `+Z`: up

Because the pose is tag-to-tag *relative*, it is invariant to camera position
— the camera can be bumped or moved without disturbing the output.

Setup, in order:

1. Calibrate the camera and verify tracking with `camera_tracking/` first
   (`calibrate.py`, then `apriltag_pose.py`).
2. Lay the **reference tag flat, face up**, rotated so its `+X/+Y` line up with
   the FR5 base `+X/+Y` (`+Z` is up for both). Confirm with `apriltag_pose.py`
   that moving the tag along robot `+X` raises X — *before* connecting the robot.
3. The robot target, as with the other sources:

   ```text
   robot_target = robot_tcp_at_anchor + scale_xyz * (moving_tag_now - moving_tag_anchor)
   ```

Re-anchor: there is no button. The reader emits a sample only when **both**
tags are cleanly detected. Briefly **cover the moving tag** (> the
`stream_timeout_ms` watchdog) — the bridge holds the robot and re-anchors when
the tag reappears, without a jump. "Cover, reposition, uncover" is the rezero.

The camera source has no physical button (`button` stays unset), so use the
keyboard trigger for paint. On Windows, global Space is hold-to-spray and works
even when the bridge terminal is not focused. Orientation direction reuses the
bridge's Quest-tuned mirror correction — if the wrist turns the wrong way, that
is the knob to revisit.

A live preview window (both tags, axes, reference-frame X/Y/Z, and fps) pops up
while the camera source runs; set `camera.preview: false` for a headless run.

The raw `solvePnP` pose jitters frame to frame, so `CameraReader` runs it
through a **One Euro filter** before publishing — an adaptive low-pass that
smooths hard when the tag is still and opens up when it moves (so it kills
jitter without adding lag during motion). Two knobs per channel: lower
`min_cutoff` = more smoothing of a still tag; higher `beta` = less lag on fast
motion. Position and rotation tune separately. Set `smoothing.enabled: false`
to publish the raw pose.

Camera config (in `painting_bridge/config.yaml`):

```yaml
camera:
  ct_config: ../camera_tracking/config.yaml   # camera + tag settings
  preview: true                               # live camera window with fps
  smoothing:
    enabled: true
    dcutoff: 1.0                              # Hz, derivative cutoff
    position: {min_cutoff: 1.0, beta: 0.05}   # mm channels
    rotation: {min_cutoff: 1.5, beta: 0.20}   # quaternion channels
```

## CLI Flags

| Flag | Default | Purpose |
|---|---|---|
| `--config <path>` | `painting_bridge/config.yaml` | Override config file. |
| `--serial <dev>` | config `serial.port` | Force Teensy serial port. Ignored for Quest/Camera sources. |
| `--source teensy\|quest\|camera` | interactive prompt | Select input source without prompting. |
| `--list-keypads` | off | Linux-only diagnostic. Print every visible evdev device (path, name, VID:PID, KEY_SPACE capability) and exit. Use this to set `keypad.match_name` or VID:PID when the default QINIZX `0x8808:0x6601` doesn't match. |
| `--dry-run` | off | Use `MockRobot`; no physical FR5 connection or motion. Still uses the selected input source. |
| `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING`, or `ERROR`. |

## Dry-Run

Dry-run replaces the physical robot with `MockRobot`. It is useful for checking:

- the selected source starts;
- samples arrive;
- anchoring and B/Y or Teensy rezero behavior work;
- recorder hotkey behavior;
- safety counters such as `clamped`, `hold`, and `skipped`.

Dry-run does not model real FR5 kinematics. The mock IK is only a placeholder for exercising the bridge loop. A stick-figure visualizer was briefly prototyped and then removed; dry-run now logs only.

Examples:

```bash
python bridge.py --dry-run --source teensy
python bridge.py --dry-run --source quest
```

## Recording

While the bridge is running on Linux/macOS in a focused terminal, press `r` to
start recording and press `r` again to stop:

```text
r  -> start recording
r  -> stop recording
```

The focused-terminal hotkey path is skipped on Windows; use Windows primarily
for camera dry-runs/verification and the known-good playback GUI path unless a
separate recorder trigger is added.

Files land in:

```text
painting_bridge/recordings/run_YYYYMMDD_HHMMSS.txt
```

The recorder samples cached robot state at `recording.period_s`. It does not command motion and does not block the servo loop.

The file format is recorder.py-compatible:

```text
header: N,1,period_ms,tool,diconfig,doconfig
rows:   j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,5,trigger,17
```

Recorded files are intended for Windows playback via `.venv/hikvision.py` or other known-safe TPD playback flow. Do not run the Linux playback GUI path on hardware.

## Safety Invariants

- **No startup jump**: the first robot target equals the current TCP pose.
- **Input-as-delta**: Teensy and Quest sources are both treated as deltas from their source anchor.
- **Re-anchor without motion**: Teensy `>rezero:1` and Quest B/Y release both refresh robot/input anchors before sending new motion.
- **Bounded velocity**: position and rotation deltas are clamped per cycle.
- **Workspace limits**: targets are clamped to the configured workspace box around `tcp_start`.
- **Reach gate**: targets outside `workspace_reach_radius_mm` are held rather than sent to IK.
- **Stream-loss watchdog**: stale input causes a hold and forces re-anchor on recovery.
- **Clean-sample gate**: Teensy trilat failures and Quest B/Y hold both pause motion.
- **Quaternion rotation path**: rotations are filtered/composed/clamped as quaternions until the final FR5 boundary.
- **Graceful shutdown**: `Ctrl-C`/SIGTERM runs `StopMotion`, `ServoMoveEnd`, and solenoid off.

## Stats Line

The bridge logs a 1 Hz line:

| Field | Meaning |
|---|---|
| `sent` | Number of robot commands successfully sent. |
| `skipped` | Cycles missed because of stream-loss/stale data. |
| `clamped` | Cycles where position or rotation target was limited by safety clamps. |
| `ik_fail` | IK failures returned by the FR5 SDK wrapper. |
| `hold` | Cycles where the bridge deliberately held position. |
| `retry_ok` | IK failures rescued by retrying with fresh joint state. |
| `retry_fail` | IK failures not rescued by retry. |
| `target` | Last attempted Cartesian target `[x, y, z, rx, ry, rz]`. |

`clamped` means the requested target was too far, too fast, or outside a configured rotation/workspace limit. Occasional clamps are normal during fast hand motion. Constant clamps mean the source scale or safety limits need attention.

## Configuration

Important keys in `config.yaml`:

| Key | Meaning |
|---|---|
| `serial.port` | `auto` or explicit serial port. Auto now works on Windows COM ports and Linux/macOS. |
| `camera_tracking.camera.device` | `auto`, Linux `/dev/videoN`, or Windows OpenCV camera index. Windows `auto` prefers external DirectShow camera names. |
| `quest.controller` | `right`, `left`, or `first tracked controller`. |
| `quest.wait_timeout_s` | Startup wait for first clean Quest sample. |
| `robot.ip` | FR5 controller IP. |
| `robot.use_servoj` | Keep `true`; ServoJ + IK is the proven path. |
| `robot.cmd_period_s` / `robot.loop_period_s` | Command and host loop period. Defaults target 100 Hz. |
| `mapping.scale_xyz` | Input millimeters to robot TCP millimeters. |
| `mapping.scale_rot` | Input rotation scale. Set `0` for translation-only testing. |
| `filter.ema_alpha_xyz` | Position smoothing alpha. |
| `filter.ema_alpha_rot` | Rotation slerp alpha. |
| `safety.max_delta_mm_per_cycle` | Per-cycle position speed clamp. |
| `safety.max_delta_deg_per_cycle` | Per-cycle rotation speed clamp. |
| `safety.workspace_box_mm` | Axis-aligned box around robot anchor. |
| `safety.workspace_rot_deg` | Rotation envelope around robot anchor. |
| `safety.workspace_reach_radius_mm` | Spherical target reach gate. |
| `safety.stream_timeout_ms` | Input silence watchdog. |
| `solenoid.enabled` / `solenoid.do_id` | Button-to-DO behavior. On Windows, global Space is hold-to-spray when enabled. |
| `recording.enabled` / `recording.period_s` | Recorder hotkey and output cadence. |

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `No Teensy serial port found` | Teensy not enumerated or wrong cable | Check Device Manager / `ls /dev/serial/by-id`; pass `--serial` explicitly. |
| `no sample from quest within ...` | SteamVR/OpenVR not publishing poses | Open SteamVR, wake headset/controllers, verify `quest.py` viewport works. |
| `waiting for Quest headset pose from OpenVR` | Headset not tracked | Wake headset, confirm SteamVR tracking. |
| `waiting for Quest controller pose from OpenVR` | Controller asleep or wrong selected hand | Wake controller; set `quest.controller: left`, `right`, or `first`. |
| Robot target jumps far on Quest source | Quest anchor was not reset where expected | Hold B/Y, move to neutral, release B/Y to re-anchor. |
| Reach gate warnings | Target outside `workspace_reach_radius_mm` from robot anchor | Re-anchor, reduce scale, or move robot to a more central pose. |
| `clamped` constantly increasing | Input asks for too much motion or rotation | Lower `scale_xyz`/`scale_rot`, move slower, or adjust safety limits carefully. |
| `ik_fail` climbing | Target unreachable or near joint limits | Re-home to mid-range; set `scale_rot: 0` for first tests. |
| Arm drifts at rest | Rotation source drift or noisy input | Set `scale_rot: 0`; increase smoothing; re-anchor. |
| Solenoid does not follow trigger | Wrong source button, `solenoid.enabled: false`, or DO wiring | Check logs for `solenoid -> ON/OFF`, verify `do_id`, test DO wiring. |

## Tests

```bash
cd painting_bridge
python -m py_compile bridge.py teensy_reader.py quest_reader.py
cd tests && python test_safety.py
cd tests && python test_recording.py
cd tests && python test_windows_support.py
```
