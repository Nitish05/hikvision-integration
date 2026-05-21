# Fairino FR5 Painting Rig

> **New here?** Start with [`HANDOFF.md`](HANDOFF.md) — the two-machine cold-start workflow (Linux records, Windows plays back). This README is the reference manual that document leans on.

A hand-held painting handle (Teensy 4.1 + BNO055 IMU + 3× draw-wire encoders) teleoperates a **Fairino FR5** cobot in real time. The operator paints by hand; the arm mirrors the motion with a paint-valve solenoid driven from a handle-mounted switch. Trajectories can be recorded during a session and replayed autonomously on subsequent parts.

Target hardware: **Fairino FR5** cobot (controller FRC100-AC, firmware V3.8.7-QX) + **Teensy 4.1** handle + painting solenoid wired to FR5 DO0.

Current bridge input sources:

- **Teensy handle**: serial Teleplot stream from the physical painting handle.
- **Quest controller**: OpenVR pose stream using `+X` right, `+Y` away from the headset reference, and `+Z` up. The Quest trigger maps to the paint/solenoid button. Holding `B` or `Y` pauses motion; releasing it re-anchors control from that release pose.
- **Camera (AprilTag)**: a handheld moving AprilTag tracked relative to a fixed reference tag via a USB camera. The pose is tag-to-tag relative, so the camera can be bumped without disturbing control. There is no built-in trigger button; paint via the **spacebar** hotkey. On Windows, global Space is hold-to-spray even when the terminal is not focused. On Linux, use the focused-terminal Space toggle or a USB HID keypad. Full rig setup is in [`camera_tracking/README.md`](camera_tracking/README.md).

Both sources are treated as **input deltas**. The robot target is always based on the FR5 TCP pose captured at anchor time:

```
robot_target = robot_tcp_at_anchor + scale * input_delta_from_source_anchor
```

```
   ┌──────────────────────┐    USB     ┌───────────────┐  Ethernet   ┌─────────────┐
   │ Teensy 4.1 handle    │───────────▶│               │             │             │
   │ BNO055 + 3× encoders │   125 Hz   │               │             │             │
   │ pin-6 switch         │  Teleplot  │               │             │             │
   └──────────────────────┘            │               │             │             │
   ┌──────────────────────┐  OpenVR    │ Host bridge   │             │ FR5 cobot   │
   │ Quest controller     │───────────▶│ painting_     │────────────▶│ 192.168.57.2│
   │ pose + trigger + B/Y │            │ bridge/       │   ~100 Hz   │ DO0 → valve │
   └──────────────────────┘            │               │  ServoJ+IK  │             │
   ┌──────────────────────┐    USB     │               │             │             │
   │ USB camera +         │───────────▶│               │             │             │
   │ AprilTags (ref+move) │            │               │             │             │
   └──────────────────────┘            │               │             │             │
   ┌──────────────────────┐  USB HID   │               │             │             │
   │ USB keypad (QINIZX)  │───────────▶│  paint        │             │             │
   │ paint trigger only   │   evdev    │  trigger OR   │             │             │
   └──────────────────────┘            └───────────────┘             └─────────────┘
```

## Repo layout

```
hikvision_integration/
├── fairino/                    Fairino Python SDK (I/O fixes for V3.8.7-QX)
├── painting_firmware/          Teensy 4.1 handle firmware (PlatformIO)
│   ├── platformio.ini
│   └── src/main.cpp            IMU fusion, trilateration, button+rezero markers
├── camera_tracking/            AprilTag camera rig (calibration + detector)
│   ├── calibrate.py            Checkerboard intrinsic calibration
│   ├── apriltag_pose.py        Standalone two-tag pose viewer (for setup)
│   ├── tag_detector.py         AprilTag detection + solvePnP
│   ├── camera.py               Camera capture (v4l2/DirectShow auto-detect, exposure, threaded grab)
│   └── config.yaml             Device, intrinsics path, tag ids/sizes
├── painting_bridge/            Real-time streaming bridge (live teleop + recorder)
│   ├── bridge.py               Main entry point
│   ├── teensy_reader.py        Serial parser
│   ├── quest_reader.py         OpenVR / Quest controller input source
│   ├── camera_reader.py        AprilTag camera input source (uses ../camera_tracking/)
│   ├── keypad_reader.py        USB HID keypad paint-trigger (Linux-only, evdev)
│   ├── windows_hotkey_reader.py Global Space paint-trigger (Windows, pynput)
│   ├── one_euro.py             One Euro low-pass filter used by camera_reader.py
│   ├── fr5_servo.py            FR5 SDK wrapper + MockRobot for --dry-run
│   ├── quat.py                 Quaternion math for rotation composition / clamps
│   ├── safety.py               EMA, delta clamp, workspace box, reach gate
│   ├── recording.py            TrajectoryRecorder thread (recorder.py-compatible)
│   ├── config.yaml             Tunables (scales, limits, filter alpha, recording)
│   └── tests/                  Unit tests (safety, recording, Windows support)
├── recorder.py                 GUI recorder + player (drag-teach workflow)
├── io_monitor.py               Real-time digital-I/O monitor GUI
├── lua_manager.py              Upload/run Lua scripts on the controller
├── di_do_passthrough.lua       Robot-side DI→DO mirror (10 ms loop)
├── quest.py                    Standalone Quest/OpenVR viewport and coordinate debugger
├── requirements.txt            Python deps
└── recordings/                 Bridge output (auto-created; gitignored)
```

---

## Hardware

### FR5 controller I/O (NPN mode — factory default)

The controller uses **NPN digital inputs** (active LOW).

**Switch wired directly to a robot DI** (drag-teach workflow, recorder.py):
```
DI0 (bottom connector, pin 1 bottom row) ── Switch (NO) ── E-0V/GND (pin 5 bottom row)
```

**Paint solenoid on DO0**:
```
E-24V (pin 5 top row) ── Solenoid (+) ── Solenoid (-) ── DO0 (pin 1 top row)
```

DO0 sinks to GND when active (NPN output, max 1 A per channel). Use an external relay if the solenoid draws more than 1 A.

#### Controller I/O pinout (bottom connector)

| Pin | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|-----|---|---|---|---|---|---|---|---|---|---|
| Top row | DO0 | DO1 | DO2 | DO3 | E-24V | DO4 | DO5 | DO6 | DO7 | E-24V |
| Bottom row | DI0 | DI1 | DI2 | DI3 | E-0V | DI4 | DI5 | DI6 | DI7 | E-0V |

### Teensy 4.1 handle pinout

| Pin(s) | Role |
|---|---|
| 18 (SDA), 19 (SCL) | BNO055 IMU over I²C |
| 2, 3 | Encoder 1 (quadrature, hardware-decoded) |
| 4, 5 | Encoder 2 |
| 30, 31 | Encoder 3 |
| 6 | Momentary push-button → paint-valve trigger (INPUT_PULLUP; idle HIGH, pressed LOW) |

Three CALT CESI-S2000 draw-wire encoders, 2000 PPR × 4× quadrature = 8000 counts/rev, 200 mm/rev → 0.025 mm/count. Anchors form a fixed triangle around the handle's home pose; positions are trilaterated in firmware at 125 Hz.

---

## Host setup

### Prerequisites

- **Python 3.12** (the Fairino SDK's Cython build doesn't yet support 3.14+)
- Linux or Windows with git
- PlatformIO for firmware builds (via VS Code extension or `pip install platformio`)
- Optional but useful on fresh Windows machines: [`uv`](https://docs.astral.sh/uv/) for creating a Python 3.12 venv when `python` / `py` is not on PATH.

### Clone + venv

```bash
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration
```

If Python 3.12 is already installed and on PATH, create the venv normally:

```bash
# Linux
python3.12 -m venv .venv
source .venv/bin/activate

# Windows
py -3.12 -m venv .venv
.venv\Scripts\activate
```

If Windows does not recognize `python`, `python3`, or `py`, use `uv` to create a self-contained Python 3.12 venv in the repo:

```powershell
# From the repo root
$env:UV_CACHE_DIR=".uv-cache"
$env:UV_PYTHON_INSTALL_DIR=".uv-python"

uv venv --managed-python --python 3.12 .venv
.\.venv\Scripts\python.exe -m ensurepip --upgrade
.\.venv\Scripts\activate
```

The `UV_CACHE_DIR` and `UV_PYTHON_INSTALL_DIR` lines keep uv's generated files inside the repo. They are ignored by `.gitignore`.

### Python dependencies

Install the core dependencies once from the repo root:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

`requirements.txt` includes the bridge, camera, and platform-specific support:

- `opencv-python`, `numpy`, `pyyaml`, `matplotlib`, `pyserial`
- Linux only: `evdev` for the USB HID keypad paint trigger
- Windows only: `pynput` for global Space hold-to-spray, and `pygrabber` for DirectShow camera-name auto-detect

Quest/OpenVR support is optional and still installed separately:

```bash
pip install pygame openvr
```

If you created the venv with `uv`, you can install everything without relying on `pip` first:

```powershell
$env:UV_CACHE_DIR=".uv-cache"
$env:UV_PYTHON_INSTALL_DIR=".uv-python"
uv pip install --python .venv\Scripts\python.exe -r requirements.txt pygame openvr
.\.venv\Scripts\python.exe -m ensurepip --upgrade
```

Verify the install:

```bash
python -c "import cv2, numpy, matplotlib, serial, yaml; print('core imports ok')"
python -c "import pygame, openvr; print('quest imports ok')"  # only if Quest deps were installed
python -m py_compile quest.py painting_bridge/bridge.py
```

### Install the Fairino SDK

Option A — editable install (needs Cython + C compiler):
```bash
pip install cython
pip install -e ./fairino
```

Option B — skip install; scripts import `fairino/` directly from the repo root. Simpler; used by the bridge by default.

Verify:
```bash
python -c "from fairino import Robot; r = Robot.RPC('192.168.57.2'); print(r.GetSDKVersion())"
```

---

## Network setup

The bridge and `recorder.py` need direct Ethernet to the controller at `192.168.57.2` on the `192.168.57.0/24` subnet. The ~100 Hz servo loop + state stream at port 20004 require wired latency (≤1 ms). WiFi is fine for pendant/setup, **not** for `ServoJ` streaming.

### Persistent NetworkManager connection (recommended)

Creates a dedicated ethernet profile that coexists with your WiFi (`ipv4.never-default` prevents it stealing the default route):

```bash
sudo nmcli con add type ethernet ifname enp7s0 con-name fr5 \
    ipv4.method manual ipv4.addresses 192.168.57.10/24 \
    ipv4.never-default yes ipv6.method disabled
sudo nmcli con up fr5
```

Replace `enp7s0` with your actual interface (`ip link`).

### Verify connectivity

```bash
ping -c 3 192.168.57.2          # ~0.2 ms RTT over wired
ip route get 192.168.57.2       # must go via enp7s0, not wlan0
```

If routing leaks to WiFi, you'll see `err=-4 No route to host` on every RPC call.

### Windows PowerShell Ethernet setup

Run PowerShell as Administrator. First find the Ethernet adapter plugged into
the FR5:

```powershell
Get-NetAdapter | Format-Table Name, InterfaceDescription, Status, LinkSpeed
```

Then replace `Ethernet` with that adapter name:

```powershell
$if = "Ethernet"
Get-NetIPAddress -InterfaceAlias $if -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Format-Table InterfaceAlias, IPAddress, PrefixLength
ping 192.168.57.2
```

If the adapter already has an address like `192.168.57.x` and the ping works,
do not change anything. If it has no IPv4 address, a `169.254.x.x` address, or
ping fails, configure the static robot subnet:

```powershell
$if = "Ethernet"

Get-NetIPAddress -InterfaceAlias $if -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Remove-NetIPAddress -Confirm:$false
Get-NetRoute -InterfaceAlias $if -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Remove-NetRoute -Confirm:$false

New-NetIPAddress -InterfaceAlias $if -IPAddress 192.168.57.11 -PrefixLength 24
Set-NetIPInterface -InterfaceAlias $if -Dhcp Disabled
Disable-NetAdapterBinding -Name $if -ComponentID ms_tcpip6

ping 192.168.57.2
Test-NetConnection 192.168.57.2 -Port 20003
```

Do not add a default gateway on the FR5 Ethernet adapter. Leaving the gateway
blank keeps normal internet traffic on WiFi while robot traffic goes over
Ethernet.

---

## Firmware flashing (painting_firmware/)

PlatformIO builds and uploads:

```bash
cd painting_firmware
pio run -t upload      # compiles and flashes the Teensy over USB
pio device monitor     # optional: view Teleplot stream
```

VS Code with the PlatformIO extension works identically — just open the `painting_firmware/` folder.

### Serial console commands (send over USB)

| Key | Effect |
|---|---|
| `z` | Re-zero: capture current handle orientation and encoder baselines as the new origin. Emits `>rezero:1` so the bridge re-anchors without moving the arm. |
| `r` | Clear IMU zero (reverts to absolute-orientation display) |
| `e` | Zero encoders only (leaves IMU zero intact) |

### Output format (Teleplot)

```
>x:243.42        Cartesian position (mm), trilaterated from 3 encoders
>y:168.89
>z:526.28
>rx:0.03         IMU Euler (deg), relative to last zero
>ry:-0.01
>rz:-2.16
>dt_ms:8
>trilatFail:N    (1 Hz) cumulative trilateration failures
>rezero:1        on every z-press
>button:1 / :0   on debounced edge of pin 6 switch
```

---

## Running the bridge (live teleop)

```bash
source .venv/bin/activate
cd painting_bridge
python bridge.py
```

When `--source` is omitted, `bridge.py` asks at runtime:

```text
Select control source:
  1. Teensy handle
  2. Quest controller
  3. Camera (AprilTag)
Press Enter for Teensy.
>
```

You can also bypass the prompt:

```bash
python bridge.py --source teensy
python bridge.py --source quest
python bridge.py --source camera
python bridge.py --dry-run --source camera
python bridge.py --list-keypads      # Linux diagnostic: list evdev devices and exit
```

Windows PowerShell examples:

```powershell
.\.venv\Scripts\python.exe .\painting_bridge\bridge.py --dry-run
.\.venv\Scripts\python.exe .\painting_bridge\bridge.py --dry-run --source quest
.\.venv\Scripts\python.exe .\painting_bridge\bridge.py --dry-run --source camera
```

**Pre-flight checklist:**
1. **E-stop within reach.** Always.
2. Move the arm to a joint-mid-range pose via the pendant (J1~0°, J2~−60°, J3~90°, J4~0°, J5~90°, J6~0°). Avoid any joint within 10° of its limit — this alone eliminates most IK failures.
3. Hold the handle **stationary**. The bridge's first command equals the current TCP pose (zero delta), so the arm must not move on `ServoMoveStart`. If it does, E-stop and investigate.

Expected startup log:
```
using serial port: /dev/serial/by-id/usb-Teensyduino_USB_Serial_...
serial opened: ... @ 115200
connected to FR5 at 192.168.57.2
preflight OK: tool_id=0 tcp=[...] joints=[...]
anchored: tcp_start=[...] handle_ref=[...]
sent=99 skipped=0 clamped=0 ik_fail=0 hold=0 retry_ok=0 retry_fail=0 target=[...]
```

### 1 Hz stats line

| Field | Meaning |
|---|---|
| `sent` | ServoJ commands successfully issued this second |
| `skipped` | cycles missed (stream-loss watchdog) |
| `clamped` | cycles where workspace-box or delta-clamp kicked in |
| `ik_fail` | cumulative IK-solver failures (non-fatal) |
| `hold` | cycles where we held position (trilat/IK/reach recovery) |
| `retry_ok` | IK failures rescued by the fresh-joint retry |
| `retry_fail` | IK failures that couldn't be rescued (target truly unreachable) |

### Solenoid trigger

The solenoid is driven by the **OR** of independent sources. Any one ON drives `DO0` ON:

- **Teensy pin-6 switch** (handle source only): debounced edge from firmware.
- **Linux/macOS stdin Space hotkey** (any source): focused-terminal SPACE toggles the solenoid request without needing Enter. The terminal must be focused. Restored on exit; skipped cleanly when stdin is not a TTY (`nohup`, systemd, piped). `r` in the same mode toggles the recorder.
- **Windows global Space hotkey** (any source): `pynput` listens system-wide, so Space is **hold-to-spray** even when another window is focused. Events are not suppressed, so the focused app still receives normal Space input.
- **USB HID keypad** (Linux-only via `evdev`): a macro keypad whose key feeds the same solenoid request, momentary by default (hold-to-spray). Enabled by default in `config.yaml`; matches the QINIZX 2-key by VID:PID `0x8808:0x6601`. The reader `grab()`s the device exclusively so its keys do **not** leak into the focused window, which means it works without terminal focus. On Windows or hosts without `evdev` the reader is a silent no-op.

`SetDO(0, 1)` / `SetDO(0, 0)` is edge-triggered, not per-cycle. Valve closes automatically on `Ctrl-C`.

Discover what device path/name to set as `keypad.match_name` (or to confirm the VID:PID match) by running:

```bash
python bridge.py --list-keypads
```

This prints every visible evdev device (path, name, VID:PID, KEY_SPACE capability) and exits.

### Re-anchoring mid-session

Press `z` on the Teensy serial console → firmware emits `>rezero:1` → bridge re-captures `tcp_start = current TCP` and `handle_ref = current handle pose` atomically. The arm doesn't move; handle→arm mapping is recentered.

### Quest source behavior

Use `--source quest` or choose `2` from the startup menu. SteamVR/OpenVR must be running, and the headset plus selected controller must be awake and tracked.

The Quest coordinate frame matches `quest.py`:

| Axis | Meaning |
|---|---|
| `+X` | right from the headset reference |
| `+Y` | away/forward from the headset reference |
| `+Z` | up |

The first valid headset pose locks the frame, and the first selected-controller pose becomes the controller anchor. The bridge receives only controller deltas from that anchor, then adds those deltas to the robot TCP anchor. It does not send raw Quest world coordinates to the robot.

Quest trigger maps to the bridge button/solenoid. Holding `B` or `Y` pauses motion; moving the controller while held does not move the robot. Releasing `B` or `Y` re-anchors the controller and robot TCP at the release pose, so control resumes without a catch-up move.

### Camera (AprilTag) source behavior

Use `--source camera` or choose `3` from the startup menu. Requires a calibrated USB camera plus a printed and measured pair of AprilTags (one **reference**, laid flat; one **moving**, held in hand). Camera rig setup — checkerboard calibration, tag generation, tag-edge measurement, and reference-tag placement — lives in [`camera_tracking/README.md`](camera_tracking/README.md); the bridge reads camera device, intrinsics, and tag ids/sizes from `camera_tracking/config.yaml` (pointed to by `camera.ct_config`).

Coordinate convention is set by where the reference tag is laid:

| Axis | Meaning |
|---|---|
| `+X` | left/right along the reference tag |
| `+Y` | away from the operator |
| `+Z` | up (perpendicular to the tag) |

Because the moving-tag pose is reported **relative to the reference tag**, it is invariant to where the camera sits. The camera can be bumped or moved between runs without disturbing the control mapping. Only the reference-tag placement matters.

Re-anchor: the camera source has no button. Briefly cover the moving tag with your hand for longer than `safety.stream_timeout_ms` — the bridge holds the robot, and when the tag reappears both the moving-tag anchor and the robot TCP anchor refresh atomically. No catch-up jump. "Cover, reposition, uncover" is the rezero.

Knobs live under `camera:` in `painting_bridge/config.yaml` — `control_frame` (`base` maps tag axes to the FR5 base via `ref_to_base_yaw_deg`; `tcp` maps them to the tool's own axes frozen at the anchor), `invert_y/invert_z/invert_rot_y/invert_rot_z` (per-axis sign), `ref_to_base_yaw_deg` (`base` mode only), One Euro smoothing per channel, and `preview` (live OpenCV window with both tags, axes, and fps). The full per-knob walkthrough is in [`painting_bridge/README.md`](painting_bridge/README.md#camera-apriltag).

Paint trigger: the camera source has no handle button, so use the [spacebar hotkey](#solenoid-trigger). On Windows, global Space is hold-to-spray without terminal focus; on Linux/macOS, focused-terminal Space toggles, or use the Linux USB keypad for hands-free hold-to-spray.

### Shutdown

`Ctrl-C` → `StopMotion` → `ServoMoveEnd` → solenoid OFF → clean exit.

---

## Recording a trajectory

While the bridge is running, **press `r` + Enter** in the terminal to start recording. Press `r` + Enter again to stop. Files land in `recordings/run_YYYYMMDD_HHMMSS.txt`.

The recorder is a daemon thread that samples cached robot state (`GetActualJointPosDegree`, `GetActualTCPPose` — both read from the SDK's `robot_state_pkg`, no per-row RPC) every `recording.period_s`. It never touches the servo loop.

### File format (recorder.py-compatible)

**Header** (fixed 10-digit zero-padded count + configuration):
```
0000003318,1,10,0,1,1
```
Fields: `N_points , 1 , period_ms , tool_id , diconfig_bits , doconfig_bits`

**Data rows** (one per sample, 15 comma-separated fields, `.4f` precision):
```
j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,5,trigger,17
```
Fields: six joints (deg), six Cartesian (mm/deg), FRTYPE=5 (FR5), trigger=DO0 state, trailer=17.

### Playback options

| Method | Timing | Host OS | Use case |
|---|---|---|---|
| **`hikvision.py` → Load → Start** (TPD on controller) | Controller-native, exact | **Windows only — see [`HANDOFF.md`](HANDOFF.md)** | **Production painting runs** |
| `recorder.py` → **Play (TPD)** | Controller-native, exact | Either (recorder.py is the drag-teach GUI) | Dev replay of a drag-teach recording |
| `recorder.py` → **Play (Servo+DO)** | Host-paced ~100 Hz | Either (best-effort) | Quick dev retest, not for production |

> **Hard rule:** `hikvision.py` playback is run **only from Windows**. Linux playback runs have caused unsafe FR5 motion (see [`HANDOFF.md`](HANDOFF.md#why-playback-from-linux-is-forbidden)). The bridge is the live teleop/recording path; keep production recording on the known-good host and do not substitute the Linux playback GUI path for it.

For TPD exact-timing playback via SDK (instead of the GUI):
```python
from fairino import Robot
r = Robot.RPC('192.168.57.2')
r.TrajectoryJUpLoad(filePath='./recordings/run_20260415_143000.txt')
# Then either use the hikvision.py GUI (Windows) or recorder.py → Load → Play (TPD).
```

See **[painting_bridge/README.md](painting_bridge/README.md)** for in-depth bridge docs and the [full file-format spec](painting_bridge/README.md#using-it). See **[`HANDOFF.md`](HANDOFF.md)** for the end-to-end record-on-Linux / play-on-Windows workflow.

---

## Configuration reference (`painting_bridge/config.yaml`)

> **Solenoid sources are OR'd**: Teensy pin-6 switch ∪ stdin spacebar toggle ∪ USB keypad. Any one ON drives `DO0` ON. The `solenoid.enabled` flag gates all three.

| Key | Default | Notes |
|---|---|---|
| `serial.port` | `auto` | Auto-detects Teensy on Windows COM ports and Linux/macOS serial devices; explicit examples: `COM5`, `/dev/ttyACM0` |
| `quest.controller` | `right` | Quest controller role: `right`, `left`, or `first` |
| `quest.wait_timeout_s` | `60.0` | Startup wait for first clean Quest/OpenVR sample |
| `camera.ct_config` | `../camera_tracking/config.yaml` | Path (relative to `painting_bridge/`) to the camera + tag settings file |
| `camera.preview` | `true` | Live OpenCV preview window: both tags, axes, fps. Set `false` for headless runs |
| `camera.control_frame` | `tcp` | `tcp` = tag X/Y/Z follow the tool's own axes (frozen at anchor); `base` = tag axes follow the FR5 base via `ref_to_base_yaw_deg` |
| `camera.invert_y` / `invert_z` | `true` / `true` | Negate the Y / Z translation (tag laid flat Z-up vs tool pointing down — lift the tag to raise the TCP) |
| `camera.invert_rot_y` / `invert_rot_z` | `true` / `true` | Reverse the rotation direction about each axis |
| `camera.swap_xy` | `false` | Swap X / Y position axes; superseded by `ref_to_base_yaw_deg` — leave `false` |
| `camera.ref_to_base_yaw_deg` | `0.0` | `base` mode only — yaw (about Z, CCW positive) from the reference-tag frame to the FR5 base. `0` = tag parallel to base |
| `camera.smoothing.enabled` | `true` | One Euro low-pass on position + rotation. Set `false` to publish the raw `solvePnP` pose |
| `robot.ip` | `192.168.57.2` | Override for different controllers |
| `robot.use_servoj` | `true` | Use ServoJ+IK (proven); `false` reverts to ServoCart (hit err=112) |
| `robot.cmd_period_s` | `0.010` | Declared cmdT; match loop_period_s |
| `robot.loop_period_s` | `0.010` | Host servo loop target (100 Hz) |
| `mapping.scale_xyz` | `1.2` | Input mm → TCP mm. Bumped from `1.0` after camera-source tuning; lower if `clamped` is firing constantly |
| `mapping.scale_rot` | `0.4` | Set to `0` for translation-only teleop (avoids wrist singularities) |
| `safety.max_delta_mm_per_cycle` | `2.5` | 250 mm/s ceiling at 100 Hz |
| `safety.max_delta_deg_per_cycle` | `1.25` | 125 deg/s ceiling at 100 Hz |
| `safety.workspace_box_mm` | `[400, 400, 400]` | Axis-aligned cube (± per axis) around `tcp_start` |
| `safety.workspace_rot_deg` | `[90, 90, 90]` | Per-axis rotation envelope around the anchor orientation |
| `safety.workspace_reach_radius_mm` | `500` | Spherical reach gate |
| `safety.stream_timeout_ms` | `50` | Reader-silence watchdog |
| `safety.trilat_recover_samples` | `2` | Clean samples required after a trilat fail |
| `filter.ema_alpha_xyz` | `0.35` | Lower = more smoothing / more lag |
| `solenoid.enabled` | `true` | Master gate; `false` ignores all three trigger sources |
| `solenoid.do_id` | `0` | FR5 DO channel |
| `keypad.enabled` | `true` | Linux-only USB HID paint trigger. Silent no-op on Windows / hosts without `evdev` |
| `keypad.match_name` | `null` | Optional substring match on evdev device name; VID:PID is preferred |
| `keypad.vendor_id` / `product_id` | `0x8808` / `0x6601` | QINIZX 2-key default. Discover yours with `python bridge.py --list-keypads` |
| `keypad.grab` | `true` | Exclusive grab so keys do **not** leak to the focused window |
| `keypad.solenoid_key` / `recorder_key` | `KEY_SPACE` / `KEY_SPACE` | Both physical QINIZX keys are remapped to space in firmware; the solenoid branch wins for the shared code |
| `keypad.momentary` | `true` | `true` = hold-to-spray (paint-gun style); `false` = press-to-toggle (same as stdin spacebar) |
| `recording.enabled` | `true` | Disable `r` hotkey |
| `recording.period_s` | `0.010` | 100 Hz; use `0.008` to match `recorder.py` CYCLETIME |

## Configuration reference (`camera_tracking/config.yaml`)

These settings are used by `camera_tracking/calibrate.py`,
`camera_tracking/apriltag_pose.py`, and the bridge camera source.

| Key | Default | Notes |
|---|---|---|
| `camera.device` | `auto` | Linux: match the Arducam OV9281 through `v4l2-ctl`. Windows: list DirectShow device names and prefer external-looking cameras such as Arducam, OV9281, USB, Webcam, Logitech, or C920. Set a `/dev/videoN` path or Windows index if you need to force a device |
| `camera.width` / `height` | `1280` / `800` | Requested capture size. Windows drivers may negotiate a nearby mode; check the startup log |
| `camera.fps` | `120` | Requested frame rate |
| `camera.fourcc` | `MJPG` | Requests MJPEG so high-FPS USB capture is practical |
| `camera.manual_exposure` | `true` | Locks exposure for sharper moving tags |
| `camera.exposure_time` | `15` | Linux `v4l2-ctl` `exposure_time_absolute` units |
| `camera.windows_auto_exposure` | `0.25` | OpenCV/DirectShow manual-exposure mode value |
| `camera.windows_exposure` | `-6` | Windows UVC exposure value. Tune on the real Arducam: more negative usually means shorter exposure/darker image |
| `checkerboard.cols` / `rows` | `8` / `6` | Inner checkerboard corners for intrinsic calibration |
| `checkerboard.square_mm` | `22.83` | Measured square size; wrong value scales all pose distances |
| `calibration.path` | `calibration.npz` | Written by `calibrate.py`, required before bridge camera source can publish samples |
| `apriltag.family` | `36h11` | AprilTag family |
| `apriltag.reference` / `moving` | ids `0` / `1` | Tag ids and measured outer black-square edge sizes in mm |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `No Teensy serial port found` | Device not enumerated | Check Device Manager or `ls /dev/serial/by-id/`; plug in or pass `--serial COM5` / `--serial /dev/ttyACM0` |
| `no sample from quest within ...` | SteamVR/OpenVR is not publishing tracked poses | Open SteamVR, wake headset and controller, verify `quest.py` shows the controller |
| `waiting for Quest headset pose from OpenVR` | Headset not awake/tracked | Wake headset and confirm SteamVR tracking |
| `waiting for Quest controller pose from OpenVR` | Controller asleep, wrong hand selected, or not tracked | Wake controller; set `quest.controller` to `left`, `right`, or `first` |
| `SDK连接机器人实时端口失败 [Errno 113] No route to host` | Ethernet not on 192.168.57.0/24 | Verify `ip route get 192.168.57.2`; bring up the `fr5` NM connection |
| `err=-4` on every RPC | Network timeout (bad route / firewall / robot off) | `ping 192.168.57.2` must succeed before anything else |
| `ServoCart err=112` / `ServoJ err=112` | IK solver can't find a solution | Non-fatal; bridge holds. Re-home arm to joint-mid-range pose; set `scale_rot: 0` for first runs |
| `ik_fail` counter climbing | Target outside reachable envelope or past joint limit | Pull handle back toward anchor pose; re-anchor with `z` |
| Clamps firing constantly (`clamped` high) | Handle noisy or scale too high | Lower `scale_xyz`; raise `ema_alpha_xyz` toward 0.15 |
| Arm drifts at rest | BNO055 yaw drift via `scale_rot > 0` | Set `scale_rot: 0` for translation-only teleop |
| `trilatFail` increments | Draw-wires slack or anchors moved | Re-tension; re-verify `ANCHOR*` constants in `main.cpp` |
| Sudden arm lunge after IK recovery | Operator moved handle during IK-hold, arm catches up at max velocity | See "Movement smoother after IK recovery" recommendation (re-anchor-on-resume pattern) |
| Motion seems ~25 % slower than recording | Recorded at rate > playback ceiling | Keep `recording.period_s` at `0.010` for `recorder.py`'s `Play (Servo+DO)`; or use TPD for exact timing |
| Camera source never anchors / `no AprilTag detected` | Tag occluded, lighting too low, wrong tag IDs/sizes, or missing calibration | Verify both tags detect with `camera_tracking/apriltag_pose.py`; re-check tag IDs/sizes in `camera_tracking/config.yaml`; recalibrate if intrinsics changed |
| `camera reader: ... calibration.npz not found` | Intrinsic calibration has not been run for this camera/resolution | Run `python camera_tracking/calibrate.py`; bridge camera mode cannot publish samples until `camera_tracking/calibration.npz` exists |
| Windows opens the built-in camera instead of the USB camera | DirectShow device name did not look external, or `camera.device` forced the wrong index | Leave `camera.device: auto` first and read the `DirectShow devices:` log. If needed, set `camera.device` to the printed Arducam/USB index |
| Arducam image is blurry or too dark on Windows | `windows_exposure` needs tuning, or lighting is too low for short exposure | Start at `windows_exposure: -6`; try `-7`, `-8`, `-9` for less blur, or add light / try `-5` if too dark |
| Camera axis is mirrored / arm goes the wrong way on one axis | `camera.invert_*` doesn't match the reference-tag placement (or `ref_to_base_yaw_deg` in `base` mode) | Twist the moving tag along each axis individually; flip the matching `invert_*`, or tune `ref_to_base_yaw_deg` |
| Camera source jitters at rest | One Euro smoothing too open | Lower `camera.smoothing.position.min_cutoff`; raise `beta` only if motion lag becomes noticeable |
| `keypad: no matching device found` | Wrong VID:PID, evdev not installed, or user not in `input` group | `pip install evdev`; `python bridge.py --list-keypads`; `sudo usermod -aG input $USER` then log out / log back in |
| Spacebar hotkey does nothing on Linux/macOS | Terminal not focused, stdin not a TTY (piped/redirected, `nohup`, systemd) | Focus the terminal or use the Linux USB keypad |
| Spacebar hotkey does nothing on Windows | `pynput` missing or blocked by OS/security software | `pip install -r requirements.txt`; check logs for `Windows global SPACE hotkey enabled` |

---

## Other tools

### `recorder.py` — GUI recorder / player (drag-teach workflow)

Freedrive the arm by hand, capture the trajectory in memory, save + play back. Independent of the bridge — useful for "arm as its own input device" workflows.

```bash
python recorder.py
```
- **Start Tracking** → robot enters freedrive (drag teaching) mode
- Drag the robot along the desired path; press the wired DI0 switch when you want the solenoid to fire
- **Stop Tracking** → 3D plot shows the path
- **Save** → writes TPD-format file and uploads to robot
- **Play (TPD)** / **Play (Servo+DO)** → replay

Robot must be in Auto mode for playback; the app switches modes automatically.

### `io_monitor.py` — live digital-I/O monitor

```bash
python io_monitor.py
```
Colored indicators for all 16 DI, 2 Tool DI, 16 DO, 2 Tool DO. Toggle buttons for DOs. "DI→DO passthrough" checkbox wires DI0 directly to DO0 for bench tests.

### `lua_manager.py` — upload/run Lua scripts on the controller

```bash
python lua_manager.py upload   # upload di_do_passthrough.lua
python lua_manager.py start    # run it (robot must be in Auto mode)
python lua_manager.py status
python lua_manager.py stop
python lua_manager.py list
```

Included `di_do_passthrough.lua` mirrors DI0 → DO0 every 10 ms on the controller, independent of any Python app. Useful when you want the solenoid to follow a physical switch without a host in the loop.

---

## Fairino SDK modifications

`fairino/Robot.py` has been modified to fix I/O reading on firmware V3.8.7-QX:

- **`GetDO()` / `GetToolDO()`** — removed `@xmlrpc_timeout` decorator that blocked reads when `RPC.is_conect` was `False` (set during SDK init timeout). Methods still read from the `robot_state_pkg` cache.
- **`GetDI()` / `GetToolDI()`** — kept reading from the state package (not XML-RPC). The XML-RPC version returned programmatic state only, not real hardware state.

The SDK's `RobotStatePkg` ctypes struct (1236 bytes) is larger than the actual port-20004 packet (953 bytes); the checksum validates on actual received length, so I/O fields still read correctly.

---

## License / attribution

Fairino FR5 SDK by Fairino (original license in `fairino/`). Integration code and firmware by this repo's authors. Contributions welcome via PR.
