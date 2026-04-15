# Fairino FR5 Painting Rig

A hand-held painting handle (Teensy 4.1 + BNO055 IMU + 3× draw-wire encoders) teleoperates a **Fairino FR5** cobot in real time. The operator paints by hand; the arm mirrors the motion with a paint-valve solenoid driven from a handle-mounted switch. Trajectories can be recorded during a session and replayed autonomously on subsequent parts.

Target hardware: **Fairino FR5** cobot (controller FRC100-AC, firmware V3.8.7-QX) + **Teensy 4.1** handle + painting solenoid wired to FR5 DO0.

```
   ┌──────────────────────┐    USB     ┌───────────────┐  Ethernet   ┌─────────────┐
   │ Teensy 4.1 handle    │───────────▶│ Host bridge   │────────────▶│ FR5 cobot   │
   │ BNO055 + 3× encoders │   125 Hz   │ painting_     │   ~100 Hz   │ 192.168.57.2│
   │ pin-6 switch         │  Teleplot  │ bridge/       │  ServoJ+IK  │ DO0 → valve │
   └──────────────────────┘            └───────────────┘             └─────────────┘
```

## Repo layout

```
hikvision_integration/
├── fairino/                    Fairino Python SDK (I/O fixes for V3.8.7-QX)
├── painting_firmware/          Teensy 4.1 handle firmware (PlatformIO)
│   ├── platformio.ini
│   └── src/main.cpp            IMU fusion, trilateration, button+rezero markers
├── painting_bridge/            Real-time streaming bridge (live teleop + recorder)
│   ├── bridge.py               Main entry point
│   ├── teensy_reader.py        Serial parser
│   ├── fr5_servo.py            FR5 SDK wrapper + MockRobot for --dry-run
│   ├── safety.py               EMA, delta clamp, workspace box, reach gate
│   ├── recording.py            TrajectoryRecorder thread (recorder.py-compatible)
│   ├── config.yaml             Tunables (scales, limits, filter alpha, recording)
│   └── tests/                  Unit tests (22/22 pass)
├── recorder.py                 GUI recorder + player (drag-teach workflow)
├── io_monitor.py               Real-time digital-I/O monitor GUI
├── lua_manager.py              Upload/run Lua scripts on the controller
├── di_do_passthrough.lua       Robot-side DI→DO mirror (10 ms loop)
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

### Clone + venv

```bash
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration

# Linux
python3.12 -m venv .venv
source .venv/bin/activate

# Windows
py -3.12 -m venv .venv
.venv\Scripts\activate
```

### Python dependencies

```bash
pip install -r requirements.txt
pip install pyserial pyyaml   # for painting_bridge
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

Press the handle's pin-6 switch → `SetDO(0, 1)`. Release → `SetDO(0, 0)`. Edge-triggered, not per-cycle. Valve closes automatically on `Ctrl-C`.

### Re-anchoring mid-session

Press `z` on the Teensy serial console → firmware emits `>rezero:1` → bridge re-captures `tcp_start = current TCP` and `handle_ref = current handle pose` atomically. The arm doesn't move; handle→arm mapping is recentered.

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

| Method | Timing | Upload required? | Use case |
|---|---|---|---|
| `recorder.py` → **Play (Servo+DO)** | Host-paced ~100 Hz | no | Quick dev retest |
| `recorder.py` → **Play (TPD)** | Controller-native, exact | yes — `TrajectoryJUpLoad` | Production painting runs |

For TPD exact-timing playback:
```python
from fairino import Robot
r = Robot.RPC('192.168.57.2')
r.TrajectoryJUpLoad(filePath='./recordings/run_20260415_143000.txt')
# Then use recorder.py GUI → Load → Play (TPD)
```

See **[painting_bridge/README.md](painting_bridge/README.md)** for in-depth bridge docs and the [full file-format spec](painting_bridge/README.md#using-it).

---

## Configuration reference (`painting_bridge/config.yaml`)

| Key | Default | Notes |
|---|---|---|
| `serial.port` | `auto` | Or explicit `/dev/ttyACM0` |
| `robot.ip` | `192.168.57.2` | Override for different controllers |
| `robot.use_servoj` | `true` | Use ServoJ+IK (proven); `false` reverts to ServoCart (hit err=112) |
| `robot.cmd_period_s` | `0.010` | Declared cmdT; match loop_period_s |
| `robot.loop_period_s` | `0.010` | Host servo loop target (100 Hz) |
| `mapping.scale_xyz` | `1.0` | 1:1 handle mm → TCP mm |
| `mapping.scale_rot` | `1.0` | Set to `0` for translation-only teleop (avoids wrist singularities) |
| `safety.max_delta_mm_per_cycle` | `2.5` | 250 mm/s ceiling at 100 Hz |
| `safety.workspace_box_mm` | `[250,250,250]` | Axis-aligned cube around `tcp_start` |
| `safety.workspace_reach_radius_mm` | `300` | Spherical reach gate |
| `filter.ema_alpha_xyz` | `0.35` | Lower = more smoothing / more lag |
| `solenoid.enabled` | `true` | Disable to ignore button entirely |
| `solenoid.do_id` | `0` | FR5 DO channel |
| `recording.enabled` | `true` | Disable `r` hotkey |
| `recording.period_s` | `0.010` | 100 Hz; use `0.008` to match `recorder.py` CYCLETIME |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `No Teensy found` | Device not enumerated | `ls /dev/serial/by-id/`; plug in or pass `--serial` |
| `SDK连接机器人实时端口失败 [Errno 113] No route to host` | Ethernet not on 192.168.57.0/24 | Verify `ip route get 192.168.57.2`; bring up the `fr5` NM connection |
| `err=-4` on every RPC | Network timeout (bad route / firewall / robot off) | `ping 192.168.57.2` must succeed before anything else |
| `ServoCart err=112` / `ServoJ err=112` | IK solver can't find a solution | Non-fatal; bridge holds. Re-home arm to joint-mid-range pose; set `scale_rot: 0` for first runs |
| `ik_fail` counter climbing | Target outside reachable envelope or past joint limit | Pull handle back toward anchor pose; re-anchor with `z` |
| Clamps firing constantly (`clamped` high) | Handle noisy or scale too high | Lower `scale_xyz`; raise `ema_alpha_xyz` toward 0.15 |
| Arm drifts at rest | BNO055 yaw drift via `scale_rot > 0` | Set `scale_rot: 0` for translation-only teleop |
| `trilatFail` increments | Draw-wires slack or anchors moved | Re-tension; re-verify `ANCHOR*` constants in `main.cpp` |
| Sudden arm lunge after IK recovery | Operator moved handle during IK-hold, arm catches up at max velocity | See "Movement smoother after IK recovery" recommendation (re-anchor-on-resume pattern) |
| Motion seems ~25 % slower than recording | Recorded at rate > playback ceiling | Keep `recording.period_s` at `0.010` for `recorder.py`'s `Play (Servo+DO)`; or use TPD for exact timing |

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
