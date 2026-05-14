# Cold-start handoff — Fairino FR5 painting rig

Two-machine operational guide. **Record on Linux. Play back on Windows.** No exceptions until the playback-from-Linux issue is root-caused.

For the full system manual see `README.md`. For bridge internals see `painting_bridge/README.md`. This document is the workflow narrative that ties the two together.

---

## TL;DR — the topology

```
  Teensy 4.1 handle           Linux host (record)             FR5 cobot
  ─────────────────           ──────────────────              ─────────
  BNO055 IMU             USB    painting_bridge/    Ethernet
  3× draw-wire    ───────────▶  bridge.py           ────────▶  192.168.57.2
  pin-6 switch    125 Hz        100 Hz ServoJ + IK            DO0 → paint valve
                                'r' Enter → record
                                  │
                                  ▼
                       recordings/run_YYYYMMDD_HHMMSS.txt
                                  │
                                  │ scp / USB stick
                                  ▼
                            Windows host (play back)
                            ────────────────────────              FR5 cobot
                            .venv\hikvision.py        Ethernet
                            Tkinter GUI               ────────▶  192.168.57.2
                            Load → Start (TPD)                  DO0 from trigger col
```

**The hard rule:** `hikvision.py` is run **only from Windows**. Running it from Linux has produced unsafe FR5 motion. The bridge itself (recording side) is fine on Linux — it is what was built for Linux. Only the playback GUI is Windows-restricted.

---

## What lives where

| Thing | Path | Notes |
|---|---|---|
| Bridge entry (Linux) | `painting_bridge/bridge.py` | argparse: `--config`, `--serial`, `--dry-run`, `--log-level` |
| Bridge config | `painting_bridge/config.yaml` | Scales, safety limits, robot IP, recording period |
| Recordings output | `painting_bridge/recordings/run_YYYYMMDD_HHMMSS.txt` | Auto-created; gitignored |
| Recording format spec | `painting_bridge/recording.py:33–47` | Header `N,1,period_ms,tool,diconfig,doconfig`; rows `j1..j6,x,y,z,rx,ry,rz,5,trigger,17` |
| Linux venv | `.venv/bin/python` | Python 3.12.13; `fairino`, `pyserial==3.5`, `pyyaml==6.0.3`, `numpy`, `matplotlib` |
| Playback GUI (Windows) | `.venv/hikvision.py` | Tkinter; **not git-tracked** — copy it onto the Windows venv |
| Playback constants | `.venv/hikvision.py:13–21` | `IP='192.168.57.2'`, `CYCLETIME=0.008`, `FRTYPE=5` |
| Start button → | `.venv/hikvision.py:44` | `run_tpd` (controller-paced; the safe one) |
| FR5 IP | hardcoded `192.168.57.2` | in `config.yaml`, `recorder.py`, `io_monitor.py`, `lua_manager.py`, `hikvision.py` |
| Fairino SDK | `fairino/Robot.py` | Patched for fw V3.8.7-QX I/O reads — keep as-is, do not commit upstream Robot.c |
| Firmware | `painting_firmware/src/main.cpp` | PlatformIO. `pio run -t upload` |

---

## One-time cold-start setup

### Linux host (records trajectories; runs the live teleop)

```bash
# 1. clone + venv
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration
python3.12 -m venv .venv
source .venv/bin/activate

# 2. python deps
pip install -r requirements.txt
pip install pyserial pyyaml

# 3. Fairino SDK — Option A (editable) or B (direct import from ./fairino, which
#    is what bridge.py does by default). Option B is fine; Option A only if you
#    want `from fairino import Robot` to work from anywhere.
pip install cython
pip install -e ./fairino   # optional

# 4. Verify SDK reaches the controller
python -c "from fairino import Robot; r = Robot.RPC('192.168.57.2'); print(r.GetSDKVersion())"
```

**Network — persistent NM connection (one-time):**

```bash
sudo nmcli con add type ethernet ifname enp7s0 con-name fr5 \
    ipv4.method manual ipv4.addresses 192.168.57.10/24 \
    ipv4.never-default yes ipv6.method disabled
sudo nmcli con up fr5

ping -c 3 192.168.57.2        # ~0.2 ms RTT
ip route get 192.168.57.2     # must go via enp7s0, NOT wlan0
```

Substitute your wired interface for `enp7s0` (`ip link` to find it). `ipv4.never-default yes` is non-negotiable — without it the wired profile steals the default route and breaks your normal internet.

**Firmware (one-time per Teensy):**

```bash
cd painting_firmware
pio run -t upload
pio device monitor        # confirm Teleplot stream: >x: >y: >z: >qw: ... >button:
```

Confirm the handle enumerates as `/dev/serial/by-id/usb-Teensyduino_USB_Serial_*`.

### Windows host (plays back trajectories only)

```powershell
# 1. clone + venv
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration
py -3.12 -m venv .venv
.venv\Scripts\activate

# 2. python deps
pip install -r requirements.txt
pip install pyserial pyyaml

# 3. Fairino SDK — Option A is simpler on Windows because hikvision.py does
#    `from fairino import Robot` unconditionally.
pip install cython
pip install -e .\fairino

# 4. Verify
python -c "from fairino import Robot; r = Robot.RPC('192.168.57.2'); print(r.GetSDKVersion())"
```

**Place `hikvision.py`.** The file is NOT in git (`.venv/` is gitignored). Get it from the Linux host's `.venv/hikvision.py` or from your backup, and put it somewhere convenient on the Windows machine — the venv root mirrors the Linux layout, but anywhere on PATH or in your working directory is fine.

**Network — Windows ethernet adapter:**

- Static IPv4: `192.168.57.11`, mask `255.255.255.0`, **leave default gateway blank**.
- Disable IPv6 on that adapter.
- `ping 192.168.57.2` must succeed.

---

## Per-session recording workflow (Linux)

**Pre-flight, every session:**

1. E-stop within reach. Test it.
2. Move the arm to a joint-mid-range pose via the pendant: J1≈0°, J2≈−60°, J3≈90°, J4≈0°, J5≈90°, J6≈0°. Stay 10°+ off every joint limit — this alone eliminates most IK failures.
3. Hold the handle **stationary**. The bridge's first command is the current TCP pose (zero delta), so the arm must not move on `ServoMoveStart`. If it does, E-stop and investigate before doing anything else.

**Run:**

```bash
source .venv/bin/activate
cd painting_bridge
python bridge.py
```

Flags (from `bridge.py:392–408`):

| Flag | Purpose |
|---|---|
| `--config <path>` | Override default `painting_bridge/config.yaml` |
| `--serial <dev>` | Override auto-detect, e.g. `/dev/ttyACM0` |
| `--dry-run` | Use `MockRobot` — no controller needed, useful for sanity-checking the stream |
| `--log-level DEBUG\|INFO\|WARNING\|ERROR` | Default `INFO` |

**Expected startup log:**

```
using serial port: /dev/serial/by-id/usb-Teensyduino_USB_Serial_...
serial opened: ... @ 115200
connected to FR5 at 192.168.57.2
preflight OK: tool_id=0 tcp=[...] joints=[...]
anchored: tcp_start=[...] handle_ref=[...]
sent=99 skipped=0 clamped=0 ik_fail=0 hold=0 retry_ok=0 retry_fail=0 target=[...]
```

**1 Hz stats line — what to watch:**

| Field | What it means | When to worry |
|---|---|---|
| `sent` | `ServoJ` calls issued | Should stay near 100/s |
| `skipped` | Cycles missed (stream-loss watchdog) | Any sustained non-zero → check serial/USB |
| `clamped` | Workspace box or per-cycle delta clamp engaged | Constant clamps → lower `scale_xyz` or raise `ema_alpha_xyz` |
| `ik_fail` | IK solver returned no solution | Non-fatal (bridge holds); persistent → re-home to mid-range, set `scale_rot: 0` |
| `hold` | Cycles spent holding (recovery after fail) | High → operator moved during recovery; re-anchor |
| `retry_ok` / `retry_fail` | Fresh-joint IK retry outcome | `retry_fail` climbing → target genuinely out of reach |

**To record a trajectory:**

```
press   r   then Enter      → recorder starts
... move the handle / paint ...
press   r   then Enter      → recorder stops
```

The file lands at `painting_bridge/recordings/run_YYYYMMDD_HHMMSS.txt`. Format spec lives in `painting_bridge/recording.py:33–47`; header looks like `0000003318,1,10,0,1,1`, data rows have 15 comma-separated fields and end in `,5,<0-or-1>,17`.

**Mid-session re-anchor:** press `z` on the Teensy serial console (`pio device monitor` or any serial terminal). Firmware emits `>rezero:1`, bridge atomically re-captures `tcp_start` and `handle_ref`. The arm doesn't move; the handle→arm mapping is recentered.

**Shutdown:** `Ctrl-C`. Bridge runs `StopMotion → ServoMoveEnd → SetDO(0,0)` and exits cleanly.

---

## Transferring the file to the Windows host

```bash
scp painting_bridge/recordings/run_*.txt  user@windows-host:'C:/Users/<you>/trajectories/'
```

A USB stick or shared drive works equally well — the file is plain ASCII, line endings don't matter.

**Sanity check on Windows** before clicking anything in the GUI: open the `.txt` in a text editor and confirm

- the first line looks like `0000003318,1,10,0,1,1` (a 10-digit count, then five small integers);
- data lines have **15** comma-separated fields, end in `,5,<0-or-1>,17`.

---

## Per-session playback workflow (Windows)

```powershell
.venv\Scripts\activate
python .venv\hikvision.py
```

(Or wherever you put `hikvision.py`.)

In the GUI:

1. **Load** → pick the `.txt` you copied over. The script calls `TrajectoryJUpLoad(filePath=...)` automatically at this step (see `hikvision.py:168`), so the file is pushed to the controller's `traj/` namespace immediately on load.
2. The 3D Matplotlib plot renders the path. **Visually confirm** it matches what you recorded — wrong file, wrong frame, anything weird — bail out here.
3. **Start** → calls `run_tpd()` (`hikvision.py:44`, body at `:273`). The sequence is:
   - `LoadTPD("traj/<name>")`
   - `GetTPDStartPose` → `GetInverseKin` → `MoveJ` to the start pose
   - `MoveTPD("traj/<name>", 1, 100)` — controller-paced playback at the period encoded in the file header.

Timing is owned by the FR5 firmware once `MoveTPD` is called. Host OS jitter no longer matters during the trajectory.

**Do not press "Start Tracking"** — that button enters drag-teach (it switches the arm into freedrive). It's used by the recorder.py workflow, not by playback. Likewise, do not edit the Start button to call `run_servo_cartesian_path` — that path is the host-paced ServoJ loop with `time.sleep(0.001)` between commands (`hikvision.py:205–271`), and it is exactly what produces violent motion when the host can't hold the 8 ms cadence.

---

## Why playback from Linux is forbidden

Empirical observation: when `hikvision.py` is run from a Linux host against this FR5, the arm has moved violently. This was reproduced, it was not investigated to root cause, and the operational fix has been "only run it from Windows."

What is plausible (but not verified — **the rule stands regardless**):

- The Fairino SDK uses XML-RPC over TCP plus a blocking state-stream socket on port 20004 (see `fairino/Robot.py`). Both are blocking I/O; both interact with the Python GIL and the Tk event loop.
- The host-paced `ServoJ` path inside `run_servo_cartesian_path` sleeps for 1 ms between commands and targets an 8 ms `cmdT`. On Linux without `PREEMPT_RT`, scheduler jitter routinely pushes that loop past the controller's tolerance, and the `MoveJ` to the start pose plus the upload sequence has its own latency profile.
- The Windows Tkinter + Winsock stack appears to interleave with these blocking sockets in a way that holds the cadence; Linux, in this codebase, does not.

TPD playback in theory runs on the controller and should be OS-agnostic, but `LoadTPD` and the `MoveJ`-to-start-pose call still happen from the host, and Linux is where the bad motion has been seen. **Do not test this on hardware.** If we ever want Linux playback, do it against a simulated controller first.

---

## Safety + troubleshooting

**First-run conservative config** (in `painting_bridge/config.yaml`):

```yaml
mapping:
  scale_xyz: 0.1          # 10:1 reduction — handle 10 mm → arm 1 mm
  scale_rot: 0            # translation only; no orientation while bringing up
safety:
  workspace_box_mm: [50, 50, 50]
  max_delta_mm_per_cycle: 2.0
```

Raise these only after you've confirmed the handle→arm direction is correct and clamps aren't constantly firing.

**Troubleshooting cross-references:**

- General — `README.md` → "Troubleshooting" table.
- Bridge-specific — `painting_bridge/README.md` → "Troubleshooting" table.

**Playback-specific row to add to your mental troubleshooting table:**

| Symptom | Cause | Fix |
|---|---|---|
| Arm moves violently during playback | You are running `hikvision.py` from Linux | E-stop. Move playback to the Windows host. |
| Arm slow/wrong-speed on TPD playback | Recorded period (header `period_ms`) doesn't match expectation | Re-record with `recording.period_s: 0.010` in `config.yaml`, or set `recording.period_s: 0.008` to match `hikvision.py`'s `CYCLETIME` |
| `hikvision.py` errors on Load with `Trajectory file ... uploaded` followed by RPC error | Filename has spaces or path separators the controller rejects | Rename to `run_YYYYMMDD_HHMMSS.txt` style (no spaces, no subdirs) |
| `err=-4` from RPC on Windows | Routing / firewall — controller unreachable | `ping 192.168.57.2`; disable Windows Firewall on the wired profile or whitelist the Fairino ports |

---

## Quick reference card

**Record (Linux):**

```bash
source .venv/bin/activate
cd painting_bridge
python bridge.py
# 'r' Enter to start recording, 'r' Enter to stop, Ctrl-C to exit
```

**Transfer:**

```bash
scp painting_bridge/recordings/run_*.txt user@windows-host:'C:/Users/<you>/trajectories/'
```

**Play back (Windows):**

```powershell
.venv\Scripts\activate
python .venv\hikvision.py
# GUI: Load → pick .txt → Start
```

---

## Files an operator should know exist

- `README.md` — system manual, hardware pinout, install
- `painting_bridge/README.md` — bridge internals, safety invariants, troubleshooting
- `painting_bridge/bridge.py` — bridge entry (argparse at `:392`)
- `painting_bridge/config.yaml` — all tunables
- `painting_bridge/recording.py` — trajectory file writer
- `.venv/hikvision.py` — Windows playback GUI
- `recorder.py` — separate GUI for drag-teach record/playback (alternative to the bridge for non-handle workflows)
- `io_monitor.py` — live DI/DO monitor; useful for verifying the paint valve before a run
- `lua_manager.py` + `di_do_passthrough.lua` — controller-side DI→DO passthrough for host-less bench tests
- `fairino/Robot.py` — patched SDK (do not commit Robot.c upstream)
