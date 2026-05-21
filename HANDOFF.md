# Cold-start handoff — Fairino FR5 painting rig

Two-machine operational guide. **Record on Linux. Play back on Windows.** No exceptions until the playback-from-Linux issue is root-caused.

For the full system manual see `README.md`. For bridge internals see `painting_bridge/README.md`. This document is the workflow narrative that ties the two together.

Current bridge input sources:

- **Teensy handle**: the original USB serial painting handle.
- **Quest controller**: OpenVR controller pose source. Quest trigger maps to the paint button. Holding `B` or `Y` pauses motion; releasing it re-anchors control from that release pose.
- **Camera (AprilTag)**: USB camera tracking a handheld moving AprilTag relative to a fixed reference tag (camera-position-invariant). No built-in trigger. On Windows, global Space is hold-to-spray without terminal focus; on Linux, use focused-terminal Space or a USB HID keypad. Cold-start for the rig is in [`camera_tracking/README.md`](camera_tracking/README.md).

The bridge asks for the source at runtime unless `--source teensy|quest|camera` is supplied.

---

## TL;DR — the topology

```
  Inputs (pick one)             Linux host (record)             FR5 cobot
  ─────────────────             ──────────────────              ─────────
  Teensy 4.1 handle      USB
  BNO055 + draw-wires  ──────┐
  pin-6 switch (paint) 125Hz │
                             │   painting_bridge/    Ethernet
  Quest controller    OpenVR │   bridge.py           ────────▶  192.168.57.2
  pose + trigger + B/Y ──────┼─▶ 100 Hz ServoJ + IK             DO0 → paint valve
                             │   'r' Enter → record
  USB camera +         USB   │     │
  AprilTags (ref+move)──────┘      │
                                   ▼
  USB keypad (Linux,   evdev    paint trigger OR
  QINIZX 8808:6601) ─────────▶  (handle / spacebar / keypad)
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
| Bridge entry | `painting_bridge/bridge.py` | argparse: `--config`, `--serial`, `--source teensy\|quest\|camera`, `--list-keypads`, `--dry-run`, `--log-level`; prompts for source if omitted |
| Bridge config | `painting_bridge/config.yaml` | Serial, Quest, camera, keypad, scales, safety limits, robot IP, recording period |
| Quest source | `painting_bridge/quest_reader.py` | OpenVR input source; publishes controller deltas, not raw Quest coordinates |
| Quest viewport/debugger | `quest.py` | Standalone OpenVR coordinate viewport; same `+X right`, `+Y away`, `+Z up` convention |
| Camera source | `painting_bridge/camera_reader.py` | AprilTag reader; publishes moving-tag-relative-to-reference-tag deltas as `HandleSample`, smoothed via One Euro |
| Camera rig | `camera_tracking/` | `calibrate.py` (checkerboard intrinsics), `apriltag_pose.py` (standalone setup viewer), `tag_detector.py` (detection + solvePnP), `camera.py` (capture wrapper), `config.yaml` (tag ids/sizes, device, calibration path) |
| Keypad source | `painting_bridge/keypad_reader.py` | Linux/evdev USB HID paint trigger; opens all matching nodes, filters by EV_KEY capability, grabs exclusively, multiplexes with `select` (per-fd unplug is handled gracefully) |
| Windows Space source | `painting_bridge/windows_hotkey_reader.py` | Windows/pynput global Space listener; hold-to-spray even when the terminal is not focused |
| One Euro filter | `painting_bridge/one_euro.py` | Adaptive low-pass used by `camera_reader.py` for jitter rejection without lag during motion |
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
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install pygame openvr  # optional: Quest source / quest.py viewport on this host
# requirements.txt includes pyserial, opencv-python, numpy, pyyaml, and Linux evdev

# 3. Fairino SDK — Option A (editable) or B (direct import from ./fairino, which
#    is what bridge.py does by default). Option B is fine; Option A only if you
#    want `from fairino import Robot` to work from anywhere.
pip install cython
pip install -e ./fairino   # optional

# 4. Verify SDK reaches the controller
python -c "from fairino import Robot; r = Robot.RPC('192.168.57.2'); print(r.GetSDKVersion())"
```

**Input-group permission (USB keypad) — Linux only:**

Reading `/dev/input/event*` needs membership in the `input` group. The QINIZX 2-key macropad (or any USB HID keypad you point `keypad.match_name` / VID:PID at) won't be visible to the bridge until this is done:

```bash
sudo usermod -aG input $USER
# log out and log back in — group membership only applies to new sessions
id | grep input                           # confirm membership
python painting_bridge/bridge.py --list-keypads
```

`--list-keypads` prints every visible evdev device (path, name, VID:PID, KEY_SPACE capability) and exits. The QINIZX default in `config.yaml` is VID:PID `0x8808:0x6601`; if your keypad differs, set `keypad.match_name` or override `vendor_id` / `product_id`. The reader is a silent no-op when `evdev` is missing or no matching device is found — Teensy and Quest sources work without the keypad.

**Camera rig (optional — only if you'll use the camera source):**

Print the AprilTags, measure each tag's outer black edge, set ids/sizes in `camera_tracking/config.yaml`, then run a one-time checkerboard calibration:

```bash
.venv/bin/python camera_tracking/calibrate.py     # SPACE = capture, C = calibrate; aim RMS < 0.5 px
.venv/bin/python camera_tracking/apriltag_pose.py # verify both tags detect and the frame is sane
```

Lay the **reference** tag flat with its `+X/+Y` aligned to the FR5 base. Full step-by-step (checkerboard cell size, expected RMS, tag placement gotchas) lives in [`camera_tracking/README.md`](camera_tracking/README.md) — don't restate it here.

Camera device notes:

- Linux `camera.device: auto` matches the Arducam OV9281 through `v4l2-ctl`.
- Windows `camera.device: auto` lists DirectShow camera names through
  `pygrabber` and prefers external-looking devices such as Arducam, OV9281,
  USB, Webcam, Logitech, or C920 over built-in cameras. The startup log prints
  the chosen index; set `camera.device` to that index only if you need to force
  a device.
- Keep `camera.fourcc: MJPG` for fast USB capture. On Windows, tune
  `windows_exposure` on the real Arducam if tags blur or the image is too dark.
  Placeholder webcam calibration is not valid for the Arducam.

If `python3.12` is not installed, install Python 3.12 with your distro/package manager first. If you use `uv` on Linux instead, the equivalent self-contained setup is:

```bash
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration
export UV_CACHE_DIR=.uv-cache
export UV_PYTHON_INSTALL_DIR=.uv-python
uv venv --managed-python --python 3.12 .venv
source .venv/bin/activate
uv pip install --python .venv/bin/python -r requirements.txt pygame openvr
python -m ensurepip --upgrade
python -c "import cv2, numpy, matplotlib, serial, yaml; print('core imports ok')"
python -c "import pygame, openvr; print('quest imports ok')"  # only if Quest deps were installed
python -m py_compile quest.py painting_bridge/bridge.py
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

### Windows host (plays back trajectories; optional camera setup/dry-runs)

```powershell
# 1. clone + venv
git clone https://github.com/Nitish05/hikvision-integration.git
cd hikvision-integration
py -3.12 -m venv .venv
.venv\Scripts\activate

# 2. python deps
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install pygame openvr # optional: Quest source / quest.py viewport on this host
# requirements.txt includes Windows pynput (global Space) and pygrabber (DirectShow camera names)

# 3. Fairino SDK — Option A is simpler on Windows because hikvision.py does
#    `from fairino import Robot` unconditionally.
pip install cython
pip install -e .\fairino

# 4. Verify
python -c "from fairino import Robot; r = Robot.RPC('192.168.57.2'); print(r.GetSDKVersion())"
```

If this is a fresh Windows machine and PowerShell says `py`, `python`, and `python3` are not recognized, use `uv` to create the venv with a managed Python 3.12:

```powershell
# From the repo root
$env:UV_CACHE_DIR=".uv-cache"
$env:UV_PYTHON_INSTALL_DIR=".uv-python"

uv venv --managed-python --python 3.12 .venv
.\.venv\Scripts\python.exe -m ensurepip --upgrade
.\.venv\Scripts\activate

uv pip install --python .venv\Scripts\python.exe -r requirements.txt pygame openvr
python -c "import cv2, numpy, matplotlib, serial, yaml, pynput, pygrabber; print('core imports ok')"
python -c "import pygame, openvr; print('quest imports ok')"  # only if Quest deps were installed
python -m py_compile quest.py painting_bridge\bridge.py
```

If uv reports cache or Python-install permission errors, keep these two environment variables set exactly as above. They force uv to use `.uv-cache/` and `.uv-python/` inside the repo; both are ignored by git.

**Place `hikvision.py`.** The file is NOT in git (`.venv/` is gitignored). Get it from the Linux host's `.venv/hikvision.py` or from your backup, and put it somewhere convenient on the Windows machine — the venv root mirrors the Linux layout, but anywhere on PATH or in your working directory is fine.

**Network — Windows ethernet adapter:**

- Static IPv4: `192.168.57.11`, mask `255.255.255.0`, **leave default gateway blank**.
- Disable IPv6 on that adapter.
- `ping 192.168.57.2` must succeed.

PowerShell setup (run as Administrator). First list adapters and identify the
Ethernet port plugged into the FR5:

```powershell
Get-NetAdapter | Format-Table Name, InterfaceDescription, Status, LinkSpeed
```

Then replace `Ethernet` below with the adapter `Name` from that table:

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

# Remove old IPv4 addresses/routes on only this adapter.
Get-NetIPAddress -InterfaceAlias $if -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Remove-NetIPAddress -Confirm:$false
Get-NetRoute -InterfaceAlias $if -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Remove-NetRoute -Confirm:$false

# Robot subnet. No DefaultGateway on purpose, so WiFi keeps internet access.
New-NetIPAddress -InterfaceAlias $if -IPAddress 192.168.57.11 -PrefixLength 24
Set-NetIPInterface -InterfaceAlias $if -Dhcp Disabled
Disable-NetAdapterBinding -Name $if -ComponentID ms_tcpip6

ping 192.168.57.2
Test-NetConnection 192.168.57.2 -Port 20003
```

If internet stops working after this, the adapter probably has a gateway route.
Check with:

```powershell
Get-NetRoute -InterfaceAlias $if -AddressFamily IPv4
```

There should be a route for `192.168.57.0/24`, but no `0.0.0.0/0` default route
on the FR5 Ethernet adapter.

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

If `--source` is omitted, the bridge asks:

```text
Select control source:
  1. Teensy handle
  2. Quest controller
  3. Camera (AprilTag)
Press Enter for Teensy.
>
```

Use `python bridge.py --source teensy` for the original handle, `--source quest` for the Quest controller, or `--source camera` for the AprilTag rig. `--dry-run` can be combined with any source. `python bridge.py --list-keypads` is a Linux-only diagnostic that prints the visible evdev devices and exits.

Flags (`bridge.py` argparse near `:606`):

| Flag | Purpose |
|---|---|
| `--config <path>` | Override default `painting_bridge/config.yaml` |
| `--serial <dev>` | Override Teensy auto-detect, e.g. `COM5` or `/dev/ttyACM0` |
| `--source teensy\|quest\|camera` | Bypass the runtime source menu |
| `--list-keypads` | Linux-only: print every visible evdev device (path, name, VID:PID, KEY_SPACE) and exit. Use to set `keypad.match_name` or VID:PID when the QINIZX `0x8808:0x6601` default doesn't match |
| `--dry-run` | Use `MockRobot` — no controller needed, useful for sanity-checking the selected source |
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

For Quest source, expected startup logs include `using quest controller source`, `quest source opened through OpenVR`, and `quest headset coordinate frame locked`. If SteamVR is not ready, the reader logs whether it is waiting for the headset pose or controller pose.

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
press   r   (no Enter)      → recorder starts
... move the handle / paint ...
press   r   (no Enter)      → recorder stops
```

On Linux/macOS, the bridge sets the terminal to cbreak at startup, so `r` and SPACE register as single keypresses (no Enter). Restored on `Ctrl-C` / exit. On a non-TTY (piped, `nohup`, systemd), cbreak is skipped cleanly and the stdin hotkeys are unavailable — use the USB keypad instead. On Windows, global Space is available for paint, but the recorder `r` hotkey is not global; keep production recording on the Linux host unless that path is extended.

**Solenoid trigger sources** (OR'd — any one ON drives `DO0` ON):

- Teensy pin-6 switch (handle source only).
- Linux/macOS stdin SPACE toggle — requires the bridge terminal to be focused.
- Windows global SPACE — hold-to-spray through `pynput`, works even when the terminal is not focused. Events are not suppressed, so the focused app still receives normal Space input.
- USB HID keypad (Linux, `evdev`) — `grab()`'d exclusively, so it fires the solenoid without needing the bridge terminal to be focused. Works alongside the other two; OR'd in. Critical for the camera source, which has no handle button.

If the keypad isn't responding, run `python bridge.py --list-keypads` to confirm it's visible and that you're in the `input` group.

The file lands at `painting_bridge/recordings/run_YYYYMMDD_HHMMSS.txt`. Format spec lives in `painting_bridge/recording.py:33–47`; header looks like `0000003318,1,10,0,1,1`, data rows have 15 comma-separated fields and end in `,5,<0-or-1>,17`.

**Mid-session re-anchor:** press `z` on the Teensy serial console (`pio device monitor` or any serial terminal). Firmware emits `>rezero:1`, bridge atomically re-captures `tcp_start` and `handle_ref`. The arm doesn't move; the handle→arm mapping is recentered.

**Quest mid-session re-anchor:** hold `B` or `Y` on either controller. While held, the bridge holds the robot and ignores controller movement. Move to the new neutral pose, then release `B`/`Y`; the Quest controller anchor and robot TCP anchor are refreshed together, so control resumes from the release pose without a catch-up move.

**Camera mid-session re-anchor:** cover the moving tag with your hand for longer than `safety.stream_timeout_ms`. The bridge holds the robot. When the tag reappears both the moving-tag anchor and the robot TCP anchor refresh atomically — no catch-up move. "Cover, reposition, uncover" is the rezero.

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

**First-run conservative config** (in `painting_bridge/config.yaml`).

The repo default is now `mapping.scale_xyz: 1.2` (bumped after camera-source tuning); the values below are a **deliberately conservative first-run override** — much lower than the default, with rotation disabled and a tight workspace box. Use them when bringing up a new source, a new operator, or a new physical layout. Walk them back up to the default only after you've confirmed direction and clamps:

```yaml
mapping:
  scale_xyz: 0.1          # 10:1 reduction — handle 10 mm → arm 1 mm
  scale_rot: 0            # translation only; no orientation while bringing up
safety:
  workspace_box_mm: [50, 50, 50]
  max_delta_mm_per_cycle: 2.0
```

Raise these only after you've confirmed the handle/Quest/tag → arm direction is correct and clamps aren't constantly firing.

**Troubleshooting cross-references:**

- General — `README.md` → "Troubleshooting" table.
- Bridge-specific — `painting_bridge/README.md` → "Troubleshooting" table.

**Playback-specific row to add to your mental troubleshooting table:**

| Symptom | Cause | Fix |
|---|---|---|
| Arm moves violently during playback | You are running `hikvision.py` from Linux | E-stop. Move playback to the Windows host. |
| Quest source never anchors | SteamVR/OpenVR is not publishing headset/controller poses | Open SteamVR, wake headset and controller, verify `quest.py` viewport works. |
| Quest source moves from an unexpected neutral pose | Controller anchor was captured in the wrong place | Hold `B`/`Y`, move to the desired neutral pose, release to re-anchor. |
| `clamped` climbs constantly | Input delta is too large or too fast for safety limits | Lower `mapping.scale_xyz` / `mapping.scale_rot`, move slower, or re-anchor. |
| Arm slow/wrong-speed on TPD playback | Recorded period (header `period_ms`) doesn't match expectation | Re-record with `recording.period_s: 0.010` in `config.yaml`, or set `recording.period_s: 0.008` to match `hikvision.py`'s `CYCLETIME` |
| `hikvision.py` errors on Load with `Trajectory file ... uploaded` followed by RPC error | Filename has spaces or path separators the controller rejects | Rename to `run_YYYYMMDD_HHMMSS.txt` style (no spaces, no subdirs) |
| `err=-4` from RPC on Windows | Routing / firewall — controller unreachable | `ping 192.168.57.2`; disable Windows Firewall on the wired profile or whitelist the Fairino ports |
| Camera source never anchors / `no AprilTag detected` | Lighting, occlusion, wrong tag IDs/sizes, or missing calibration | Verify with `camera_tracking/apriltag_pose.py` standalone; re-check tag IDs/sizes in `camera_tracking/config.yaml`; recalibrate if the lens/focus changed |
| `calibration.npz not found` on camera bridge startup | Intrinsic calibration has not been run for that camera/resolution | Run `camera_tracking/calibrate.py` first; placeholder webcam calibration is not valid for the Arducam |
| Windows picks the built-in camera | DirectShow order/name did not identify the external camera, or config forces the wrong index | Keep `camera.device: auto` and read the `DirectShow devices:` startup log; set `camera.device` to the printed Arducam/USB index only if needed |
| Arducam tags blur on Windows | Exposure too long or lighting too low | Tune `camera_tracking/config.yaml` `windows_exposure` (`-6` start, more negative for shorter exposure) and add light |
| Camera-source axis mirrored | `camera.invert_*` or `ref_to_base_yaw_deg` doesn't match the physical reference-tag placement | Twist the moving tag along each base axis individually; flip the matching `invert_*`, or in `control_frame: base` tune `ref_to_base_yaw_deg` |
| Keypad does nothing | Disabled in config; user not in `input` group; wrong VID:PID; `evdev` missing | `pip install evdev`; `sudo usermod -aG input $USER` (log out/in); `python bridge.py --list-keypads`; set `keypad.match_name` or override VID:PID |
| Spacebar hotkey does nothing on Linux/macOS | Terminal not focused; stdin not a TTY (`nohup`, systemd, piped, some IDE shells) | Focus the terminal or use the Linux USB keypad |
| Spacebar hotkey does nothing on Windows | `pynput` missing or blocked | `pip install -r requirements.txt`; check for `Windows global SPACE hotkey enabled` in the bridge log |

---

## Quick reference card

**Record (Linux):**

```bash
source .venv/bin/activate
cd painting_bridge
python bridge.py
# choose source from menu, or pass --source teensy / --source quest / --source camera
# SPACE toggles solenoid on Linux/macOS when terminal-focused; USB keypad triggers it hands-free
# 'r' starts/stops recording, Ctrl-C to exit
# python bridge.py --list-keypads     # Linux diagnostic: list evdev devices
```

**Camera dry-run / setup check (Windows):**

```powershell
.\.venv\Scripts\python.exe .\camera_tracking\calibrate.py
.\.venv\Scripts\python.exe .\camera_tracking\apriltag_pose.py
.\.venv\Scripts\python.exe .\painting_bridge\bridge.py --dry-run --source camera
# global Space is hold-to-spray when solenoid.enabled is true
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
- `camera_tracking/README.md` — camera-rig cold-start (calibration, tag print/measure, reference-tag placement)
- `painting_bridge/bridge.py` — bridge entry (argparse near `:606`)
- `painting_bridge/quest_reader.py` — Quest/OpenVR source for bridge teleop
- `painting_bridge/camera_reader.py` — AprilTag camera source for bridge teleop
- `painting_bridge/keypad_reader.py` — USB HID keypad paint trigger (Linux-only, evdev)
- `painting_bridge/windows_hotkey_reader.py` — Windows global Space hold-to-spray trigger
- `painting_bridge/one_euro.py` — One Euro low-pass filter used by the camera source
- `painting_bridge/teensy_reader.py` — Teensy serial source with Windows/Linux auto-detect
- `painting_bridge/config.yaml` — all tunables (including `camera:` and `keypad:` blocks)
- `camera_tracking/` — AprilTag rig (`calibrate.py`, `apriltag_pose.py`, `tag_detector.py`, `camera.py`, `config.yaml`)
- `quest.py` — Quest controller coordinate viewport/debugger
- `painting_bridge/recording.py` — trajectory file writer
- `.venv/hikvision.py` — Windows playback GUI
- `recorder.py` — separate GUI for drag-teach record/playback (alternative to the bridge for non-handle workflows)
- `io_monitor.py` — live DI/DO monitor; useful for verifying the paint valve before a run
- `lua_manager.py` + `di_do_passthrough.lua` — controller-side DI→DO passthrough for host-less bench tests
- `fairino/Robot.py` — patched SDK (do not commit Robot.c upstream)
