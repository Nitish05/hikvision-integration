# Fairino FR5 - Trajectory Recorder with Solenoid Trigger

Record robot trajectories via drag teaching while capturing solenoid trigger events from a physical switch. Play back both the motion and trigger events simultaneously.

Built for the **Fairino FR5** cobot (controller FRC100-AC, firmware V3.8.7-QX).

## Hardware Setup

### Wiring (NPN mode - factory default)

The FR5 controller uses **NPN digital inputs** by default (active LOW).

**Switch (DI0):**
```
DI0 (bottom connector, pin 1 bottom row) ---- Switch (NO) ---- E-0V/GND (pin 5 bottom row)
```

**Solenoid (DO0):**
```
E-24V (bottom connector, pin 5 top row) ---- Solenoid (+) ---- Solenoid (-) ---- DO0 (pin 1 top row)
```

DO0 sinks to GND when active (NPN output, max 1A per channel). Use an external relay if solenoid draws more than 1A.

### Controller I/O Pinout (Bottom Connector)

| Pin | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|-----|---|---|---|---|---|---|---|---|---|---|
| Top row | DO0 | DO1 | DO2 | DO3 | E-24V | DO4 | DO5 | DO6 | DO7 | E-24V |
| Bottom row | DI0 | DI1 | DI2 | DI3 | E-0V | DI4 | DI5 | DI6 | DI7 | E-0V |

## Installation

### Prerequisites

- Python 3.12 (the Fairino SDK requires Cython for build; Python 3.14 is not yet supported)
- Network connection to the FR5 controller (default IP: `192.168.57.2`)

### Setup

```bash
# Create virtual environment with Python 3.12
python3.12 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install the Fairino SDK (included in this repo)
pip install -e ./fairino
# If editable install fails due to Cython, install Cython first:
# pip install cython && pip install -e ./fairino
# Or simply run the apps from this directory - the SDK is importable from the fairino/ folder
```

## SDK Modifications

The Fairino Python SDK (`fairino/Robot.py`) has been modified to fix I/O reading on firmware V3.8.7-QX:

### Problem

The SDK's `GetDI()`, `GetToolDI()`, `GetDO()`, and `GetToolDO()` methods read from a `RobotStatePkg` ctypes struct that is updated via a background TCP socket on port 20004. The struct definition (1236 bytes) is larger than the actual packet from the robot (953 bytes), but the checksum still passes because it's calculated on the actual received data. The I/O fields read correctly from the state package.

The original SDK also had XML-RPC implementations for these methods commented out. The XML-RPC `GetDI()` does not return real-time hardware state - it returns programmatic state only.

### Changes Made

1. **`GetDO()`** - Removed `@xmlrpc_timeout` decorator that was blocking reads when `RPC.is_conect` was `False` (set during SDK init timeout). The method still reads from the state package.

2. **`GetToolDO()`** - Same fix as `GetDO()`.

3. **`GetDI()`** and **`GetToolDI()`** - Kept reading from the state package (not XML-RPC) since the state package provides real-time hardware state while the XML-RPC version does not.

## Applications

### 1. Trajectory Recorder (`recorder.py`)

Record robot trajectories with solenoid trigger events.

```bash
python recorder.py
```

**Recording:**
1. Click **"Start Tracking"** - robot enters freedrive (drag teaching) mode
2. Drag the robot along the desired path
3. Press the physical switch when you want the solenoid to fire - the I/O indicators show live state
4. Click **"Stop Tracking"** - recording stops, 3D plot shows the path with trigger points in red
5. Click **"Save"** - saves trajectory to a TPD-format text file and uploads to robot

**Playback:**
- **"Play (TPD)"** - uses the robot's built-in `MoveTPD` for trajectory playback
- **"Play (Servo+DO)"** - uses `ServoJ` with manual `SetDO` control, guaranteed to fire solenoid at recorded moments

**Note:** The robot must be in Auto mode for playback. The app switches modes automatically, but you may need to enable Auto mode from the teach pendant or web UI first.

**Data format:** Each recorded point stores `[x, y, z, rx, ry, rz, trigger]` where trigger is 0 or 1.

### 2. I/O Monitor (`io_monitor.py`)

Real-time visualization of all digital I/O on the FR5 controller.

```bash
python io_monitor.py
```

- Shows all 16 DI, 2 Tool DI, 16 DO, 2 Tool DO as colored indicators (green=HIGH, red=LOW)
- Toggle buttons for each DO/Tool DO
- **DI->DO passthrough** checkbox - when enabled, DI0 directly drives DO0 (switch triggers solenoid)
- Reads from the SDK's real-time state package at 100ms intervals

### 3. Lua Script Manager (`lua_manager.py`)

Upload and manage Lua scripts on the robot controller.

```bash
# Upload the DI0->DO0 passthrough script to the robot
python lua_manager.py upload

# Start the script (robot must be in Auto mode)
python lua_manager.py start

# Check if it's running
python lua_manager.py status

# Stop the script
python lua_manager.py stop

# List all Lua files on the robot
python lua_manager.py list
```

The included `di_do_passthrough.lua` script mirrors DI0 to DO0 every 10ms on the robot controller, independent of any Python app.

**Lua API reference (Fairino FR Lua):**
- `SetDO(id, status, smooth, thread)` - id: 0-15, status: 0/1, smooth: 0-Break/1-Serious, thread: 0-No/1-Yes
- `GetDI(id, thread)` - returns 0 (invalid) or 1 (valid)
- `WaitMs(t_ms)` - wait in milliseconds

## Configuration

All apps use these constants at the top of each file:

```python
IP = '192.168.57.2'        # Robot controller IP
CYCLETIME = 0.008           # Recording sample rate (8ms)
TOOL = 0                    # Tool coordinate system number
USER = 0                    # Workpiece coordinate system number
MOVEJ_VEL = 10.0            # Playback move speed (%)
DI_SWITCH = 0               # Digital input channel for switch
DO_SOLENOID = 0             # Digital output channel for solenoid
```

## Project Structure

```
hikvision_integration/
  fairino/                  # Fairino Python SDK (modified)
    Robot.py                # Main SDK file with I/O fixes
    setup.py                # SDK build script
  recorder.py               # Trajectory + solenoid recorder app
  io_monitor.py             # Digital I/O monitor app
  lua_manager.py            # Lua script upload/run manager
  di_do_passthrough.lua     # Lua script for DI0->DO0 passthrough
  requirements.txt          # Python dependencies
```
