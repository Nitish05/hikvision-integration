"""Serial reader for the Teensy painting-handle firmware.

Parses the Teleplot-style stream emitted by
painting/painting/src/main.cpp and publishes the latest complete 6-DOF
sample to a single-slot queue consumed by the servo loop.
"""

from __future__ import annotations

import glob
import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import serial

log = logging.getLogger(__name__)

# Teensy USB VID. Any PID from PJRC works (Teensy 4.x enumerates as ACM).
TEENSY_VID_SUBSTR = "16C0"

# Order matches main.cpp Serial.print order. A sample is "complete" once we
# have seen all six pose keys since the last emission.
POSE_KEYS = ("x", "y", "z", "rx", "ry", "rz")
# Optional relative-rotation quaternion (qRef^-1 * qCurrent), emitted by
# firmware after the Euler lines. When all four are present the bridge uses
# proper rotation composition; if absent it falls back to Euler addition.
QUAT_KEYS = ("qw", "qx", "qy", "qz")


@dataclass
class HandleSample:
    t_mono: float                  # host monotonic timestamp (s)
    pose: list                     # [x, y, z, rx, ry, rz]
    seq: int                       # monotonic sample counter
    trilat_ok: bool                # False if firmware reported a fresh trilat failure
    rezero: bool = False           # True if this sample coincided with a rezero event
    button: Optional[bool] = None  # last-known switch state, None = never reported
    quat: Optional[Tuple[float, float, float, float]] = None  # (w,x,y,z) relative quat


def autodetect_teensy() -> Optional[str]:
    """Return a /dev/serial/by-id/* path matching a Teensy, or None."""
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        if TEENSY_VID_SUBSTR in path or "Teensy" in path:
            return path
    return None


def resolve_port(configured: str) -> str:
    if configured and configured != "auto":
        return configured
    p = autodetect_teensy()
    if p is None:
        raise RuntimeError(
            "No Teensy found on /dev/serial/by-id/. Plug it in or pass --serial."
        )
    return p


class TeensyReader(threading.Thread):
    """Background thread. Blocks on readline(), publishes only the most
    recent complete HandleSample into a queue of maxsize=1 (dropping stale
    samples the servo loop hasn't picked up yet)."""

    def __init__(self, port: str, baud: int, out_q: queue.Queue,
                 read_timeout_s: float = 0.05):
        super().__init__(name="TeensyReader", daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self.read_timeout_s = read_timeout_s
        self._stop_evt = threading.Event()
        self._ser: Optional[serial.Serial] = None
        self._seq = 0
        self._partial = {}                  # key -> float, cleared after a full sample
        self._partial_q = {}                # quat keys, cleared with _partial
        self._trilat_fail_last = 0
        self._trilat_fail_pending = False   # a failure happened since last emission
        self._button_state: Optional[bool] = None  # last observed switch state

    def stop(self) -> None:
        self._stop_evt.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    def run(self) -> None:
        try:
            self._ser = serial.Serial(
                self.port, self.baud, timeout=self.read_timeout_s
            )
        except serial.SerialException as e:
            log.error("serial open failed: %s", e)
            return
        log.info("serial opened: %s @ %d", self.port, self.baud)

        rezero_pending = False
        while not self._stop_evt.is_set():
            try:
                line = self._ser.readline()
            except (serial.SerialException, OSError, TypeError, ValueError) as e:
                # TypeError/ValueError can surface on pyserial cleanup when
                # stop() closes the port mid-readline. Treat all as shutdown.
                if not self._stop_evt.is_set():
                    log.error("serial read error: %s", e)
                break
            if not line:
                continue
            try:
                s = line.decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if not s:
                continue

            # Teleplot lines look like ">key:value". Control/banner lines are
            # ignored except for the rezero marker.
            if s.startswith(">rezero:"):
                rezero_pending = True
                continue
            if not s.startswith(">"):
                continue
            try:
                key, val = s[1:].split(":", 1)
            except ValueError:
                continue
            key = key.strip()
            val = val.strip()

            if key == "trilatFail":
                try:
                    n = int(float(val))
                except ValueError:
                    continue
                if n > self._trilat_fail_last:
                    self._trilat_fail_pending = True
                    self._trilat_fail_last = n
                continue

            if key == "button":
                try:
                    self._button_state = bool(int(float(val)))
                except ValueError:
                    pass
                continue

            if key in QUAT_KEYS:
                try:
                    self._partial_q[key] = float(val)
                except ValueError:
                    pass
                continue

            if key not in POSE_KEYS:
                continue
            try:
                self._partial[key] = float(val)
            except ValueError:
                continue

            # Emit when all 6 pose keys are present. Quat is attached only if
            # all 4 quat keys arrived since the last emission (graceful fallback
            # for old firmware that doesn't emit them).
            if len(self._partial) == 6 and all(k in self._partial for k in POSE_KEYS):
                pose = [self._partial[k] for k in POSE_KEYS]
                quat: Optional[Tuple[float, float, float, float]] = None
                if all(k in self._partial_q for k in QUAT_KEYS):
                    quat = tuple(self._partial_q[k] for k in QUAT_KEYS)  # type: ignore[assignment]
                self._partial.clear()
                self._partial_q.clear()
                self._seq += 1
                sample = HandleSample(
                    t_mono=time.monotonic(),
                    pose=pose,
                    seq=self._seq,
                    trilat_ok=not self._trilat_fail_pending,
                    rezero=rezero_pending,
                    button=self._button_state,
                    quat=quat,
                )
                self._trilat_fail_pending = False
                rezero_pending = False
                # Single-slot queue: replace any stale sample.
                try:
                    self.out_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.out_q.put_nowait(sample)
                except queue.Full:
                    pass

        log.info("reader thread exiting")
