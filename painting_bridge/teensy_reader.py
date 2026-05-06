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
from typing import Optional

import serial

log = logging.getLogger(__name__)

# Teensy USB VID. Any PID from PJRC works (Teensy 4.x enumerates as ACM).
TEENSY_VID_SUBSTR = "16C0"

# Order matches main.cpp Serial.print order. A sample is "complete" once we
# have seen all required keys since the last emission.
POS_KEYS = ("x", "y", "z")
QUAT_KEYS = ("qw", "qx", "qy", "qz")
SAMPLE_KEYS = POS_KEYS + QUAT_KEYS


@dataclass
class HandleSample:
    t_mono: float                  # host monotonic timestamp (s)
    pos: list                      # [x, y, z] handle position in world frame (mm)
    q_rel: list                    # [w, x, y, z] qRel = qRef^-1 * qCurrent
    seq: int                       # monotonic sample counter
    trilat_ok: bool                # False if firmware reported a fresh trilat failure
    rezero: bool = False           # True if this sample coincided with a rezero event
    button: Optional[bool] = None  # last-known switch state, None = never reported


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

            if key not in SAMPLE_KEYS:
                continue
            try:
                self._partial[key] = float(val)
            except ValueError:
                continue

            # Emit when xyz + quaternion are all present.
            if all(k in self._partial for k in SAMPLE_KEYS):
                pos = [self._partial[k] for k in POS_KEYS]
                q_rel = [self._partial[k] for k in QUAT_KEYS]
                self._partial.clear()
                self._seq += 1
                sample = HandleSample(
                    t_mono=time.monotonic(),
                    pos=pos,
                    q_rel=q_rel,
                    seq=self._seq,
                    trilat_ok=not self._trilat_fail_pending,
                    rezero=rezero_pending,
                    button=self._button_state,
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
