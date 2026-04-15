"""Trajectory recorder for the painting bridge.

Writes files byte-compatible with recorder.py's Play (Servo+DO) loader.
Runs on its own thread so it never stalls the servo loop. All robot-state
reads are from the SDK's cached state packet (robot_state_pkg), so they
are essentially free — no XML-RPC per cycle.

File format (verbatim recorder.py:220-234):

    Header:  "{N},1,{period_ms},{tool},{diconfig},{doconfig}\\n"
    Row:     "j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,5,trigger,17\\n"
              floats -> %.4f, FRTYPE=5 (FR5), trailer constant 17
"""

from __future__ import annotations

import datetime as _dt
import logging
import os
import threading
import time
from typing import List, Optional

log = logging.getLogger(__name__)

# Fixed-width placeholder for the header's N field. Keeps header byte length
# constant so we can seek(0) + rewrite on stop without shifting rows.
_N_WIDTH = 10
FRTYPE = 5
TRAILER = 17


def format_row(joints: List[float], tcp: List[float], trigger: int,
               frtype: int = FRTYPE, trailer: int = TRAILER) -> str:
    """Single recorded row, comma-delimited, %.4f precision, trailing \\n."""
    if len(joints) != 6 or len(tcp) != 6:
        raise ValueError("joints and tcp must each be length 6")
    floats = list(joints) + list(tcp)
    body = ",".join(f"{v:.4f}" for v in floats)
    return f"{body},{int(frtype)},{int(trigger)},{int(trailer)}\n"


def format_header(n_points: int, period_s: float, tool: int = 0,
                  diconfig: int = 1, doconfig: int = 1) -> str:
    period_ms = int(round(period_s * 1000))
    return (f"{int(n_points):0{_N_WIDTH}d},1,{period_ms},"
            f"{int(tool)},{int(diconfig)},{int(doconfig)}\n")


def default_filename(prefix: str) -> str:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{stamp}.txt"


class TrajectoryRecorder:
    """Background thread that samples the robot's cached state at a fixed
    period and appends rows to a recorder.py-compatible text file."""

    def __init__(self, servo, period_s: float, out_dir: str,
                 filename_prefix: str = "run", tool: int = 0):
        self.servo = servo
        self.period_s = float(period_s)
        self.out_dir = out_dir
        self.filename_prefix = filename_prefix
        self.tool = int(tool)
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._file = None
        self._n = 0
        self._path: Optional[str] = None
        self._lock = threading.Lock()   # guards start/stop so 'r' spam is safe

    def is_recording(self) -> bool:
        return self._running.is_set()

    def start(self) -> Optional[str]:
        """Open a new timestamped file and start the sampler thread. Returns
        the file path on success, or None if already recording."""
        with self._lock:
            if self._running.is_set():
                log.warning("recorder already running: %s", self._path)
                return None
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(self.out_dir,
                                default_filename(self.filename_prefix))
            f = open(path, "w", buffering=1)  # line-buffered
            # Placeholder header; rewritten on stop() with the correct N.
            f.write(format_header(0, self.period_s, tool=self.tool))
            self._file = f
            self._path = path
            self._n = 0
            self._running.set()
            self._thread = threading.Thread(
                target=self._run, name="TrajectoryRecorder", daemon=True)
            self._thread.start()
            log.info("recording -> %s", path)
            return path

    def stop(self) -> Optional[str]:
        """Stop the thread, finalize the header with the true row count,
        close the file. Idempotent and safe to call from any thread."""
        with self._lock:
            if not self._running.is_set():
                return None
            self._running.clear()
            t = self._thread
            self._thread = None
        if t is not None:
            t.join(timeout=2.0)
        with self._lock:
            f = self._file
            path = self._path
            n = self._n
            self._file = None
            self._path = None
            if f is None:
                return None
            try:
                f.flush()
                f.seek(0)
                f.write(format_header(n, self.period_s, tool=self.tool))
                f.flush()
            except Exception as e:
                log.warning("failed to rewrite header: %s", e)
            try:
                f.close()
            except Exception:
                pass
            log.info("recording stopped: %s (%d rows)", path, n)
            return path

    def _run(self) -> None:
        period = self.period_s
        next_t = time.perf_counter() + period
        while self._running.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
            next_t += period
            # Skip-ahead if we fell badly behind (shouldn't happen; cache reads are µs).
            if time.perf_counter() - next_t > period * 5:
                next_t = time.perf_counter() + period

            try:
                ej, joints = self.servo.robot.GetActualJointPosDegree()
                et, tcp = self.servo.robot.GetActualTCPPose()
            except Exception as e:
                log.debug("recorder state-read error: %s", e)
                continue
            if ej != 0 or et != 0 or joints is None or tcp is None:
                continue

            trigger = 1 if getattr(self.servo, "last_do_state", False) else 0
            row = format_row(list(joints), list(tcp), trigger)
            try:
                # File access is only contended with start/stop; self._file is
                # live for the entire _run() duration per the _running flag.
                f = self._file
                if f is not None:
                    f.write(row)
                    self._n += 1
            except Exception as e:
                log.warning("recorder write failed: %s", e)
                break
