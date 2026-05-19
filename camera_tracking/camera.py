"""Shared camera helpers for the Arducam UC-844 Rev B (OV9281) AprilTag rig.

The OV9281 is a 1 MP global-shutter *monochrome* sensor. It still streams MJPG
over UVC, so OpenCV hands us 3-channel BGR frames whose channels are identical
grayscale -- callers should work in single-channel gray (see read_gray).

Used by calibrate.py and apriltag_pose.py.
"""

from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import yaml

HERE = Path(__file__).resolve().parent

# Device-name fragment v4l2 reports for this camera (see `v4l2-ctl --list-devices`).
ARDUCAM_NAME = "Arducam OV9281"


def load_config(path: Optional[str] = None) -> dict:
    """Load config.yaml. Relative paths inside it are resolved here too."""
    cfg_path = Path(path) if path else (HERE / "config.yaml")
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    # Resolve calibration.path relative to the config file so the scripts can
    # be launched from any working directory.
    calib = cfg.get("calibration", {})
    p = Path(calib.get("path", "calibration.npz"))
    if not p.is_absolute():
        p = cfg_path.resolve().parent / p
    calib["path"] = str(p)
    cfg["calibration"] = calib
    return cfg


def find_device(device: str) -> str:
    """Resolve a config `device` value to a /dev/videoN path.

    "auto" parses `v4l2-ctl --list-devices` for the Arducam block and returns
    its first node. Anything else is returned unchanged.
    """
    if device != "auto":
        return device
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.DEVNULL
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "device=auto needs v4l2-ctl; install v4l-utils or set an explicit "
            "/dev/videoN in config.yaml"
        )

    # Output blocks: a header line, then indented /dev/... nodes until the next
    # unindented line.
    current_is_arducam = False
    for line in out.splitlines():
        if not line.strip():
            continue
        if not line.startswith(("\t", " ")):
            current_is_arducam = ARDUCAM_NAME in line
        elif current_is_arducam and "/dev/video" in line:
            return line.strip()
    raise RuntimeError(
        f"camera matching '{ARDUCAM_NAME}' not found; plug it in or set an "
        f"explicit device in config.yaml"
    )


def _v4l2_set(device: str, **controls) -> None:
    """Set v4l2 controls via v4l2-ctl. Best-effort: logs but does not raise."""
    args = ",".join(f"{k}={v}" for k, v in controls.items())
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", args],
            check=True, stderr=subprocess.PIPE, text=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        msg = getattr(e, "stderr", "") or str(e)
        print(f"[camera] warning: could not set {args}: {msg}")


def open_camera(cfg: dict) -> "tuple[cv2.VideoCapture, str]":
    """Open the camera per config and return (capture, resolved_device_path).

    Sets MJPG + resolution + fps. If camera.manual_exposure is true, locks
    exposure with v4l2-ctl (OpenCV's V4L2 exposure mapping is unreliable).
    """
    cam = cfg["camera"]
    device = find_device(cam.get("device", "auto"))

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera {device}")

    fourcc = cam.get("fourcc", "MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    cap.set(cv2.CAP_PROP_FPS, cam["fps"])
    # Keep >=2 V4L2 buffers: with a single buffer the camera cannot fill the
    # next frame while we hold the only one, which halves the rate (~68 fps
    # instead of 120). 2 buffers restores full throughput at low latency.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    if cam.get("manual_exposure", False):
        # auto_exposure=1 is "Manual Mode" on UVC; 3 is aperture-priority/auto.
        _v4l2_set(device, auto_exposure=1)
        _v4l2_set(device, exposure_time_absolute=int(cam.get("exposure_time", 60)))

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[camera] {device}  {w}x{h} @ {fps:.0f} fps  fourcc={fourcc}")
    return cap, device


def read_gray(cap: cv2.VideoCapture) -> "tuple[bool, object, object]":
    """Grab one frame. Returns (ok, bgr_frame, gray_frame).

    gray is single-channel; bgr is kept for drawing overlays in color.
    """
    ok, frame = cap.read()
    if not ok:
        return False, None, None
    if frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame
        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return True, frame, gray


class FrameGrabber(threading.Thread):
    """Background thread that always holds the newest camera frame.

    cap.read() blocks for ~one frame interval. Running it in its own thread
    lets the main loop's detection + imshow overlap that wait, so the display
    loop runs at the camera rate instead of (capture + processing) summed.

    Each grabbed frame gets a monotonically increasing seq so a consumer can
    tell a fresh frame from one it has already processed. cap.read() returns a
    fresh array every call, so the consumer may draw on its frame without a
    lock once it has been handed out.
    """

    def __init__(self, cap: cv2.VideoCapture):
        super().__init__(name="FrameGrabber", daemon=True)
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._gray = None
        self._seq = 0
        self._stop_evt = threading.Event()

    def run(self) -> None:
        while not self._stop_evt.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.001)
                continue
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            with self._lock:
                self._frame = frame
                self._gray = gray
                self._seq += 1

    def latest(self) -> "tuple[int, object, object]":
        """Return (seq, bgr_frame, gray_frame) of the newest frame.

        seq is 0 with None frames until the first grab completes.
        """
        with self._lock:
            return self._seq, self._frame, self._gray

    def stop(self) -> None:
        self._stop_evt.set()
