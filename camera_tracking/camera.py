"""Shared camera helpers for the Arducam UC-844 Rev B (OV9281) AprilTag rig.

The OV9281 is a 1 MP global-shutter *monochrome* sensor. It still streams MJPG
over UVC, so OpenCV hands us 3-channel BGR frames whose channels are identical
grayscale -- callers should work in single-channel gray (see read_gray).

Used by calibrate.py and apriltag_pose.py.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Union

import cv2
import yaml

HERE = Path(__file__).resolve().parent

# Device-name fragment v4l2 reports for this camera (see `v4l2-ctl --list-devices`).
ARDUCAM_NAME = "Arducam OV9281"
WINDOWS_CAMERA_PROBE_MAX_INDEX = 10
WINDOWS_EXTERNAL_NAME_HINTS = (
    "arducam",
    "ov9281",
    "usb",
    "webcam",
    "logitech",
    "c920",
)
WINDOWS_BUILTIN_NAME_HINTS = (
    "integrated",
    "built-in",
    "builtin",
    "surface camera",
    "intel",
    "front",
    "rear",
)


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


def _is_windows() -> bool:
    return sys.platform.startswith("win")


def _camera_index(device) -> Optional[int]:
    """Return a numeric OpenCV camera index, or None for path/name devices."""
    if isinstance(device, int):
        return device
    if isinstance(device, str):
        s = device.strip()
        if s.isdecimal():
            return int(s)
    return None


def windows_probe_indices(max_index: int = WINDOWS_CAMERA_PROBE_MAX_INDEX) -> list[int]:
    """Probe external-looking indices before index 0, the common built-in slot."""
    return list(range(1, int(max_index) + 1)) + [0]


def windows_directshow_devices() -> list[str]:
    """Return DirectShow camera names in OpenCV-compatible index order.

    pygrabber is optional so camera auto-detect still has a fallback on systems
    where it is not installed.
    """
    try:
        from pygrabber.dshow_graph import FilterGraph
    except Exception:
        return []
    try:
        return [str(name) for name in FilterGraph().get_input_devices()]
    except Exception:
        return []


def _looks_external_camera(name: str) -> bool:
    n = name.lower()
    if any(hint in n for hint in WINDOWS_EXTERNAL_NAME_HINTS):
        return True
    if any(hint in n for hint in WINDOWS_BUILTIN_NAME_HINTS):
        return False
    return False


def windows_auto_indices() -> list[int]:
    names = windows_directshow_devices()
    if not names:
        return windows_probe_indices()

    external = [i for i, name in enumerate(names) if _looks_external_camera(name)]
    remainder = [i for i in range(len(names)) if i not in external]
    return external + remainder


def find_device(device: Union[str, int]) -> Union[str, int]:
    """Resolve a config `device` value to a camera device.

    "auto" parses `v4l2-ctl --list-devices` for the Arducam block and returns
    its first node on Linux. On Windows, explicit numeric values are OpenCV
    camera indices; "auto" is resolved by open_camera() with DirectShow probing.
    Anything else is returned unchanged.
    """
    if _is_windows():
        if device == "auto":
            return device
        idx = _camera_index(device)
        return idx if idx is not None else device
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


def _opencv_set(cap: cv2.VideoCapture, prop, value, label: str) -> bool:
    """Best-effort OpenCV property set with readback logging."""
    try:
        ok = bool(cap.set(prop, value))
        actual = cap.get(prop)
    except Exception as e:
        print(f"[camera] warning: could not set {label}={value}: {e}")
        return False
    if ok:
        print(f"[camera] {label}={value} (readback {actual})")
    else:
        print(f"[camera] warning: backend rejected {label}={value} "
              f"(readback {actual})")
    return ok


def _windows_set_manual_exposure(cap: cv2.VideoCapture, cam: dict) -> None:
    """Best-effort DirectShow/UVC exposure control.

    Windows OpenCV exposure units are backend/driver-specific. With DirectShow,
    many UVC cameras use negative log2-ish exposure values such as -5..-9,
    while Linux v4l2 uses exposure_time_absolute units. Keep them separate.
    """
    auto_value = float(cam.get("windows_auto_exposure", 0.25))
    exposure = cam.get("windows_exposure", None)
    if exposure is None:
        exposure = cam.get("exposure", None)
    if exposure is None:
        print("[camera] warning: manual_exposure is enabled on Windows but "
              "camera.windows_exposure is unset")
        return

    _opencv_set(cap, cv2.CAP_PROP_AUTO_EXPOSURE, auto_value,
                "CAP_PROP_AUTO_EXPOSURE")
    _opencv_set(cap, cv2.CAP_PROP_EXPOSURE, float(exposure),
                "CAP_PROP_EXPOSURE")


def _configure_capture(cap: cv2.VideoCapture, cam: dict) -> None:
    fourcc = cam.get("fourcc", "MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
    cap.set(cv2.CAP_PROP_FPS, cam["fps"])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)


def _open_capture(device, backend) -> cv2.VideoCapture:
    return cv2.VideoCapture(device, backend)


def _open_windows_index(index: int, cam: dict) -> cv2.VideoCapture:
    cap = _open_capture(index, cv2.CAP_DSHOW)
    if cap.isOpened():
        _configure_capture(cap, cam)
    return cap


def _open_first_windows_camera(cam: dict) -> "tuple[cv2.VideoCapture, int]":
    names = windows_directshow_devices()
    if names:
        print("[camera] DirectShow devices: " +
              ", ".join(f"{i}:{name}" for i, name in enumerate(names)))
        candidates = windows_auto_indices()
    else:
        max_index = int(cam.get("probe_max_index", WINDOWS_CAMERA_PROBE_MAX_INDEX))
        candidates = windows_probe_indices(max_index)
    for idx in candidates:
        cap = _open_windows_index(idx, cam)
        if cap.isOpened():
            return cap, idx
        try:
            cap.release()
        except Exception:
            pass
    raise RuntimeError(
        "failed to open a USB camera on Windows. Set camera.device to an "
        "explicit OpenCV index such as 1 in camera_tracking/config.yaml"
    )


def open_camera(cfg: dict) -> "tuple[cv2.VideoCapture, str]":
    """Open the camera per config and return (capture, resolved_device_path).

    Sets MJPG + resolution + fps. If camera.manual_exposure is true, locks
    exposure with v4l2-ctl (OpenCV's V4L2 exposure mapping is unreliable).
    """
    cam = cfg["camera"]
    device = find_device(cam.get("device", "auto"))

    if _is_windows():
        if device == "auto":
            cap, index = _open_first_windows_camera(cam)
            device_label = f"Windows camera index {index} (CAP_DSHOW)"
        else:
            index = _camera_index(device)
            if index is None:
                raise RuntimeError(
                    "Windows camera.device must be 'auto' or an OpenCV camera "
                    f"index, got {device!r}"
                )
            cap = _open_windows_index(index, cam)
            device_label = f"Windows camera index {index} (CAP_DSHOW)"
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera {device_label}")
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera {device}")
        _configure_capture(cap, cam)
        device_label = str(device)

    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera {device_label}")

    fourcc = cam.get("fourcc", "MJPG")
    # Keep >=2 camera buffers: with a single buffer the camera cannot fill the
    # next frame while we hold the only one, which halves the rate (~68 fps
    # instead of 120). 2 buffers restores full throughput at low latency.

    if cam.get("manual_exposure", False) and not _is_windows():
        # auto_exposure=1 is "Manual Mode" on UVC; 3 is aperture-priority/auto.
        _v4l2_set(str(device), auto_exposure=1)
        _v4l2_set(str(device), exposure_time_absolute=int(cam.get("exposure_time", 60)))
    elif cam.get("manual_exposure", False):
        _windows_set_manual_exposure(cap, cam)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    backend = int(cap.get(cv2.CAP_PROP_BACKEND)) if hasattr(cv2, "CAP_PROP_BACKEND") else 0
    print(f"[camera] {device_label}  {w}x{h} @ {fps:.0f} fps  "
          f"fourcc={fourcc} backend={backend}")
    return cap, device_label


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
