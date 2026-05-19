"""Shared AprilTag detection + pose estimation for the camera_tracking tools.

Used by apriltag_pose.py (live readout) and axis_calibrate.py (camera->robot
axis calibration). One source of truth for the detector setup, the decimation
speed trick, and the rotation conversions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

# config family string -> OpenCV predefined dictionary
_FAMILIES = {
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def load_intrinsics(path: str) -> "tuple[np.ndarray, np.ndarray, float]":
    """Load (K, dist, rms) from a calibrate.py calibration.npz. Raises FileNotFoundError."""
    cal = np.load(path)
    return cal["K"], cal["dist"], float(cal["rms"])


def rot_to_quat(R: np.ndarray) -> "tuple[float, float, float, float]":
    """Rotation matrix -> unit quaternion (w, x, y, z).

    Same branch-by-trace method as painting_bridge/quest_reader.py:rot_to_quat,
    so downstream conventions match the existing bridge.
    """
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return w / n, x / n, y / n, z / n


def rot_to_euler_deg(R: np.ndarray) -> "tuple[float, float, float]":
    """Rotation matrix -> (roll, pitch, yaw) degrees, XYZ Tait-Bryan, for display."""
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal-lock fallback
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def tag_object_points(size_mm: float) -> np.ndarray:
    """The tag's 4 corners in its own frame (mm), matching aruco's corner order.

    detectMarkers returns corners clockwise from top-left. Z points out of the
    tag face toward the camera. Edge = the outer black-square side.
    """
    h = float(size_mm) / 2.0
    return np.array([
        [-h,  h, 0.0],   # top-left
        [ h,  h, 0.0],   # top-right
        [ h, -h, 0.0],   # bottom-right
        [-h, -h, 0.0],   # bottom-left
    ], dtype=np.float32)


@dataclass
class TagPose:
    """One detected tag's pose in the CAMERA optical frame."""
    id: int
    rvec: np.ndarray      # (3,1) rotation vector
    tvec: np.ndarray      # (3,1) translation, mm (camera -> tag centre)
    R: np.ndarray         # (3,3) rotation matrix, tag -> camera
    corners: np.ndarray   # (4,2) full-resolution image corners


class TagDetector:
    """Detects AprilTags and estimates the pose of the configured tag ids.

    tag_sizes maps a tag id to its outer black-square edge length (mm). Only
    those ids are pose-solved; every other detected tag is ignored. Different
    ids may have different sizes (e.g. a large reference tag, a small handheld
    one).
    """

    def __init__(self, family: str, tag_sizes: "dict[int, float]",
                 decimate: float, K: np.ndarray, dist: np.ndarray):
        family = str(family).lower()
        if family not in _FAMILIES:
            raise ValueError(f"unknown AprilTag family '{family}'; "
                             f"choose from {list(_FAMILIES)}")
        self.family = family
        self.tag_sizes = {int(k): float(v) for k, v in tag_sizes.items()}
        self.decimate = float(decimate)
        self.K = K
        self.dist = dist
        # one object-point template per id (cached, keyed by id)
        self._objp = {tid: tag_object_points(sz)
                      for tid, sz in self.tag_sizes.items()}
        aruco_dict = cv2.aruco.getPredefinedDictionary(_FAMILIES[family])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def detect(self, gray: np.ndarray) -> "list[TagPose]":
        """Detect tags in a grayscale frame; solve pose for configured ids only."""
        d = self.decimate
        if d > 1.0:
            # Detect on a downscaled image (faster), then map corners back to
            # full resolution so solvePnP keeps full-res pose accuracy.
            small = cv2.resize(gray, None, fx=1.0 / d, fy=1.0 / d,
                               interpolation=cv2.INTER_AREA)
            corners, ids, _ = self._detector.detectMarkers(small)
            corners = tuple(c * d for c in corners)
        else:
            corners, ids, _ = self._detector.detectMarkers(gray)

        out: "list[TagPose]" = []
        if ids is None:
            return out
        for tag_corners, tag_id in zip(corners, ids.ravel()):
            tid = int(tag_id)
            objp = self._objp.get(tid)
            if objp is None:        # not a configured tag -> ignore
                continue
            img_pts = tag_corners.reshape(4, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                objp, img_pts, self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            R, _ = cv2.Rodrigues(rvec)
            out.append(TagPose(tid, rvec, tvec, R, img_pts))
        return out
