"""Intrinsic calibration of the Arducam UC-844 (OV9281) from a checkerboard.

Workflow:
  1. Print a checkerboard and measure it; set checkerboard.{cols,rows,square_mm}
     in config.yaml. cols/rows count INNER corners (where 4 squares meet).
  2. Run this script. A live window opens.
  3. Hold the board in view, tilt/move it, and press SPACE to capture each view.
     Aim for 15-25 views spread across the frame: centre, corners, near, far,
     and tilted both ways. The detected grid is drawn green when locked.
  4. Press C to calibrate, Q to quit.

Output: calibration.npz (camera matrix K, distortion dist, image_size, rms).

    .venv/bin/python camera_tracking/calibrate.py
"""

from __future__ import annotations

import sys

import cv2
import numpy as np

from camera import load_config, open_camera, read_gray


def build_object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """One board's 3D corner grid, in mm, on the Z=0 plane.

    Scaling by square_mm makes calibrateCamera report translations in mm, and
    keeps the unit consistent with the AprilTag size used in apriltag_pose.py.
    """
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_mm)
    return objp


def find_corners(gray: np.ndarray, pattern: "tuple[int, int]"):
    """Locate the checkerboard. Returns (found, corners) with sub-pixel corners.

    Prefers findChessboardCornersSB (robust, sub-pixel native); falls back to
    the classic detector + cornerSubPix on older OpenCV builds.
    """
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        return bool(found), corners

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return bool(found), corners


def calibrate(objpoints, imgpoints, image_size, out_path):
    """Run calibrateCamera, report errors, and save the result."""
    print(f"\n[calib] running on {len(objpoints)} views ...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    # Per-view reprojection error -- spots a bad/blurred capture.
    print(f"[calib] overall RMS reprojection error: {rms:.4f} px")
    worst = 0.0
    for i, (op, ip) in enumerate(zip(objpoints, imgpoints)):
        proj, _ = cv2.projectPoints(op, rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(ip, proj, cv2.NORM_L2) / len(proj)
        worst = max(worst, err)
        print(f"  view {i:2d}: {err:.4f} px")
    print(f"[calib] worst view: {worst:.4f} px")

    if rms > 0.5:
        print("[calib] WARNING: RMS > 0.5 px. Recapture with sharper, more "
              "varied views (tilts, frame corners, different distances).")

    print("\n[calib] camera matrix K:\n", K)
    print("[calib] distortion coeffs:", dist.ravel())

    np.savez(
        out_path,
        K=K, dist=dist,
        image_size=np.array(image_size),
        rms=np.array(rms),
    )
    print(f"[calib] saved -> {out_path}")
    return K, dist


def main() -> int:
    cfg = load_config()
    cb = cfg["checkerboard"]
    pattern = (int(cb["cols"]), int(cb["rows"]))
    objp = build_object_points(pattern[0], pattern[1], cb["square_mm"])
    min_views = int(cfg["calibration"]["min_views"])
    out_path = cfg["calibration"]["path"]

    cap, _ = open_camera(cfg)

    objpoints: list = []
    imgpoints: list = []
    image_size = None
    last_corners = None

    print(f"\n[calib] pattern={pattern} inner corners, square={cb['square_mm']} mm")
    print("[calib] SPACE = capture view   C = calibrate   Q = quit\n")

    win = "calibrate (SPACE capture / C calibrate / Q quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame, gray = read_gray(cap)
            if not ok:
                print("[calib] frame grab failed")
                break
            image_size = gray.shape[::-1]  # (w, h)

            found, corners = find_corners(gray, pattern)
            last_corners = corners if found else None
            if found:
                cv2.drawChessboardCorners(frame, pattern, corners, found)

            status = "BOARD LOCKED - press SPACE" if found else "no board"
            color = (0, 255, 0) if found else (0, 0, 255)
            cv2.putText(frame, status, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, color, 2)
            cv2.putText(frame, f"views captured: {len(objpoints)}  (need >= {min_views})",
                        (12, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            cv2.imshow(win, frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                print("[calib] quit without calibrating")
                return 0
            if key == ord(" "):
                if last_corners is not None:
                    objpoints.append(objp.copy())
                    imgpoints.append(last_corners)
                    print(f"[calib] captured view {len(objpoints)}")
                else:
                    print("[calib] no board in view -- not captured")
            if key in (ord("c"), ord("C")):
                if len(objpoints) < min_views:
                    print(f"[calib] need >= {min_views} views, have {len(objpoints)}")
                    continue
                K, dist = calibrate(objpoints, imgpoints, image_size, out_path)
                # Sanity preview: distortion should visibly straighten edges.
                undist = cv2.undistort(frame, K, dist)
                cv2.imshow("undistorted preview - any key to exit", undist)
                cv2.waitKey(0)
                return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
