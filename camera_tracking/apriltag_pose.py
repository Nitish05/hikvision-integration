"""Live two-tag AprilTag pose from the Arducam UC-844 (OV9281).

Tracks a handheld MOVING tag relative to a fixed REFERENCE tag. The reference
tag, laid flat in the workspace, defines the control frame:

    +X = left / right     +Y = away from the operator     +Z = up

Because the moving tag's pose is measured *relative to the reference tag*, the
result is invariant to camera pose -- the camera can be bumped or even move and
the reported X/Y/Z does not change. No camera->robot calibration step is needed;
placing the reference tag IS the calibration.

    .venv/bin/python camera_tracking/apriltag_pose.py

Math: each tag's solvePnP pose is (R, t) in the camera frame. The moving tag in
the reference frame is  t_rel = R_refᵀ·(t_mov - t_ref),  R_rel = R_refᵀ·R_mov.

Run calibrate.py first to produce calibration.npz; set the reference/moving tag
ids and sizes in config.yaml.
"""

from __future__ import annotations

import sys
import time

import cv2
import numpy as np

from camera import load_config, open_camera, FrameGrabber
from tag_detector import TagDetector, load_intrinsics, rot_to_euler_deg, rot_to_quat

REF_COLOR = (255, 200, 0)     # cyan-ish: the fixed reference tag
MOV_COLOR = (0, 255, 0)       # green: the handheld moving tag


def _draw_tag(frame, tp, K, dist, size_mm, color, label):
    cv2.polylines(frame, [tp.corners.astype(np.int32)], True, color, 2)
    cv2.drawFrameAxes(frame, K, dist, tp.rvec, tp.tvec, size_mm * 0.5, 2)
    c = tp.corners[0]
    cv2.putText(frame, label, (int(c[0]), int(c[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def main() -> int:
    cfg = load_config()
    ap = cfg["apriltag"]
    decimate = float(ap.get("detect_decimate", 1.0))
    ref = ap["reference"]
    mov = ap["moving"]
    ref_id, mov_id = int(ref["id"]), int(mov["id"])
    ref_size, mov_size = float(ref["size_mm"]), float(mov["size_mm"])

    if ref_id == mov_id:
        print(f"[pose] reference.id and moving.id are both {ref_id} -- "
              "they must be different tags")
        return 1

    cal_path = cfg["calibration"]["path"]
    try:
        K, dist, rms = load_intrinsics(cal_path)
    except FileNotFoundError:
        print(f"[pose] {cal_path} not found -- run calibrate.py first")
        return 1

    try:
        detector = TagDetector(ap["family"], {ref_id: ref_size, mov_id: mov_size},
                               decimate, K, dist)
    except ValueError as e:
        print(f"[pose] {e}")
        return 1

    print(f"[pose] intrinsics from {cal_path} (rms={rms:.4f} px)")
    print(f"[pose] family=tag{detector.family}  reference id={ref_id} "
          f"({ref_size} mm)  moving id={mov_id} ({mov_size} mm)  detect@1/{decimate:g}")
    print("[pose] X/Y/Z = moving tag in the REFERENCE frame "
          "(+X left/right, +Y away, +Z up). Press Q to quit.\n")

    cap, _ = open_camera(cfg)
    grabber = FrameGrabber(cap)
    grabber.start()

    win = "two-tag pose (Q quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t_print = 0.0
    t_prev = time.monotonic()
    fps = 0.0
    last_seq = -1

    try:
        while True:
            seq, frame, gray = grabber.latest()
            if frame is None or seq == last_seq:
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
                continue
            last_seq = seq

            tags = {tp.id: tp for tp in detector.detect(gray)}
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                inst = 1.0 / dt
                fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
            do_print = (now - t_print) >= 0.1   # throttle console to ~10 Hz
            if do_print:
                t_print = now

            ref_tp = tags.get(ref_id)
            mov_tp = tags.get(mov_id)
            if ref_tp is not None:
                _draw_tag(frame, ref_tp, K, dist, ref_size, REF_COLOR,
                          f"ref id{ref_id}")
            if mov_tp is not None:
                _draw_tag(frame, mov_tp, K, dist, mov_size, MOV_COLOR,
                          f"moving id{mov_id}")

            status = "both visible"
            if ref_tp is not None and mov_tp is not None:
                # moving tag expressed in the reference tag's frame
                t_rel = ref_tp.R.T @ (mov_tp.tvec.ravel() - ref_tp.tvec.ravel())
                R_rel = ref_tp.R.T @ mov_tp.R
                x, y, z = t_rel
                roll, pitch, yaw = rot_to_euler_deg(R_rel)
                cv2.putText(frame, f"X{x:+.0f} Y{y:+.0f} Z{z:+.0f} mm",
                            (12, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 200, 255), 2)
                if do_print:
                    qw, qx, qy, qz = rot_to_quat(R_rel)
                    print(f"ref-frame  X={x:+8.1f} Y={y:+8.1f} Z={z:+8.1f} mm  "
                          f"rpy=({roll:+7.1f},{pitch:+7.1f},{yaw:+7.1f})  "
                          f"quat=({qw:+.4f},{qx:+.4f},{qy:+.4f},{qz:+.4f})")
            else:
                missing = []
                if ref_tp is None:
                    missing.append("reference")
                if mov_tp is None:
                    missing.append("moving")
                status = " + ".join(missing) + " tag not visible"
                cv2.putText(frame, status.upper(), (12, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                if do_print:
                    print(status)

            hud = f"{fps:5.1f} fps"
            if decimate > 1.0:
                hud += f"  detect@1/{decimate:g}"
            cv2.putText(frame, hud, (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            cv2.imshow(win, frame)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    finally:
        grabber.stop()
        grabber.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
