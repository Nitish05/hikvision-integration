"""Camera/AprilTag reader for the painting bridge.

Tracks a handheld MOVING AprilTag relative to a fixed REFERENCE tag and
publishes the moving tag's pose as the same HandleSample stream the Teensy and
Quest readers use.

  pos    = [x, y, z] moving tag in the reference tag's frame, millimetres
  q_rel  = [w, x, y, z] moving tag orientation in the reference tag's frame

Because the pose is tag-relative, it is invariant to camera position -- the
camera may be bumped or moved without disturbing the output.

Re-anchor (rezero): there is no button. The reader emits a sample ONLY when
both tags are cleanly detected. Briefly cover the moving tag (>stream_timeout)
and the bridge watchdog holds the robot, then re-anchors on recovery -- so
"cover, reposition, uncover" is the rezero gesture.

Camera, intrinsics, and tag ids/sizes come from camera_tracking/config.yaml.
"""

from __future__ import annotations

import logging
import os
import queue
import sys
import threading
import time

# camera_tracking/ holds the shared camera + tag-detection code.
_CT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "camera_tracking"))
if _CT_DIR not in sys.path:
    sys.path.insert(0, _CT_DIR)

import cv2                                                          # noqa: E402
import numpy as np                                                 # noqa: E402

from camera import load_config, open_camera, FrameGrabber          # noqa: E402
from tag_detector import TagDetector, load_intrinsics, rot_to_quat  # noqa: E402

from teensy_reader import HandleSample
from one_euro import OneEuroPose

log = logging.getLogger(__name__)

_REF_COLOR = (255, 200, 0)    # cyan-ish: fixed reference tag
_MOV_COLOR = (0, 255, 0)      # green: handheld moving tag
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_GAP_RESET_S = 0.12           # tracking gap longer than this resets the smoother


class CameraReader(threading.Thread):
    """Publishes a moving AprilTag's reference-frame pose as HandleSample.

    With preview=True it also pops up a live camera window (tags, axes, the
    reference-frame X/Y/Z, and fps). The window is display-only; if the GUI
    cannot open it is disabled and sample publishing continues unaffected.
    """

    def __init__(self, out_q: queue.Queue, ct_config_path: str,
                 preview: bool = True, smoothing: dict = None,
                 swap_xy: bool = False):
        super().__init__(name="CameraReader", daemon=True)
        self.out_q = out_q
        self.ct_config_path = ct_config_path
        self.preview = preview
        self.swap_xy = bool(swap_xy)
        self._stop_evt = threading.Event()
        self._seq = 0
        # One Euro pose smoother (jitter low-pass before publishing).
        sm = smoothing or {}
        if sm.get("enabled", True):
            self._smoother = OneEuroPose(sm.get("position") or {},
                                         sm.get("rotation") or {},
                                         float(sm.get("dcutoff", 1.0)))
        else:
            self._smoother = None

    def stop(self) -> None:
        self._stop_evt.set()

    def _publish(self, sample: HandleSample) -> None:
        """Single-slot publish: keep only the freshest sample (drain then put)."""
        try:
            self.out_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self.out_q.put_nowait(sample)
        except queue.Full:
            pass

    @staticmethod
    def _draw(frame, K, dist, ref_tp, mov_tp, ref_size, mov_size, t_rel, fps):
        """Annotate the preview frame with tags, axes, fps, and the readout."""
        for tp, size, color, label in (
            (ref_tp, ref_size, _REF_COLOR, "ref"),
            (mov_tp, mov_size, _MOV_COLOR, "moving"),
        ):
            if tp is None:
                continue
            cv2.polylines(frame, [tp.corners.astype(np.int32)], True, color, 2)
            cv2.drawFrameAxes(frame, K, dist, tp.rvec, tp.tvec, size * 0.5, 2)
            c = tp.corners[0]
            cv2.putText(frame, f"{label} id{tp.id}", (int(c[0]), int(c[1]) - 10),
                        _FONT, 0.6, color, 2)
        cv2.putText(frame, f"{fps:5.1f} fps", (12, 32), _FONT, 0.9,
                    (0, 255, 255), 2)
        if t_rel is not None:
            x, y, z = t_rel
            cv2.putText(frame, f"X{x:+.0f} Y{y:+.0f} Z{z:+.0f} mm", (12, 70),
                        _FONT, 0.9, (0, 200, 255), 2)
        else:
            miss = "REFERENCE" if ref_tp is None else "MOVING"
            cv2.putText(frame, f"{miss} TAG NOT VISIBLE", (12, 70),
                        _FONT, 0.9, (0, 0, 255), 2)

    def run(self) -> None:
        try:
            cfg = load_config(self.ct_config_path)
        except Exception as e:
            log.error("camera reader: cannot load %s: %s", self.ct_config_path, e)
            return

        ap = cfg["apriltag"]
        ref, mov = ap["reference"], ap["moving"]
        ref_id, mov_id = int(ref["id"]), int(mov["id"])
        if ref_id == mov_id:
            log.error("camera reader: reference.id and moving.id are both %d",
                      ref_id)
            return

        cal_path = cfg["calibration"]["path"]
        try:
            K, dist, rms = load_intrinsics(cal_path)
        except FileNotFoundError:
            log.error("camera reader: %s not found -- run calibrate.py", cal_path)
            return

        try:
            detector = TagDetector(
                ap["family"],
                {ref_id: float(ref["size_mm"]), mov_id: float(mov["size_mm"])},
                float(ap.get("detect_decimate", 1.0)), K, dist)
        except ValueError as e:
            log.error("camera reader: %s", e)
            return

        try:
            cap, dev = open_camera(cfg)
        except Exception as e:
            log.error("camera reader: open_camera failed: %s", e)
            return

        grabber = FrameGrabber(cap)
        grabber.start()
        ref_size, mov_size = float(ref["size_mm"]), float(mov["size_mm"])
        log.info("camera source opened: %s  reference id=%d  moving id=%d  "
                 "(intrinsics rms=%.3f px)", dev, ref_id, mov_id, rms)

        preview = self.preview
        win = "camera reader (bridge)"
        if preview:
            try:
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            except Exception as e:
                log.warning("camera reader: preview unavailable (%s) -- disabled", e)
                preview = False

        last_frame_seq = -1
        t_status = 0.0
        t_prev = time.monotonic()
        t_last_emit = None
        fps = 0.0
        try:
            while not self._stop_evt.is_set():
                seq, frame, gray = grabber.latest()
                if frame is None or seq == last_frame_seq:
                    time.sleep(0.001)        # wait for a fresh frame
                    continue
                last_frame_seq = seq

                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                if dt > 0:
                    inst = 1.0 / dt
                    fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst

                tags = {tp.id: tp for tp in detector.detect(gray)}
                ref_tp = tags.get(ref_id)
                mov_tp = tags.get(mov_id)

                t_rel = None
                if ref_tp is not None and mov_tp is not None:
                    # moving tag expressed in the reference tag's frame
                    rel = ref_tp.R.T @ (mov_tp.tvec.ravel() - ref_tp.tvec.ravel())
                    if self.swap_xy:
                        # swap X and Y position only -- no sign flip, Z and
                        # orientation untouched
                        pos = [float(rel[1]), float(rel[0]), float(rel[2])]
                    else:
                        pos = [float(rel[0]), float(rel[1]), float(rel[2])]
                    quat = [float(v) for v in rot_to_quat(ref_tp.R.T @ mov_tp.R)]

                    # One Euro jitter low-pass, before the robot sees it.
                    if self._smoother is not None:
                        if t_last_emit is None:
                            dt_emit = 1.0 / 120.0
                        else:
                            dt_emit = now - t_last_emit
                            if dt_emit > _GAP_RESET_S:
                                # tracking gap -- start fresh, no catch-up jump
                                self._smoother.reset()
                        pos, quat = self._smoother.filter(pos, quat, dt_emit)
                    t_last_emit = now
                    t_rel = pos                       # smoothed, for the HUD

                    self._seq += 1
                    self._publish(HandleSample(
                        t_mono=now,
                        pos=pos,
                        q_rel=quat,
                        seq=self._seq,
                        trilat_ok=True,
                        rezero=False,
                        button=None,
                    ))
                elif now - t_status >= 2.0:
                    # Emit nothing -- the bridge watchdog holds the robot and
                    # re-anchors when both tags come back.
                    t_status = now
                    miss = [n for n, tp in (("reference", ref_tp),
                                            ("moving", mov_tp)) if tp is None]
                    log.info("camera: %s tag not visible -- holding",
                             " + ".join(miss))

                if preview:
                    try:
                        self._draw(frame, K, dist, ref_tp, mov_tp,
                                   ref_size, mov_size, t_rel, fps)
                        cv2.imshow(win, frame)
                        cv2.waitKey(1)
                    except Exception as e:
                        log.warning("camera reader: preview error (%s) -- "
                                    "disabled", e)
                        preview = False
        finally:
            grabber.stop()
            grabber.join(timeout=1.0)
            if preview:
                try:
                    cv2.destroyWindow(win)
                except Exception:
                    pass
            try:
                cap.release()
            except Exception:
                pass
            log.info("camera reader thread exiting")


if __name__ == "__main__":
    # Standalone smoke test: print emitted samples for ~10 s. No robot involved.
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ct = os.path.normpath(os.path.join(_CT_DIR, "config.yaml"))
    q: "queue.Queue[HandleSample]" = queue.Queue(maxsize=1)
    reader = CameraReader(out_q=q, ct_config_path=ct)
    reader.start()
    t_end = time.monotonic() + 10.0
    last_seq = -1
    try:
        while time.monotonic() < t_end and reader.is_alive():
            try:
                s = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if s.seq == last_seq:
                continue
            last_seq = s.seq
            x, y, z = s.pos
            print(f"seq={s.seq:<5d} pos=({x:+8.1f},{y:+8.1f},{z:+8.1f}) mm  "
                  f"q_rel=({s.q_rel[0]:+.4f},{s.q_rel[1]:+.4f},"
                  f"{s.q_rel[2]:+.4f},{s.q_rel[3]:+.4f})  trilat_ok={s.trilat_ok}")
    finally:
        reader.stop()
        reader.join(timeout=2.0)
