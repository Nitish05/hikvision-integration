"""OpenVR/Quest reader for the painting bridge.

Publishes Quest controller pose as the same HandleSample stream used by the
Teensy reader:
  pos   = [x, y, z] in millimeters, fixed headset frame
  q_rel = [w, x, y, z] controller orientation in that same frame

Coordinate convention:
  +X right from headset, +Y away/forward from headset, +Z up.

B/Y reset behavior:
  While reset is held, samples are marked not-clean so the bridge holds.
  On release, one clean sample is marked rezero=True so the robot anchors at
  its current TCP and control resumes from the controller release pose.
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from typing import List, Optional, Tuple

import openvr

from teensy_reader import HandleSample

log = logging.getLogger(__name__)

READ_HZ = 90

Matrix3 = List[List[float]]
Vector3 = Tuple[float, float, float]

STEAMVR_TO_HEADSET_CARTESIAN: Matrix3 = [
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
]


def mat3_transpose(m: Matrix3) -> Matrix3:
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]


def mat3_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    return [
        [
            a[row][0] * b[0][col]
            + a[row][1] * b[1][col]
            + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def mat3_vec_mul(m: Matrix3, v: Vector3) -> Vector3:
    x, y, z = v
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def matrix_to_pos_rot(matrix) -> Tuple[Vector3, Matrix3]:
    pos = (
        matrix[0][3] * 1000.0,
        matrix[1][3] * 1000.0,
        matrix[2][3] * 1000.0,
    )
    rot = [
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]],
    ]
    return pos, rot


def rot_to_quat(m: Matrix3) -> Tuple[float, float, float, float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def transform_to_reference_frame(
    pos: Vector3,
    rot: Matrix3,
    ref_pos: Vector3,
    ref_rot: Matrix3,
) -> Tuple[Vector3, Matrix3]:
    ref_inv = mat3_transpose(ref_rot)
    delta = (
        pos[0] - ref_pos[0],
        pos[1] - ref_pos[1],
        pos[2] - ref_pos[2],
    )
    rel_pos_steam = mat3_vec_mul(ref_inv, delta)
    rel_rot_steam = mat3_mul(ref_inv, rot)

    basis = STEAMVR_TO_HEADSET_CARTESIAN
    basis_inv = mat3_transpose(basis)
    out_pos = mat3_vec_mul(basis, rel_pos_steam)
    out_rot = mat3_mul(mat3_mul(basis, rel_rot_steam), basis_inv)
    return out_pos, out_rot


class QuestReader(threading.Thread):
    def __init__(
        self,
        out_q: queue.Queue,
        read_hz: int = READ_HZ,
        controller_role: str = "right",
    ):
        super().__init__(name="QuestReader", daemon=True)
        self.out_q = out_q
        self.read_hz = read_hz
        self.controller_role = controller_role
        self._stop_evt = threading.Event()
        self._seq = 0
        self._reset_was_pressed = False
        self._rezero_on_next_release = False

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        try:
            openvr.init(openvr.VRApplication_Background)
            vr = openvr.VRSystem()
        except Exception as e:
            log.error("OpenVR init failed: %s", e)
            return

        log.info("quest source opened through OpenVR")
        period = 1.0 / max(1, self.read_hz)
        hmd_reference: Optional[Tuple[Vector3, Matrix3]] = None
        controller_anchor_pos: Optional[Vector3] = None
        controller_anchor_rot: Optional[Matrix3] = None
        t_status = 0.0

        try:
            while not self._stop_evt.is_set():
                start = time.perf_counter()
                poses = vr.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding,
                    0,
                    openvr.k_unMaxTrackedDeviceCount,
                )

                hmd_pose = None
                controllers = []

                for i in range(openvr.k_unMaxTrackedDeviceCount):
                    device_class = vr.getTrackedDeviceClass(i)
                    if device_class not in (
                        openvr.TrackedDeviceClass_HMD,
                        openvr.TrackedDeviceClass_Controller,
                    ):
                        continue
                    pose = poses[i]
                    if not pose.bPoseIsValid:
                        continue
                    pos_rot = matrix_to_pos_rot(pose.mDeviceToAbsoluteTracking)
                    if device_class == openvr.TrackedDeviceClass_HMD:
                        hmd_pose = pos_rot
                    else:
                        controllers.append((i, pos_rot))

                if hmd_reference is None and hmd_pose is not None:
                    hmd_reference = hmd_pose
                    log.info("quest headset coordinate frame locked")

                if hmd_reference is None or not controllers:
                    now = time.monotonic()
                    if now - t_status >= 2.0:
                        t_status = now
                        if hmd_reference is None:
                            log.info("waiting for Quest headset pose from OpenVR")
                        else:
                            log.info("waiting for Quest controller pose from OpenVR")
                    time.sleep(period)
                    continue

                selected = self._select_controller(vr, controllers)
                if selected is None:
                    now = time.monotonic()
                    if now - t_status >= 2.0:
                        t_status = now
                        log.info("waiting for selected Quest controller role: %s",
                                 self.controller_role)
                    time.sleep(period)
                    continue

                device_index, (ctrl_pos, ctrl_rot) = selected
                trigger_pressed, reset_pressed = self._read_buttons(vr, device_index)

                if reset_pressed:
                    self._rezero_on_next_release = True

                pos, rot = transform_to_reference_frame(
                    ctrl_pos,
                    ctrl_rot,
                    hmd_reference[0],
                    hmd_reference[1],
                )

                rezero = (
                    self._rezero_on_next_release
                    and self._reset_was_pressed
                    and not reset_pressed
                )
                if controller_anchor_pos is None or rezero:
                    controller_anchor_pos = pos
                    controller_anchor_rot = rot

                anchor_inv = mat3_transpose(controller_anchor_rot)
                rel_pos = (
                    pos[0] - controller_anchor_pos[0],
                    pos[1] - controller_anchor_pos[1],
                    pos[2] - controller_anchor_pos[2],
                )
                rel_rot = mat3_mul(anchor_inv, rot)

                self._seq += 1
                sample = HandleSample(
                    t_mono=time.monotonic(),
                    pos=[rel_pos[0], rel_pos[1], rel_pos[2]],
                    q_rel=list(rot_to_quat(rel_rot)),
                    seq=self._seq,
                    trilat_ok=not reset_pressed,
                    rezero=rezero,
                    button=trigger_pressed,
                )
                if sample.rezero:
                    self._rezero_on_next_release = False
                    log.info("quest reset released: re-anchor requested")
                self._reset_was_pressed = reset_pressed

                try:
                    self.out_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.out_q.put_nowait(sample)
                except queue.Full:
                    pass

                elapsed = time.perf_counter() - start
                time.sleep(max(0.0, period - elapsed))
        finally:
            try:
                openvr.shutdown()
            except Exception:
                pass
            log.info("quest reader thread exiting")

    def _select_controller(self, vr, controllers):
        if self.controller_role in ("left", "right"):
            desired = (
                openvr.TrackedControllerRole_LeftHand
                if self.controller_role == "left"
                else openvr.TrackedControllerRole_RightHand
            )
            for device_index, pos_rot in controllers:
                try:
                    if vr.getControllerRoleForTrackedDeviceIndex(device_index) == desired:
                        return device_index, pos_rot
                except Exception:
                    pass
        return controllers[0]

    @staticmethod
    def _read_buttons(vr, device_index: int) -> Tuple[bool, bool]:
        try:
            ok, state = vr.getControllerState(device_index)
        except openvr.OpenVRError:
            return False, False
        if not ok:
            return False, False

        trigger_mask = 1 << openvr.k_EButton_SteamVR_Trigger
        trigger_pressed = bool(state.ulButtonPressed & trigger_mask)
        try:
            trigger_pressed = trigger_pressed or state.rAxis[1].x > 0.5
        except Exception:
            pass

        reset_button_ids = {
            openvr.k_EButton_ApplicationMenu,
            getattr(openvr, "k_EButton_IndexController_B", openvr.k_EButton_ApplicationMenu),
        }
        reset_pressed = any(
            bool(state.ulButtonPressed & (1 << button_id))
            for button_id in reset_button_ids
        )
        return trigger_pressed, reset_pressed
