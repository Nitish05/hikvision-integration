"""Thin wrapper around fairino.Robot.RPC for the streaming bridge.

Handles pre-flight (auto-mode + enable + error-reset), lifecycle
(ServoMoveStart / ServoMoveEnd), error-code decoding, and a MockRobot
implementation so --dry-run can exercise the full pipeline without a
physical FR5.
"""

from __future__ import annotations

import logging
import sys
import time
from typing import List, Optional, Protocol

log = logging.getLogger(__name__)

Pose = List[float]


class RobotProto(Protocol):
    def ServoMoveStart(self) -> int: ...
    def ServoMoveEnd(self) -> int: ...
    def ServoCart(self, mode: int, desc_pos: Pose, pos_gain: Pose,
                  acc: float, vel: float, cmdT: float,
                  filterT: float, gain: float) -> int: ...
    def GetActualTCPPose(self, flag: int = 1): ...
    def GetActualTCPNum(self, flag: int = 1): ...
    def Mode(self, state: int) -> int: ...
    def RobotEnable(self, state: int) -> int: ...
    def StopMotion(self) -> int: ...
    def ResetAllError(self) -> int: ...


class MockRobot:
    """In-memory stand-in that simulates a stationary FR5. Supports both
    ServoCart and ServoJ code paths for --dry-run."""

    def __init__(self, start_pose: Pose = None, cmd_period_s: float = 0.008):
        self._pose = list(start_pose) if start_pose else [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]
        self._last = list(self._pose)
        self._joints = [0.0, -60.0, 90.0, -120.0, -90.0, 0.0]
        self._in_servo = False
        self._cmd_period = cmd_period_s
        self._n_cart = 0
        self._last_call_t: Optional[float] = None

    def Mode(self, state): return 0
    def RobotEnable(self, state): return 0
    def ResetAllError(self): return 0
    def GetActualTCPPose(self, flag=1): return 0, list(self._pose)
    def GetActualTCPNum(self, flag=1): return 0, 0
    def GetActualJointPosDegree(self, flag=1): return 0, list(self._joints)
    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        # Fake IK: return reference joints perturbed by the translational delta.
        jp = list(joint_pos_ref)
        for i in range(3):
            jp[i] += 0.01 * (desc_pos[i] - self._pose[i])
        return 0, jp
    def ServoJ(self, joint_pos, axisPos, acc=0.0, vel=0.0, cmdT=0.008,
               filterT=0.0, gain=0.0, id=0):
        if not self._in_servo:
            return 1
        self._joints = list(joint_pos)
        self._n_cart += 1
        return 0
    def connect_to_robot(self): return 0
    def StopMotion(self):
        log.info("[mock] StopMotion")
        self._in_servo = False
        return 0

    def ServoMoveStart(self):
        log.info("[mock] ServoMoveStart")
        self._in_servo = True
        self._last = list(self._pose)
        self._last_call_t = None
        return 0

    def ServoMoveEnd(self):
        log.info("[mock] ServoMoveEnd (n_cart=%d)", self._n_cart)
        self._in_servo = False
        return 0

    def ServoCart(self, mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain):
        if not self._in_servo:
            log.error("[mock] ServoCart called outside servo mode")
            return 1
        for i in range(3):
            if abs(desc_pos[i] - self._last[i]) > 50.0:
                log.error("[mock] delta too large on axis %d: %.2f -> %.2f",
                          i, self._last[i], desc_pos[i])
                return 2
        self._last = list(desc_pos)
        self._pose = list(desc_pos)
        self._n_cart += 1
        self._last_call_t = time.monotonic()
        return 0


def _import_real_rpc():
    """fairino.Robot.RPC lives one dir up from painting_bridge/."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    from fairino import Robot  # type: ignore
    return Robot


class FR5Servo:
    def __init__(self, ip: str, cmd_period_s: float, pos_gain: Pose,
                 filter_t: float, dry_run: bool = False,
                 use_servoj: bool = True):
        """use_servoj: stream via ServoJ (joint-space, with per-cycle IK).
        Matches the proven pattern in recorder.py. ServoCart has been
        observed to reject motion with err=112 on this controller."""
        self.cmd_period_s = cmd_period_s
        self.pos_gain = pos_gain
        self.filter_t = filter_t
        self.dry_run = dry_run
        self.use_servoj = use_servoj
        self._in_servo = False
        self._joint_ref: Optional[list] = None  # IK warm-start / ServoJ last cmd
        self.robot: RobotProto

        if dry_run:
            log.warning("DRY-RUN: using MockRobot (no physical FR5 will move)")
            self.robot = MockRobot(cmd_period_s=cmd_period_s)
        else:
            Robot = _import_real_rpc()
            self.robot = Robot.RPC(ip)
            log.info("connected to FR5 at %s", ip)
            # recorder.py does this before any command — some SDK versions
            # need it to arm the real-time state socket.
            if hasattr(self.robot, "connect_to_robot"):
                try:
                    self.robot.connect_to_robot()
                except Exception as e:
                    log.warning("connect_to_robot raised %s (continuing)", e)

    def preflight(self) -> Pose:
        """Clear errors, set auto mode + enabled, read starting TCP pose.
        Returns the pose that will anchor all subsequent targets.

        First call after connect is often cold (XML-RPC timeout -> err=-4);
        retry transient failures instead of aborting the whole session."""
        self._soft("ResetAllError", self.robot.ResetAllError, retries=3)
        self._soft("Mode(0=auto)", lambda: self.robot.Mode(0), retries=3)
        self._soft("RobotEnable(1)", lambda: self.robot.RobotEnable(1), retries=3)
        # Settle time after enabling servos.
        time.sleep(0.3)
        err, tcp = self._retry_tuple(self.robot.GetActualTCPPose, retries=3)
        if err != 0 or tcp is None:
            raise RuntimeError(f"GetActualTCPPose failed: err={err}")
        _, tool_id = self._retry_tuple(self.robot.GetActualTCPNum, retries=2)
        # Seed the IK warm-start with the current joint angles so ServoJ
        # gets a smooth first step (no configuration flip from IK branch).
        err_j, joints = self._retry_tuple(self.robot.GetActualJointPosDegree, retries=3)
        if err_j == 0 and joints is not None:
            self._joint_ref = list(joints)
            log.info("preflight OK: tool_id=%s tcp=%s joints=%s", tool_id,
                     [round(v, 3) for v in tcp],
                     [round(v, 3) for v in joints])
        else:
            log.warning("could not read joint start pos (err=%s)", err_j)
            log.info("preflight OK: tool_id=%s tcp=%s", tool_id,
                     [round(v, 3) for v in tcp])
        return list(tcp)

    def begin(self) -> None:
        self._check("ServoMoveStart", self.robot.ServoMoveStart())
        self._in_servo = True

    def end(self) -> None:
        if self._in_servo:
            try:
                self.robot.ServoMoveEnd()
            finally:
                self._in_servo = False

    def emergency_stop(self) -> None:
        try:
            self.robot.StopMotion()
        except Exception as e:
            log.error("StopMotion failed: %s", e)
        self.end()

    # Non-fatal sentinel returned by send() when the IK step fails. The
    # bridge loop treats this like a trilat-fail: hold pose, log, resume.
    IK_FAIL = -112

    def send(self, target: Pose) -> int:
        """One streaming command. Caller owns cadence. In ServoJ mode (default)
        we IK the target against the last commanded joint pose and stream
        joint positions — matches recorder.py's proven pattern.

        IK failure returns IK_FAIL (non-fatal). Do not update the joint
        reference on failure so the next valid target is relative to the
        last known-good pose."""
        if not self.use_servoj:
            return self.robot.ServoCart(
                0, target, self.pos_gain, 0.0, 0.0,
                self.cmd_period_s, self.filter_t, 0.0,
            )
        if self._joint_ref is None:
            err_j, joints = self.robot.GetActualJointPosDegree()
            if err_j != 0 or joints is None:
                log.error("ServoJ: no joint ref and GetActualJointPosDegree err=%s", err_j)
                return err_j if err_j else 1
            self._joint_ref = list(joints)
        err, jp = self.robot.GetInverseKinRef(0, target, self._joint_ref)
        if err != 0 or jp is None:
            # IK failure is recoverable — caller will hold position.
            return self.IK_FAIL
        rc = self.robot.ServoJ(jp, [0.0, 0.0, 0.0, 0.0], 0.0, 0.0,
                               self.cmd_period_s, 0.0, 0.0)
        if rc == 0:
            self._joint_ref = jp
        return rc

    @staticmethod
    def _check(name: str, err) -> None:
        if err != 0:
            raise RuntimeError(f"{name} failed: err={err}")

    @staticmethod
    def _soft(name: str, call, retries: int = 3, delay_s: float = 0.25) -> None:
        """Retry a preflight call on transient XML-RPC timeouts (err=-4).
        Logs a warning and moves on if still failing — robot is often already
        in the desired state and a stale timeout shouldn't abort the session."""
        last = None
        for i in range(retries):
            try:
                err = call()
            except Exception as e:
                last = e
                log.warning("%s raised %s (attempt %d/%d)", name, e, i + 1, retries)
                time.sleep(delay_s)
                continue
            if err == 0:
                return
            last = err
            log.warning("%s returned err=%s (attempt %d/%d)",
                        name, err, i + 1, retries)
            time.sleep(delay_s)
        log.warning("%s giving up after %d attempts (last=%s) — continuing",
                    name, retries, last)

    @staticmethod
    def _retry_tuple(call, retries: int = 3, delay_s: float = 0.25):
        """Retry a (err, data) call until err==0 or attempts exhausted."""
        err, data = -1, None
        for i in range(retries):
            try:
                err, data = call()
            except Exception as e:
                log.warning("call raised %s (attempt %d/%d)", e, i + 1, retries)
                time.sleep(delay_s)
                continue
            if err == 0:
                return err, data
            time.sleep(delay_s)
        return err, data
