import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from fr5_servo import FR5Servo, MockRobot  # noqa: E402


class FlipMock(MockRobot):
    """MockRobot whose IK returns a joint config far from the seed —
    simulates an IK branch flip near a wrist singularity."""

    def __init__(self, flip_axis: int, flip_amount: float):
        super().__init__()
        self.flip_axis = flip_axis
        self.flip_amount = flip_amount
        self.last_servoj = None

    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        jp = list(joint_pos_ref)
        jp[self.flip_axis] += self.flip_amount  # huge jump on one joint
        return 0, jp

    def ServoJ(self, joint_pos, axisPos, acc=0.0, vel=0.0, cmdT=0.008,
               filterT=0.0, gain=0.0, id=0):
        self.last_servoj = list(joint_pos)
        return super().ServoJ(joint_pos, axisPos, acc, vel, cmdT, filterT, gain, id)


def _make_servo(limit, mock):
    s = FR5Servo(ip="x", cmd_period_s=0.01, pos_gain=[1.0] * 6, filter_t=0.0,
                 dry_run=True, use_servoj=True, max_joint_delta_deg=limit)
    s.robot = mock
    s._joint_ref = [0.0] * 6
    s._in_servo = True
    mock._in_servo = True
    return s


def test_joint_clamp_caps_branch_flip():
    mock = FlipMock(flip_axis=3, flip_amount=45.0)  # IK wants J4 to jump 45 deg
    servo = _make_servo(0.5, mock)
    rc = servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert rc == 0
    assert servo.joint_clamped == 1
    # Commanded J4 was clamped to +0.5 deg, not +45.
    assert abs(mock.last_servoj[3] - 0.5) < 1e-9
    # Other joints unchanged.
    for i in (0, 1, 2, 4, 5):
        assert abs(mock.last_servoj[i]) < 1e-9


def test_joint_clamp_negative_delta():
    mock = FlipMock(flip_axis=5, flip_amount=-30.0)
    servo = _make_servo(0.5, mock)
    servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert servo.joint_clamped == 1
    assert abs(mock.last_servoj[5] - (-0.5)) < 1e-9


def test_joint_clamp_no_op_within_limit():
    mock = FlipMock(flip_axis=2, flip_amount=0.3)  # under the 0.5 limit
    servo = _make_servo(0.5, mock)
    servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert servo.joint_clamped == 0
    assert abs(mock.last_servoj[2] - 0.3) < 1e-9


def test_joint_clamp_disabled_when_none():
    mock = FlipMock(flip_axis=0, flip_amount=10.0)
    servo = _make_servo(None, mock)
    servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert servo.joint_clamped == 0
    assert abs(mock.last_servoj[0] - 10.0) < 1e-9


def test_joint_ref_advances_to_clamped_value():
    # After a clamped send, _joint_ref should track the SENT (clamped) joints,
    # not the raw IK output. So the next cycle's clamp delta is measured from
    # the clamped position, allowing a steady ramp toward the IK target.
    mock = FlipMock(flip_axis=3, flip_amount=45.0)
    servo = _make_servo(0.5, mock)
    servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert abs(servo._joint_ref[3] - 0.5) < 1e-9
    # Second cycle: IK still wants the seed + 45 (FlipMock is stateless wrt
    # the seed). New seed is 0.5, so IK returns 0.5 + 45 = 45.5; clamp to
    # 0.5 + 0.5 = 1.0.
    servo.send([300.0, 0.0, 400.0, 180.0, 0.0, 0.0])
    assert abs(servo._joint_ref[3] - 1.0) < 1e-9
    assert servo.joint_clamped == 2
