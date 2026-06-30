import numpy as np
import pytest

from piper_vr.piper_driver import JointPose, PiperDriver
from piper_vr.vr_teleop import command_joint_hold_on_exit


class NoFeedbackDriver(PiperDriver):
    def __init__(self):
        super().__init__(dry_run=False)
        self.sent = []

    def read_joint_pose(self, *, debug_feedback=False):
        return None

    def send_joint_pose(self, joints_deg):
        self.sent.append(np.asarray(joints_deg, dtype=float))
        self.has_sent_joint_command = True


def test_hold_joints_refuses_neutral_without_feedback_or_command():
    driver = NoFeedbackDriver()
    with pytest.raises(RuntimeError, match="Cannot hold joints safely"):
        driver.hold_joints()
    assert not driver.sent


def test_hold_joints_allows_explicit_last_command_fallback_after_send():
    driver = NoFeedbackDriver()
    driver.last_joint_command = JointPose(np.array([1.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
    driver.has_sent_joint_command = True
    driver.hold_joints(allow_last_command_fallback=True)
    np.testing.assert_allclose(driver.sent[-1], [1.0, 90.0, -90.0, 0.0, 0.0, 0.0])


class RaisingJointCtrl:
    def JointCtrl(self, *args):
        raise RuntimeError("boom")


def test_send_joint_pose_does_not_mark_sent_if_jointctrl_raises():
    driver = PiperDriver(dry_run=False)
    driver.arm = RaisingJointCtrl()
    with pytest.raises(RuntimeError, match="boom"):
        driver.send_joint_pose(np.array([1.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
    assert not driver.has_sent_joint_command


def test_exit_hold_helper_does_not_raise_without_feedback_or_command():
    driver = NoFeedbackDriver()
    assert command_joint_hold_on_exit(driver) is False
    assert not driver.sent
