import numpy as np
import pytest

from piper_vr.piper_driver import JointPose, PiperDriver


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
