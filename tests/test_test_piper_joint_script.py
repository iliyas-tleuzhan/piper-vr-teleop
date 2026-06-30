import numpy as np

from piper_vr.piper_driver import JointPose
from scripts.test_piper_joint import run_joint_step_test


class FakeDriver:
    def __init__(self):
        self.pose = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
        self.sent = []

    def read_joint_pose(self):
        return self.pose

    def send_joint_pose(self, joints_deg):
        self.pose = JointPose(np.asarray(joints_deg, dtype=float))
        self.sent.append(self.pose.joints_deg.copy())


def test_run_joint_step_test_repeats_commands():
    driver = FakeDriver()
    start, target, after = run_joint_step_test(driver, joint=2, delta_deg=3.0, duration_s=0.05, rate_hz=100.0)
    np.testing.assert_allclose(start, [0.0, 90.0, -90.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(target, [0.0, 93.0, -90.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(after, target)
    assert len(driver.sent) >= 2
