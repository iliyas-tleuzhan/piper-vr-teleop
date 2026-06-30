import numpy as np

from piper_vr.human_arm_model import HumanArmConfig
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.piper_driver import JointPose
from piper_vr.session import JointMimicSession
from piper_vr.types import QuestSample, TeleopState


class FakeJointDriver:
    def __init__(self, pose=True, dry_run=False):
        self.pose = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
        self.feedback_available = pose
        self.dry_run = dry_run
        self.has_sent_joint_command = False
        self.sent = []
        self.holds = 0

    def read_joint_pose(self):
        return self.pose if self.feedback_available else None

    def send_joint_pose(self, joints_deg):
        self.pose = JointPose(np.asarray(joints_deg, dtype=float))
        self.has_sent_joint_command = True
        self.sent.append(self.pose)

    def hold_joints(self, allow_last_command_fallback=False):
        self.holds += 1
        if not allow_last_command_fallback:
            raise RuntimeError("fallback not allowed")
        self.send_joint_pose(self.pose.joints_deg)


def sample(buttons=None, age=0.0, x=0.4):
    transform = np.eye(4)
    transform[:3, 3] = [x, 0.0, 0.0]
    return QuestSample(0.0, "test", {"right": transform}, buttons or {}, age)


def make_session():
    return JointMimicSession(
        side="right",
        deadman_button="rightGrip",
        calibrate_button="A",
        human_config=HumanArmConfig.from_config({}),
        mimic_config=JointMimicConfig.from_config({"smoothing_alpha": 1.0, "max_joint_speed_deg_s": [1000] * 6}),
        stale_timeout_s=0.25,
    )


def test_joint_session_requires_calibration_and_repress():
    session = make_session()
    driver = FakeJointDriver()
    assert session.step(sample(), driver).state == TeleopState.WAITING_FOR_CALIBRATION
    calibrated = session.step(sample({"A": True, "rightGrip": (1.0,)}), driver)
    assert calibrated.state == TeleopState.READY_IDLE
    assert calibrated.calibrated
    assert session.step(sample({"rightGrip": (1.0,)}), driver).reason == "release_deadman_required"
    assert session.step(sample({"rightGrip": (0.0,)}), driver).reason == "deadman_released"
    assert session.step(sample({"rightGrip": (1.0,)}), driver).reason == "armed_this_cycle"
    sent = session.step(sample({"rightGrip": (1.0,)}, x=0.42), driver)
    assert sent.action == "sent"
    assert driver.sent


def test_joint_session_stale_tracking_holds():
    session = make_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}), driver)
    stale = session.step(sample({"rightGrip": (1.0,)}, age=1.0), driver)
    assert stale.state == TeleopState.HOLDING
    assert stale.reason == "tracking_stale"
    assert driver.sent


def test_joint_session_refuses_real_calibration_without_feedback():
    session = make_session()
    driver = FakeJointDriver(pose=False, dry_run=False)
    calibrated = session.step(sample({"A": True}), driver)
    assert calibrated.state == TeleopState.WAITING_FOR_CALIBRATION
    assert calibrated.reason == "joint_feedback_required_for_calibration"
    assert not driver.sent


def test_joint_session_allows_dry_run_calibration_without_feedback():
    session = make_session()
    driver = FakeJointDriver(pose=False, dry_run=True)
    calibrated = session.step(sample({"A": True}), driver)
    assert calibrated.state == TeleopState.READY_IDLE
    assert calibrated.calibrated
    assert not driver.sent


def test_joint_session_deadman_release_faults_without_feedback_or_command():
    session = make_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}), driver)
    driver.feedback_available = False
    released = session.step(sample({"rightGrip": (0.0,)}), driver)
    assert released.state == TeleopState.FAULT
    assert released.reason == "joint_feedback_required_for_hold"
