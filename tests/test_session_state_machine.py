import numpy as np

from piper_vr.piper_driver import EndPose
from piper_vr.safety import OrientationLimiter, SafetyLimiter, SignalFilter
from piper_vr.session import TeleopSession
from piper_vr.types import QuestSample, TeleopState
from piper_vr.vr_mapping import AxisMapping


class FakeDriver:
    def __init__(self):
        self.last_pose = EndPose(np.array([0.35, 0.0, 0.25]), np.zeros(3))
        self.sent = []
        self.holds = 0

    def read_end_pose(self):
        return self.last_pose

    def send_end_pose(self, xyz_m, rpy_deg):
        self.last_pose = EndPose(np.asarray(xyz_m, dtype=float), np.asarray(rpy_deg, dtype=float))
        self.sent.append(self.last_pose)

    def hold(self):
        self.holds += 1


def sample(buttons=None, age=0.0, x=0.0):
    transform = np.eye(4)
    transform[:3, 3] = [x, 0.0, 0.0]
    return QuestSample(0.0, "test", {"right": transform}, buttons or {}, age)


def make_session():
    return TeleopSession(
        side="right",
        deadman_button="rightGrip",
        calibrate_button="A",
        scale=1.0,
        mapping=AxisMapping(),
        safety=SafetyLimiter(np.array([0.0, -1.0, 0.0]), np.array([1.0, 1.0, 1.0]), 10.0, max_position_jump_m=1.0),
        position_filter=SignalFilter(0.0, 1.0),
        orientation_safety=OrientationLimiter(1000.0),
        orientation_filter=SignalFilter(0.0, 1.0),
        stale_timeout_s=0.25,
    )


def test_session_requires_calibration_and_repress():
    session = make_session()
    driver = FakeDriver()
    assert session.step(sample(), driver).state == TeleopState.WAITING_FOR_CALIBRATION
    calibrated = session.step(sample({"A": True, "rightGrip": (1.0,)}), driver)
    assert calibrated.state == TeleopState.READY_IDLE
    assert calibrated.calibrated
    assert session.step(sample({"rightGrip": (1.0,)}), driver).reason == "release_deadman_required"
    assert session.step(sample({"rightGrip": (0.0,)}), driver).reason == "deadman_released"
    armed = session.step(sample({"rightGrip": (1.0,)}), driver)
    assert armed.reason == "armed_this_cycle"
    sent = session.step(sample({"rightGrip": (1.0,)}, x=0.1), driver)
    assert sent.action == "sent"


def test_stale_tracking_holds_and_requires_repress():
    session = make_session()
    driver = FakeDriver()
    session.step(sample({"A": True}), driver)
    stale = session.step(sample({"rightGrip": (1.0,)}, age=1.0), driver)
    assert stale.state == TeleopState.HOLDING
    assert stale.reason == "tracking_stale"
    assert driver.holds >= 1
