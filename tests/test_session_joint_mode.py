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


def sample(buttons=None, age=0.0, x=0.4, transform=None):
    transform = np.eye(4) if transform is None else np.asarray(transform, dtype=float)
    transform[:3, 3] = [x, transform[1, 3], transform[2, 3]]
    return QuestSample(0.0, "test", {"right": transform, "hmd": np.eye(4)}, buttons or {}, age)


def make_session():
    return JointMimicSession(
        side="right",
        deadman_button="rightGrip",
        calibrate_button="A",
        human_config=HumanArmConfig.from_config({}),
        mimic_config=JointMimicConfig.from_config({"smoothing_alpha": 1.0, "max_joint_speed_deg_s": [1000] * 6}),
        stale_timeout_s=0.25,
    )


def make_relative_session(config=None):
    config = config or {}
    mimic_config = {
        "mapping_mode": "relative_delta",
        "smoothing_alpha": 1.0,
        "max_joint_speed_deg_s": [1000] * 6,
        "translation_deadband_m": 0.0,
        "rotation_deadband_deg": 0.0,
        "settle_frames_on_stop": 2,
        "cancel_backlog_on_stop": True,
        "relative_gain_matrix": [
            [30, 0, 0, 0, 0, 0],
            [0, 0, 30, 0, 0, 0],
            [0, 0, -30, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ],
        **config,
    }
    return JointMimicSession(
        side="right",
        deadman_button="rightGrip",
        calibrate_button="A",
        human_config=HumanArmConfig.from_config({}),
        mimic_config=JointMimicConfig.from_config(mimic_config),
        stale_timeout_s=0.25,
    )


def rotate_z(deg):
    angle = np.radians(deg)
    c, s = np.cos(angle), np.sin(angle)
    transform = np.eye(4)
    transform[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return transform


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
    session.step(sample({"A": True, "rightGrip": (1.0,)}), driver)
    driver.feedback_available = False
    released = session.step(sample({"rightGrip": (0.0,)}), driver)
    assert released.state == TeleopState.FAULT
    assert released.reason == "joint_feedback_required_for_hold"


def test_fresh_deadman_press_reanchors_without_jump_after_controller_moves_idle():
    session = make_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}, x=0.4), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.4), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.6), driver)

    armed = session.step(sample({"rightGrip": (1.0,)}, x=0.6), driver)
    assert armed.reason == "armed_this_cycle"
    assert not driver.sent

    sent = session.step(sample({"rightGrip": (1.0,)}, x=0.6), driver)
    assert sent.action == "sent"
    np.testing.assert_allclose(sent.human_delta_deg, np.zeros(6), atol=1e-6)
    np.testing.assert_allclose(sent.safe_joint_target_deg, driver.sent[-1].joints_deg, atol=1e-6)
    np.testing.assert_allclose(sent.safe_joint_target_deg, sent.robot_home_joints_deg, atol=1e-6)


def test_after_clutch_anchor_motion_changes_delta_and_target():
    session = make_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}, x=0.4), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.6), driver)
    session.step(sample({"rightGrip": (1.0,)}, x=0.6), driver)

    moved = session.step(sample({"rightGrip": (1.0,)}, x=0.7), driver)
    assert moved.action == "sent"
    assert np.linalg.norm(moved.human_delta_deg) > 1e-6
    assert np.linalg.norm(moved.safe_joint_target_deg - moved.robot_home_joints_deg) > 1e-6


def test_fresh_deadman_press_requires_feedback_for_real_clutch():
    session = make_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}, x=0.4), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.4), driver)
    driver.feedback_available = False

    armed = session.step(sample({"rightGrip": (1.0,)}, x=0.6), driver)
    assert armed.state == TeleopState.FAULT
    assert armed.reason == "joint_feedback_required_for_clutch"
    assert not driver.sent


def test_relative_controller_stop_does_not_continue_target_drift():
    session = make_relative_session()
    driver = FakeJointDriver()
    session.step(sample({"A": True}, x=0.0), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.0), driver)
    session.step(sample({"rightGrip": (1.0,)}, x=0.0), driver)
    moved = session.step(sample({"rightGrip": (1.0,)}, x=0.1), driver)
    first_target = moved.safe_joint_target_deg.copy()
    assert moved.action == "sent"
    stopped_1 = session.step(sample({"rightGrip": (1.0,)}, x=0.1), driver)
    stopped_2 = session.step(sample({"rightGrip": (1.0,)}, x=0.1), driver)
    assert stopped_1.action == "skipped"
    assert stopped_2.action == "skipped"
    np.testing.assert_allclose(stopped_2.safe_joint_target_deg, first_target)


def test_relative_cancel_backlog_on_stop_resets_to_measured_command():
    session = make_relative_session({"max_joint_speed_deg_s": [0.5] * 6, "relative_gain_matrix": [[300, 0, 0, 0, 0, 0]] + [[0, 0, 0, 0, 0, 0]] * 5})
    driver = FakeJointDriver()
    session.step(sample({"A": True}, x=0.0), driver)
    session.step(sample({"rightGrip": (0.0,)}, x=0.0), driver)
    session.step(sample({"rightGrip": (1.0,)}, x=0.0), driver)
    moved = session.step(sample({"rightGrip": (1.0,)}, x=0.2), driver)
    stopped = session.step(sample({"rightGrip": (1.0,)}, x=0.2), driver)
    stopped = session.step(sample({"rightGrip": (1.0,)}, x=0.2), driver)
    assert stopped.reason == "controller_stopped"
    np.testing.assert_allclose(stopped.safe_joint_target_deg, driver.pose.joints_deg)
    assert np.linalg.norm(stopped.safe_joint_target_deg - moved.raw_joint_target_deg) > 1.0


def test_relative_twist_disabled_ignores_rotation_only_motion():
    session = make_relative_session({"wrist_rotation_enabled": False})
    driver = FakeJointDriver()
    session.step(sample({"A": True}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (0.0,)}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(0)), driver)
    result = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(15)), driver)
    assert result.action == "skipped"
    assert not driver.sent


def test_relative_wrist_rotation_can_optionally_require_trigger():
    session = make_relative_session({"wrist_rotation_enabled": True, "wrist_rotation_deadman": "rightTrig", "rotation_deadband_deg": 0.0})
    driver = FakeJointDriver()
    session.step(sample({"A": True}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (0.0,)}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(0)), driver)
    no_trigger = session.step(sample({"rightGrip": (1.0,), "rightTrig": (0.0,)}, transform=rotate_z(15)), driver)
    assert no_trigger.action == "skipped"
    with_trigger = session.step(sample({"rightGrip": (1.0,), "rightTrig": (1.0,)}, transform=rotate_z(30)), driver)
    assert with_trigger.action == "sent"
    assert abs(with_trigger.safe_joint_target_deg[5] - with_trigger.robot_home_joints_deg[5]) > 1e-6


def test_relative_wrist_rotation_default_needs_no_trigger():
    session = make_relative_session(
        {
            "wrist_rotation_enabled": True,
            "wrist_rotation_deadman": None,
            "rotation_deadband_deg": 0.0,
            "wrist_rotation_filter_alpha": 1.0,
            "relative_gain_matrix": [[0, 0, 0, 0, 0, 0]] * 5 + [[0, 0, 0, 0, 0, 0.6]],
        }
    )
    driver = FakeJointDriver()
    session.step(sample({"A": True}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (0.0,)}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(0)), driver)
    result = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(10)), driver)
    assert result.action == "sent"
    assert result.wrist_dq_deg[2] > 0.0
    assert abs(result.safe_joint_target_deg[5] - result.robot_home_joints_deg[5]) > 1e-6


def test_first_right_grip_clutch_frame_does_not_send_wrist_jump():
    session = make_relative_session({"wrist_rotation_enabled": True, "wrist_rotation_deadman": None, "rotation_deadband_deg": 0.0})
    driver = FakeJointDriver()
    session.step(sample({"A": True}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (0.0,)}, transform=rotate_z(45)), driver)
    armed = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(45)), driver)
    assert armed.reason == "armed_this_cycle"
    assert not driver.sent
    stopped = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(45)), driver)
    assert stopped.action == "skipped"


def test_stopping_controller_rotation_stops_wrist_target_updates():
    session = make_relative_session(
        {
            "wrist_rotation_enabled": True,
            "wrist_rotation_deadman": None,
            "rotation_deadband_deg": 0.0,
            "wrist_rotation_filter_alpha": 1.0,
            "settle_frames_on_stop": 2,
            "relative_gain_matrix": [[0, 0, 0, 0, 0, 0]] * 5 + [[0, 0, 0, 0, 0, 0.6]],
        }
    )
    driver = FakeJointDriver()
    session.step(sample({"A": True}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (0.0,)}, transform=rotate_z(0)), driver)
    session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(0)), driver)
    moved = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(10)), driver)
    stopped_1 = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(10)), driver)
    stopped_2 = session.step(sample({"rightGrip": (1.0,)}, transform=rotate_z(10)), driver)
    assert moved.action == "sent"
    assert stopped_1.action == "skipped"
    assert stopped_2.action == "skipped"
    np.testing.assert_allclose(stopped_2.safe_joint_target_deg, moved.safe_joint_target_deg)
