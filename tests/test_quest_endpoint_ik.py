import subprocess
import sys
import os

import numpy as np
import pytest

from piper_vr.config import deep_merge
from piper_vr.piper_driver import JointPose
from piper_vr.quest_endpoint_ik import (
    EndpointAxisMapping,
    EndpointRotationMapping,
    QuestEndpointIKConfig,
    QuestEndpointIKSession,
    clamp_orientation_delta,
    clamp_workspace,
    endpoint_target_from_controller,
)
from piper_vr.types import QuestSample, TeleopState


class FakeJointDriver:
    def __init__(self, pose=True, dry_run=False):
        self.pose = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
        self.feedback_available = pose
        self.dry_run = dry_run
        self.has_sent_joint_command = False
        self.sent = []

    def read_joint_pose(self):
        return self.pose if self.feedback_available else None

    def send_joint_pose(self, joints_deg):
        self.pose = JointPose(np.asarray(joints_deg, dtype=float))
        self.sent.append(self.pose)
        self.has_sent_joint_command = True

    def hold_joints(self, allow_last_command_fallback=False):
        self.send_joint_pose(self.pose.joints_deg)


def transform(xyz=(0.0, 0.0, 0.0), yaw_deg=0.0):
    angle = np.radians(yaw_deg)
    c, s = np.cos(angle), np.sin(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    matrix[:3, 3] = xyz
    return matrix


def sample(buttons=None, age=0.0, pose=None):
    return QuestSample(0.0, "test", {"right": transform() if pose is None else pose}, buttons or {}, age)


def config(**overrides):
    base = {
        "urdf_path": "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf",
        "scale": 1.0,
        "orientation_scale": 1.0,
        "workspace_min_m": [0.0, -1.0, 0.0],
        "workspace_max_m": [1.0, 1.0, 1.0],
        "position_filter_alpha": 1.0,
        "orientation_filter_alpha": 1.0,
        "position_deadband_m": 0.0,
        "orientation_deadband_deg": 0.0,
        "max_position_step_m": 1.0,
        "max_orientation_step_deg": 180.0,
        "max_joint_speed_deg_s": [1000] * 6,
    }
    base.update(overrides)
    return QuestEndpointIKConfig.from_config(base)


def test_controller_relative_transform_to_robot_delta_mapping():
    cfg = config(axis_mapping={"robot_x": "-quest_z", "robot_y": "-quest_x", "robot_z": "+quest_y"})
    target_xyz, _, debug = endpoint_target_from_controller(
        transform(),
        transform((0.1, 0.2, -0.3)),
        np.array([0.4, 0.0, 0.2]),
        np.zeros(3),
        cfg,
    )
    np.testing.assert_allclose(debug["controller_delta_xyz"], [0.1, 0.2, -0.3])
    np.testing.assert_allclose(debug["mapped_robot_delta_xyz"], [0.3, -0.1, 0.2])
    np.testing.assert_allclose(target_xyz, [0.7, -0.1, 0.4])


def test_axis_mapping_signs_and_rotation_mapping():
    assert EndpointAxisMapping(robot_x="+quest_x").apply(np.array([2.0, 3.0, 4.0]))[0] == 2.0
    assert EndpointAxisMapping(robot_x="-quest_x").apply(np.array([2.0, 3.0, 4.0]))[0] == -2.0
    assert EndpointRotationMapping(robot_yaw="-quest_yaw").apply(np.array([1.0, 2.0, 3.0]))[2] == -3.0


def test_workspace_and_orientation_clamp():
    cfg = config(workspace_min_m=[0.2, -0.2, 0.1], workspace_max_m=[0.5, 0.2, 0.4], max_orientation_delta_deg=[10, 20, 30])
    np.testing.assert_allclose(clamp_workspace(np.array([1.0, -1.0, 0.0]), cfg), [0.5, -0.2, 0.1])
    np.testing.assert_allclose(clamp_orientation_delta(np.array([30.0, -30.0, 40.0]), cfg), [10.0, -20.0, 30.0])


def test_endpoint_ik_no_movement_before_calibration_and_deadman_required():
    session = QuestEndpointIKSession(side="right", deadman_button="rightGrip", calibrate_button="A", config=config(), stale_timeout_s=0.25)
    driver = FakeJointDriver()
    assert session.step(sample({"rightGrip": (1.0,)}), driver).reason == "not_calibrated"
    session.step(sample({"A": True}), driver)
    released = session.step(sample({"rightGrip": (0.0,)}), driver)
    assert released.reason == "deadman_released"
    assert not driver.sent


def test_endpoint_ik_right_grip_clutch_prevents_jump():
    session = QuestEndpointIKSession(side="right", deadman_button="rightGrip", calibrate_button="A", config=config(), stale_timeout_s=0.25)
    driver = FakeJointDriver()
    session.step(sample({"A": True}, pose=transform((0.0, 0.0, 0.0))), driver)
    session.step(sample({"rightGrip": (0.0,)}, pose=transform((0.5, 0.0, 0.0))), driver)
    armed = session.step(sample({"rightGrip": (1.0,)}, pose=transform((0.5, 0.0, 0.0))), driver)
    assert armed.reason == "armed_this_cycle"
    assert not driver.sent


def test_ik_unavailable_gives_clear_error():
    with pytest.raises(RuntimeError, match="Quest endpoint IK solver unavailable"):
        QuestEndpointIKSession(
            side="right",
            deadman_button="rightGrip",
            calibrate_button="A",
            config=config(urdf_path="missing.urdf"),
            stale_timeout_s=0.25,
        )


def test_predict_endpoint_ik_script_help_works():
    env = dict(os.environ)
    env["PYTHONPATH"] = os.getcwd() + os.pathsep + env.get("PYTHONPATH", "")
    completed = subprocess.run(
        [sys.executable, "scripts/predict_endpoint_ik_from_controller.py", "--help"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    assert "Predict endpoint IK targets" in completed.stdout


def test_generated_endpoint_mapping_config_deep_merges():
    base = {"quest_endpoint_ik": {"scale": 1.0, "axis_mapping": {"robot_x": "-quest_z", "robot_y": "-quest_x"}}}
    patch = {"quest_endpoint_ik": {"axis_mapping": {"robot_x": "+quest_y"}, "rotation_mapping": {"robot_yaw": "-quest_yaw"}}}
    merged = deep_merge(base, patch)
    assert merged["quest_endpoint_ik"]["scale"] == 1.0
    assert merged["quest_endpoint_ik"]["axis_mapping"]["robot_x"] == "+quest_y"
    assert merged["quest_endpoint_ik"]["axis_mapping"]["robot_y"] == "-quest_x"
    assert merged["quest_endpoint_ik"]["rotation_mapping"]["robot_yaw"] == "-quest_yaw"
