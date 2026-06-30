import subprocess
import sys
import os

import numpy as np
import pytest

from piper_vr.config import deep_merge
from piper_vr.piper_official_kinematics import PiperOfficialDHForwardKinematics
from piper_vr.piper_driver import EndPose, JointPose
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
from piper_vr.units import degrees_to_piper_rpy, meters_to_piper_xyz


class FakeJointDriver:
    def __init__(self, pose=True, dry_run=False):
        self.pose = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
        self.feedback_available = pose
        self.dry_run = dry_run
        self.has_sent_joint_command = False
        self.sent = []
        self.end_pose = EndPose(np.array([0.35, 0.0, 0.25]), np.zeros(3))

    def read_joint_pose(self):
        return self.pose if self.feedback_available else None

    def read_end_pose(self):
        return self.end_pose

    def send_joint_pose(self, joints_deg):
        self.pose = JointPose(np.asarray(joints_deg, dtype=float))
        self.sent.append(self.pose)
        self.has_sent_joint_command = True

    def hold_joints(self, allow_last_command_fallback=False):
        self.send_joint_pose(self.pose.joints_deg)

    def hold(self):
        return None


def transform(xyz=(0.0, 0.0, 0.0), yaw_deg=0.0):
    angle = np.radians(yaw_deg)
    c, s = np.cos(angle), np.sin(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    matrix[:3, 3] = xyz
    return matrix


def yaw_frame(deg):
    angle = np.radians(deg)
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


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
        "max_delta_from_home_m": [1.0, 1.0, 1.0],
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


def test_hmd_yaw_control_frame_stable_mapping():
    cfg = config(axis_mapping={"robot_x": "+quest_z", "robot_y": "-quest_x", "robot_z": "+quest_y"})
    _, _, debug = endpoint_target_from_controller(
        transform(),
        transform((1.0, 0.0, 0.0)),
        np.array([0.3, 0.0, 0.2]),
        np.zeros(3),
        cfg,
        control_frame=yaw_frame(90),
    )
    np.testing.assert_allclose(debug["controller_delta_xyz"], [0.0, 0.0, 1.0], atol=1e-6)


def test_forward_physical_movement_positive_robot_x_default():
    cfg = QuestEndpointIKConfig.from_config({"axis_mapping": {"robot_x": "+quest_z", "robot_y": "-quest_x", "robot_z": "+quest_y"}, "scale_xyz": [1, 1, 1]})
    _, _, debug = endpoint_target_from_controller(transform(), transform((0.0, 0.0, 0.2)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    assert debug["mapped_robot_delta_xyz"][0] > 0


def test_axis_mapping_signs_and_rotation_mapping():
    assert EndpointAxisMapping(robot_x="+quest_x").apply(np.array([2.0, 3.0, 4.0]))[0] == 2.0
    assert EndpointAxisMapping(robot_x="-quest_x").apply(np.array([2.0, 3.0, 4.0]))[0] == -2.0
    assert EndpointRotationMapping(robot_yaw="-quest_yaw").apply(np.array([1.0, 2.0, 3.0]))[2] == -3.0


def test_firmware_endpoint_unit_conversion():
    assert meters_to_piper_xyz(0.123) == 123000
    assert degrees_to_piper_rpy(12.5) == 12500


def test_workspace_and_orientation_clamp():
    cfg = config(workspace_min_m=[0.2, -0.2, 0.1], workspace_max_m=[0.5, 0.2, 0.4], max_orientation_delta_deg=[10, 20, 30])
    np.testing.assert_allclose(clamp_workspace(np.array([1.0, -1.0, 0.0]), cfg), [0.5, -0.2, 0.1])
    np.testing.assert_allclose(clamp_orientation_delta(np.array([30.0, -30.0, 40.0]), cfg), [10.0, -20.0, 30.0])


def test_backend_selection_and_position_only_default():
    cfg = QuestEndpointIKConfig.from_config({})
    assert cfg.backend == "firmware_endpoint"
    assert cfg.orientation_enabled is False
    assert QuestEndpointIKConfig.from_config({"backend": "host_ik_sdk_fk"}).backend == "host_ik_sdk_fk"


def test_official_dh_fk_sanity_units_are_meters():
    fk = PiperOfficialDHForwardKinematics()
    xyz, rotation = fk.forward(np.radians([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))
    assert xyz.shape == (3,)
    assert rotation.shape == (3, 3)
    assert 0.05 < np.linalg.norm(xyz) < 1.0


def test_home_relative_clamp():
    cfg = config(max_delta_from_home_m=[0.1, 0.1, 0.1], axis_mapping={"robot_x": "+quest_x", "robot_y": "+quest_y", "robot_z": "+quest_z"})
    target_xyz, _, debug = endpoint_target_from_controller(transform(), transform((0.5, 0.0, 0.0)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    np.testing.assert_allclose(target_xyz, [0.4, 0.0, 0.2])
    assert debug["home_delta_clamped"] is True


def test_scale_xyz_affects_only_requested_axis_and_y_is_larger():
    cfg = config(scale=0.5, scale_xyz=[0.6, 1.5, 0.5], axis_mapping={"robot_x": "+quest_x", "robot_y": "+quest_y", "robot_z": "+quest_z"})
    _, _, x_debug = endpoint_target_from_controller(transform(), transform((0.1, 0.0, 0.0)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    _, _, y_debug = endpoint_target_from_controller(transform(), transform((0.0, 0.1, 0.0)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    _, _, z_debug = endpoint_target_from_controller(transform(), transform((0.0, 0.0, 0.1)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    np.testing.assert_allclose(x_debug["scaled_robot_delta_xyz"], [0.03, 0.0, 0.0])
    np.testing.assert_allclose(y_debug["scaled_robot_delta_xyz"], [0.0, 0.075, 0.0])
    np.testing.assert_allclose(z_debug["scaled_robot_delta_xyz"], [0.0, 0.0, 0.025])
    assert y_debug["scaled_robot_delta_xyz"][1] > x_debug["scaled_robot_delta_xyz"][0]


def test_endpoint_target_debug_contains_clamp_stages_and_axes():
    cfg = config(max_delta_from_home_m=[0.05, 0.05, 0.05], workspace_max_m=[0.34, 0.04, 0.24], axis_mapping={"robot_x": "+quest_x", "robot_y": "+quest_y", "robot_z": "+quest_z"})
    _, _, debug = endpoint_target_from_controller(transform(), transform((0.2, 0.2, 0.2)), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    assert "target_before_home_clamp" in debug
    assert "target_after_home_clamp" in debug
    assert "target_after_workspace_clamp" in debug
    assert debug["clamped_axes"] == ["x", "y", "z"]


def test_max_position_step_m_xyz_allows_y_faster_than_xz():
    session = QuestEndpointIKSession(
        side="right",
        deadman_button="rightGrip",
        calibrate_button="A",
        config=config(max_position_step_m_xyz=[0.01, 0.03, 0.01], axis_mapping={"robot_x": "+quest_x", "robot_y": "+quest_y", "robot_z": "+quest_z"}),
        stale_timeout_s=0.25,
    )
    session.filtered_xyz = np.array([0.3, 0.0, 0.2])
    target = np.array([0.5, 0.2, 0.4])
    step = np.clip(target - session.filtered_xyz, -session.config.max_position_step_m_xyz, session.config.max_position_step_m_xyz)
    np.testing.assert_allclose(step, [0.01, 0.03, 0.01])


def test_position_only_mode_ignores_orientation_delta():
    cfg = config(position_only_default=True, orientation_enabled=False)
    _, target_rpy, debug = endpoint_target_from_controller(transform(), transform(yaw_deg=45), np.array([0.3, 0.0, 0.2]), np.array([1.0, 2.0, 3.0]), cfg)
    np.testing.assert_allclose(target_rpy, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(debug["mapped_robot_delta_rpy_deg"], [0.0, 0.0, 0.0])


def test_orientation_opt_in_maps_rotation():
    cfg = config(position_only_default=False, orientation_enabled=True)
    _, target_rpy, _ = endpoint_target_from_controller(transform(), transform(yaw_deg=10), np.array([0.3, 0.0, 0.2]), np.zeros(3), cfg)
    assert abs(target_rpy[2]) > 1.0


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
    with pytest.raises(RuntimeError, match="git submodule update --init --recursive"):
        QuestEndpointIKSession(
            side="right",
            deadman_button="rightGrip",
            calibrate_button="A",
            config=config(backend="host_ik_urdf", urdf_path="missing.urdf"),
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


def test_verify_endpoint_directions_script_help_works():
    env = dict(os.environ)
    env["PYTHONPATH"] = os.getcwd() + os.pathsep + env.get("PYTHONPATH", "")
    completed = subprocess.run(
        [sys.executable, "scripts/verify_endpoint_directions.py", "--help"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    assert "Verify endpoint IK direction signs" in completed.stdout


def test_generated_endpoint_mapping_config_deep_merges():
    base = {"quest_endpoint_ik": {"scale": 1.0, "axis_mapping": {"robot_x": "-quest_z", "robot_y": "-quest_x"}}}
    patch = {"quest_endpoint_ik": {"axis_mapping": {"robot_x": "+quest_y"}, "rotation_mapping": {"robot_yaw": "-quest_yaw"}}}
    merged = deep_merge(base, patch)
    assert merged["quest_endpoint_ik"]["scale"] == 1.0
    assert merged["quest_endpoint_ik"]["axis_mapping"]["robot_x"] == "+quest_y"
    assert merged["quest_endpoint_ik"]["axis_mapping"]["robot_y"] == "-quest_x"
    assert merged["quest_endpoint_ik"]["rotation_mapping"]["robot_yaw"] == "-quest_yaw"


def test_official_sdk_fk_compare_if_available():
    try:
        from piper_vr.piper_official_kinematics import PiperSDKForwardKinematics

        sdk_fk = PiperSDKForwardKinematics()
    except RuntimeError:
        pytest.skip("piper_sdk official FK is not installed")
    local_fk = PiperOfficialDHForwardKinematics()
    joints = np.radians([0.0, 90.0, -90.0, 0.0, 0.0, 0.0])
    sdk_xyz, _ = sdk_fk.forward(joints)
    local_xyz, _ = local_fk.forward(joints)
    assert np.linalg.norm(sdk_xyz - local_xyz) < 0.20
