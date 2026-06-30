import numpy as np
import yaml
from argparse import Namespace
from pathlib import Path

from piper_vr.config import apply_profile, deep_merge
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.piper_driver import JointPose
from piper_vr.session import SessionResult
from piper_vr.types import TeleopState
from piper_vr.vr_teleop import _apply_common_overrides, _print_motion_debug


def test_config_deep_merge_mapping_config():
    base = {"speed_percent": 5, "joint_mimic": {"smoothing_alpha": 0.25, "translation_deadband_m": 0.003}}
    patch = {"joint_mimic": {"smoothing_alpha": 0.65}}
    merged = deep_merge(base, patch)
    assert merged["speed_percent"] == 5
    assert merged["joint_mimic"]["translation_deadband_m"] == 0.003
    assert merged["joint_mimic"]["smoothing_alpha"] == 0.65


def test_profile_application_safe_normal_fast():
    config = {
        "profiles": {
            "safe": {"speed_percent": 10, "max_joint_speed_deg_s": [10] * 6},
            "normal": {"speed_percent": 50, "max_joint_speed_deg_s": [45] * 6},
            "fast": {"speed_percent": 100, "max_joint_speed_deg_s": [90] * 6},
        },
        "joint_mimic": {"max_joint_speed_deg_s": [1] * 6},
    }
    for name, speed, joint_speed in (("safe", 10, 10), ("normal", 50, 45), ("fast", 100, 90)):
        applied = apply_profile(config, name)
        assert applied["speed_percent"] == speed
        assert applied["joint_mimic"]["max_joint_speed_deg_s"] == [joint_speed] * 6
        assert applied["active_profile"] == name


def test_debug_motion_formatting_without_measured_joints(capsys):
    class Driver:
        def read_joint_pose(self):
            return None

    result = SessionResult(
        state=TeleopState.ACTIVE,
        calibrated=True,
        controller_xyz=np.array([1.0, 2.0, 3.0]),
        delta_xyz=np.array([0.0, -0.2, 0.01]),
        delta_rot_raw_deg=np.array([1.0, 2.0, 3.0]),
        delta_rot_deg=np.array([0.5, 1.0, 1.5]),
        relative_dq_deg=np.array([30.0, 20.0, -20.0, 0.0, 0.0, 0.0]),
        translation_dq_deg=np.array([30.0, 20.0, -20.0]),
        wrist_dq_deg=np.array([0.0, 0.0, 0.0]),
        raw_joint_target_deg=np.zeros(6),
        safe_joint_target_deg=np.ones(6),
        measured_joints=None,
    )
    _print_motion_debug(result, Driver())
    output = capsys.readouterr().out
    assert "[debug-motion]" in output
    assert "measured_joints=None" in output
    assert "dominant=dy negative" in output


def test_default_config_has_wrist_enabled_and_nonzero_rows():
    config = yaml.safe_load(Path("configs/single_piper.yaml").read_text(encoding="utf-8"))
    mimic = JointMimicConfig.from_config(config["joint_mimic"])
    assert mimic.wrist_rotation_enabled is True
    assert mimic.wrist_rotation_deadman is None
    assert np.count_nonzero(mimic.relative_gain_matrix[3:6, 3:6]) == 3


def test_wrist_rotation_deadman_null_parses_as_none():
    assert JointMimicConfig.from_config({"wrist_rotation_deadman": None}).wrist_rotation_deadman is None
    assert JointMimicConfig.from_config({"wrist_rotation_deadman": "None"}).wrist_rotation_deadman is None
    assert JointMimicConfig.from_config({"wrist_rotation_deadman": "null"}).wrist_rotation_deadman is None


def _args(**overrides):
    values = {
        "control_mode": None,
        "can": None,
        "hz": None,
        "speed_percent": None,
        "side": None,
        "deadman_button": None,
        "calibrate_button": None,
        "max_joint_speed": None,
        "viz": False,
        "viz_host": None,
        "viz_port": None,
        "disable_wrist": False,
        "wrist_gain": None,
        "translation_only": False,
        "rotation_only": False,
    }
    values.update(overrides)
    return Namespace(**values)


def test_disable_wrist_and_translation_only_zero_rotation_channels():
    config = {"joint_mimic": {"relative_gain_matrix": np.eye(6).tolist(), "wrist_rotation_enabled": True}}
    disabled = _apply_common_overrides(config, _args(disable_wrist=True))
    matrix = np.asarray(disabled["joint_mimic"]["relative_gain_matrix"])
    assert disabled["joint_mimic"]["wrist_rotation_enabled"] is False
    assert np.count_nonzero(matrix[3:6, :]) == 0
    assert np.count_nonzero(matrix[:, 3:6]) == 0

    config = {"joint_mimic": {"relative_gain_matrix": np.eye(6).tolist(), "wrist_rotation_enabled": True}}
    translation_only = _apply_common_overrides(config, _args(translation_only=True))
    matrix = np.asarray(translation_only["joint_mimic"]["relative_gain_matrix"])
    assert np.count_nonzero(matrix[3:6, :]) == 0
    assert np.count_nonzero(matrix[:, 3:6]) == 0


def test_rotation_only_zeroes_translation_channels():
    config = {"joint_mimic": {"relative_gain_matrix": np.eye(6).tolist(), "wrist_rotation_enabled": False}}
    rotation_only = _apply_common_overrides(config, _args(rotation_only=True))
    matrix = np.asarray(rotation_only["joint_mimic"]["relative_gain_matrix"])
    assert rotation_only["joint_mimic"]["wrist_rotation_enabled"] is True
    assert np.count_nonzero(matrix[0:3, :]) == 0
    assert np.count_nonzero(matrix[:, 0:3]) == 0
