import numpy as np
import yaml
from argparse import Namespace
from pathlib import Path

from piper_vr.config import apply_profile, deep_merge
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.piper_driver import JointPose
from piper_vr.quest_endpoint_ik import EndpointIKResult
from piper_vr.session import SessionResult
from piper_vr.types import TeleopState
from piper_vr.vr_teleop import _apply_common_overrides, _print_ik_debug, _print_motion_debug, build_parser


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
        "endpoint_ik": False,
        "ik_backend": None,
        "ik_scale": None,
        "ik_orientation_scale": None,
        "disable_orientation": False,
        "position_only": False,
        "orientation_enabled": False,
        "full_workspace": False,
        "no_home_delta_clamp": False,
        "no_workspace_clamp": False,
        "home_delta_clamp": False,
        "workspace_clamp": False,
        "max_delta_from_home": None,
        "workspace_min": None,
        "workspace_max": None,
        "max_position_step_xyz": None,
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


def test_endpoint_full_workspace_cli_disables_target_clamps():
    config = {"quest_endpoint_ik": {"home_delta_clamp_enabled": True, "workspace_clamp_enabled": True}}
    updated = _apply_common_overrides(config, _args(full_workspace=True))
    assert updated["quest_endpoint_ik"]["home_delta_clamp_enabled"] is False
    assert updated["quest_endpoint_ik"]["workspace_clamp_enabled"] is False


def test_endpoint_cli_numeric_workspace_overrides():
    updated = _apply_common_overrides(
        {"quest_endpoint_ik": {}},
        _args(
            max_delta_from_home=[0.6, 0.5, 0.4],
            workspace_min=[-1.0, -0.9, -0.2],
            workspace_max=[1.0, 0.9, 0.8],
            max_position_step_xyz=[0.025, 0.035, 0.025],
        ),
    )
    endpoint = updated["quest_endpoint_ik"]
    assert endpoint["max_delta_from_home_m"] == [0.6, 0.5, 0.4]
    assert endpoint["workspace_min_m"] == [-1.0, -0.9, -0.2]
    assert endpoint["workspace_max_m"] == [1.0, 0.9, 0.8]
    assert endpoint["max_position_step_m_xyz"] == [0.025, 0.035, 0.025]


def test_endpoint_workspace_flags_parse():
    args = build_parser().parse_args(
        [
            "--endpoint-ik",
            "--full-workspace",
            "--max-delta-from-home",
            "0.6",
            "0.6",
            "0.5",
            "--workspace-min",
            "-1.0",
            "-1.0",
            "-0.2",
            "--workspace-max",
            "1.0",
            "1.0",
            "1.0",
            "--max-position-step-xyz",
            "0.025",
            "0.035",
            "0.025",
        ]
    )
    assert args.endpoint_ik is True
    assert args.full_workspace is True
    assert args.max_delta_from_home == [0.6, 0.6, 0.5]
    assert args.workspace_min == [-1.0, -1.0, -0.2]
    assert args.workspace_max == [1.0, 1.0, 1.0]
    assert args.max_position_step_xyz == [0.025, 0.035, 0.025]


def test_debug_ik_prints_limit_active_when_clamped(capsys):
    class Driver:
        def read_joint_pose(self):
            return None

    result = EndpointIKResult(
        state=TeleopState.ACTIVE,
        calibrated=True,
        home_delta_clamp_enabled=True,
        workspace_clamp_enabled=False,
        home_delta_clamped=True,
        workspace_clamped=False,
        clamped_axes=["x", "z"],
    )
    _print_ik_debug(result, Driver())
    output = capsys.readouterr().out
    assert "home_delta_clamp_enabled=True" in output
    assert "workspace_clamp_enabled=False" in output
    assert "LIMIT ACTIVE: target_clamped_home_delta axes=['x', 'z']" in output
