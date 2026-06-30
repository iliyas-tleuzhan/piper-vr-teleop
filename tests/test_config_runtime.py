import numpy as np

from piper_vr.config import apply_profile, deep_merge
from piper_vr.piper_driver import JointPose
from piper_vr.session import SessionResult
from piper_vr.types import TeleopState
from piper_vr.vr_teleop import _print_motion_debug


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
        relative_u=np.array([0.0, -0.2, 0.01, 0.0, 0.0, 0.0]),
        relative_dq_deg=np.array([30.0, 20.0, -20.0, 0.0, 0.0, 0.0]),
        raw_joint_target_deg=np.zeros(6),
        safe_joint_target_deg=np.ones(6),
        measured_joints=None,
    )
    _print_motion_debug(result, Driver())
    output = capsys.readouterr().out
    assert "[debug-motion]" in output
    assert "measured_joints=None" in output
    assert "dominant=dy negative" in output
