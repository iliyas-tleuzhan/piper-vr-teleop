from types import SimpleNamespace

import numpy as np

from piper_vr.piper_driver import JointPose
from piper_vr.types import TeleopState
from piper_vr.viz_broadcaster import QuestVizBroadcaster


def result(**overrides):
    base = {
        "state": TeleopState.ACTIVE,
        "calibrated": True,
        "deadman": True,
        "controller_xyz": np.array([0.1, 0.2, 0.3]),
        "safe_joint_target_deg": np.array([0, 90, -90, 0, 0, 0], dtype=float),
        "raw_joint_target_deg": None,
        "measured_joints": JointPose(np.array([1, 89, -91, 0, 0, 0], dtype=float)),
        "sample_age_s": 0.01,
        "action": "sent",
        "reason": "ok",
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def test_packet_format_contains_visualization_contract():
    broadcaster = QuestVizBroadcaster(enabled=False)
    packet = broadcaster.build_packet(result(), mode="joint_mimic", mapping_mode="relative_delta")

    assert packet["type"] == "piper_joint_state"
    assert packet["state"] == "ACTIVE"
    assert packet["mode"] == "joint_mimic"
    assert packet["mapping_mode"] == "relative_delta"
    assert packet["commanded_joints_deg"] == [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    assert packet["measured_joints_deg"] == [1.0, 89.0, -91.0, 0.0, 0.0, 0.0]
    assert packet["controller_xyz"] == [0.1, 0.2, 0.3]


def test_udp_send_does_not_crash_without_receiver():
    broadcaster = QuestVizBroadcaster(host="127.0.0.1", port=5055, enabled=True)
    try:
        assert broadcaster.send(result(), mode="joint_mimic", mapping_mode="relative_delta") is True
    finally:
        broadcaster.close()


def test_missing_fields_serialize_as_nulls():
    broadcaster = QuestVizBroadcaster(enabled=False)
    packet = broadcaster.build_packet(
        result(
            controller_xyz=None,
            safe_joint_target_deg=None,
            raw_joint_target_deg=None,
            measured_joints=None,
            sample_age_s=None,
        )
    )

    assert packet["commanded_joints_deg"] is None
    assert packet["measured_joints_deg"] is None
    assert packet["controller_xyz"] is None
    assert packet["sample_age_s"] is None


def test_joint_arrays_must_be_six_values_or_null():
    broadcaster = QuestVizBroadcaster(enabled=False)
    packet = broadcaster.build_packet(
        result(
            safe_joint_target_deg=np.array([1, 2, 3]),
            measured_joints=JointPose(np.array([1, 2, 3, 4, 5, 6, 7], dtype=float)),
        )
    )

    assert packet["commanded_joints_deg"] is None
    assert packet["measured_joints_deg"] is None
