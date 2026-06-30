import numpy as np

from piper_vr.joint_limits import clamp_joints_deg, degrees_to_piper_joint_units, piper_joint_units_to_degrees, rate_limit_joints_deg


def test_joint_unit_roundtrip():
    assert degrees_to_piper_joint_units(12.345) == 12345
    assert piper_joint_units_to_degrees(-90000) == -90.0


def test_clamp_joints_to_piper_limits():
    q = clamp_joints_deg(np.array([-200, -10, 10, 120, -90, 200], dtype=float))
    assert q.tolist() == [-150.0, 0.0, 0.0, 100.0, -70.0, 120.0]


def test_rate_limit_joints_independently():
    q = rate_limit_joints_deg([10, 10, -80, 30, 30, 30], [0, 0, -90, 0, 0, 0], [5, 10, 20, 30, 40, 50], 0.5)
    np.testing.assert_allclose(q, [2.5, 5.0, -80.0, 15.0, 20.0, 25.0])
