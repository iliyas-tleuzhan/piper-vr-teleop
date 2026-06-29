import numpy as np

from piper_vr.safety import SafetyLimiter, tracking_is_stale


def test_workspace_clamp_and_reason():
    limiter = SafetyLimiter(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]), 1.0, max_position_jump_m=0.5)
    value, reason = limiter.clamp_workspace_with_reason(np.array([2.0, 0.5, -1.0]))
    np.testing.assert_allclose(value, [1.0, 0.5, 0.0])
    assert reason == "workspace_clamped"


def test_speed_limit_reason():
    limiter = SafetyLimiter(np.zeros(3), np.ones(3) * 10.0, 0.1, max_position_jump_m=1.0)
    limiter.reset(np.zeros(3), now_s=0.0)
    value, reason = limiter.limit_step_with_reason(np.array([1.0, 0.0, 0.0]), now_s=0.1)
    assert value[0] < 1.0
    assert reason in ("speed_limited", "max_position_jump_limited")


def test_tracking_stale_false_for_recent(monkeypatch):
    monkeypatch.setattr("piper_vr.safety.time.monotonic", lambda: 10.0)
    assert tracking_is_stale(9.9, 0.25) is False
    assert tracking_is_stale(9.0, 0.25) is True
