import numpy as np

from piper_vr.human_arm_model import HumanArmConfig, build_human_arm_state, solve_elbow_position


def config():
    return HumanArmConfig.from_config({})


def test_elbow_inference_reachable():
    elbow = solve_elbow_position(np.zeros(3), np.array([0.4, 0.0, 0.0]), 0.3, 0.27, 0.0)
    assert elbow.shape == (3,)
    assert np.isfinite(elbow).all()


def test_elbow_inference_too_far_and_too_close_are_stable():
    far = solve_elbow_position(np.zeros(3), np.array([2.0, 0.0, 0.0]), 0.3, 0.27, -0.5)
    close = solve_elbow_position(np.zeros(3), np.array([0.001, 0.0, 0.0]), 0.3, 0.27, -0.5)
    assert np.isfinite(far).all()
    assert np.isfinite(close).all()


def test_build_human_arm_state_fields():
    transform = np.eye(4)
    transform[:3, 3] = [0.4, 0.0, 0.0]
    human = build_human_arm_state(np.zeros(3), transform, -0.5, config())
    assert human.shoulder_angles_deg.shape == (3,)
    assert human.wrist_angles_deg.shape == (3,)
    assert 0.0 <= human.elbow_flex_deg <= 180.0
