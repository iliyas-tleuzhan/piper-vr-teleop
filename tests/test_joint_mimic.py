import numpy as np

from piper_vr.human_arm_model import HumanArmState
from piper_vr.joint_limits import PIPER_JOINT_MAX_DEG, PIPER_JOINT_MIN_DEG
from piper_vr.joint_mimic import JointMimicConfig, human_arm_to_mimic_vector_deg, human_arm_to_piper_joints, mimic_vector_to_piper_joints


def test_human_arm_to_piper_joints_shape_and_limits():
    human = HumanArmState(
        shoulder_xyz_m=np.zeros(3),
        elbow_xyz_m=np.ones(3),
        wrist_xyz_m=np.ones(3) * 2,
        hand_rotation=np.eye(3),
        shoulder_angles_deg=np.array([200.0, 200.0, 50.0]),
        elbow_flex_deg=180.0,
        wrist_angles_deg=np.array([200.0, -200.0, 300.0]),
    )
    joints = human_arm_to_piper_joints(human, JointMimicConfig.from_config({}))
    assert joints.shape == (6,)
    assert np.all(joints >= PIPER_JOINT_MIN_DEG)
    assert np.all(joints <= PIPER_JOINT_MAX_DEG)


def test_human_arm_to_mimic_vector_deg():
    human = HumanArmState(
        shoulder_xyz_m=np.zeros(3),
        elbow_xyz_m=np.ones(3),
        wrist_xyz_m=np.ones(3) * 2,
        hand_rotation=np.eye(3),
        shoulder_angles_deg=np.array([10.0, 20.0, 30.0]),
        elbow_flex_deg=45.0,
        wrist_angles_deg=np.array([5.0, 6.0, 7.0]),
    )
    np.testing.assert_allclose(human_arm_to_mimic_vector_deg(human), [10.0, 20.0, 45.0, 35.0, 6.0, 7.0])


def test_calibration_relative_mapping_equals_robot_home_at_home():
    config = JointMimicConfig.from_config({})
    human_home = np.array([10.0, 20.0, 90.0, 0.0, 0.0, 0.0])
    robot_home = np.array([5.0, 80.0, -100.0, 1.0, 2.0, 3.0])
    target = mimic_vector_to_piper_joints(human_home, human_home, robot_home, config)
    np.testing.assert_allclose(target, robot_home)


def test_calibration_relative_mapping_applies_signs_gains_delta():
    config = JointMimicConfig.from_config({"signs": [1, -1, 1, 1, 1, 1], "gains": [1, 2, 1, 1, 1, 1]})
    human_home = np.zeros(6)
    human = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    robot_home = np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0])
    target = mimic_vector_to_piper_joints(human, human_home, robot_home, config)
    np.testing.assert_allclose(target, [1.0, 86.0, -87.0, 4.0, 5.0, 6.0])
