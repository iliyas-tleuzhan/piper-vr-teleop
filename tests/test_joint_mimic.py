import numpy as np

from piper_vr.human_arm_model import HumanArmState
from piper_vr.joint_limits import PIPER_JOINT_MAX_DEG, PIPER_JOINT_MIN_DEG
from piper_vr.joint_mimic import JointMimicConfig, human_arm_to_piper_joints


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
