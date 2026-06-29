import numpy as np

from piper_vr.vr_mapping import AxisMapping, target_from_home


def test_axis_mapping_controller_home():
    home = np.eye(4)
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.2, -0.3]
    target = target_from_home(home, current, np.array([0.4, 0.0, 0.2]), AxisMapping(), 1.0)
    np.testing.assert_allclose(target, [0.7, -0.1, 0.4])
