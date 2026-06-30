import numpy as np

from piper_vr.frame_calibration import controller_delta_in_control_frame


def test_controller_delta_direction_in_control_frame():
    previous = np.eye(4)
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.2, 0.3]
    translation, rotation = controller_delta_in_control_frame(previous, current, np.eye(3))
    np.testing.assert_allclose(translation, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(rotation, [0.0, 0.0, 0.0])
