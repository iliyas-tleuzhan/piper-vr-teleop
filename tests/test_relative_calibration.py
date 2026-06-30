import numpy as np

from piper_vr.relative_calibration import build_observation, dominant_channel, generated_mapping_config


def synthetic_calibration():
    return {
        "movements": [
            build_observation("up", [0.004, 0.183, -0.012]),
            build_observation("down", [-0.004, -0.181, 0.011]),
            build_observation("left", [0.146, -0.008, -0.002]),
            build_observation("right", [-0.146, 0.008, 0.002]),
            build_observation("forward", [0.003, 0.006, 0.152]),
            build_observation("backward", [-0.003, -0.006, -0.151]),
        ]
    }


def test_dominant_channel_and_sign_detection():
    index, channel, sign, value = dominant_channel(np.array([0.004, 0.183, -0.012]))
    assert index == 1
    assert channel == "dy"
    assert sign == "positive"
    assert value == 0.183


def test_guided_axis_calibration_json_schema_observation():
    row = build_observation("right", [-0.146, 0.008, 0.002])
    assert row["movement"] == "right"
    assert row["delta_xyz"] == [-0.146, 0.008, 0.002]
    assert row["dominant_channel"] == "dx"
    assert row["dominant_index"] == 0
    assert row["sign"] == "negative"


def test_gain_generation_left_right_sign_flip():
    matrix = np.asarray(generated_mapping_config(synthetic_calibration())["joint_mimic"]["relative_gain_matrix"])
    assert matrix[0, 0] == -300.0
    assert np.count_nonzero(matrix[3:, :]) == 0


def test_gain_generation_up_down_sign_flip():
    matrix = np.asarray(generated_mapping_config(synthetic_calibration())["joint_mimic"]["relative_gain_matrix"])
    assert matrix[1, 1] == 275.0
    assert matrix[2, 1] == -275.0


def test_gain_generation_forward_backward_nonzero():
    matrix = np.asarray(generated_mapping_config(synthetic_calibration())["joint_mimic"]["relative_gain_matrix"])
    assert matrix[1, 2] == 250.0
    assert matrix[2, 2] == 250.0
