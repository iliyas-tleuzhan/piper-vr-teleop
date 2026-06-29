import numpy as np

from piper_vr.transports.adb_logcat import parse_oculus_payload


def test_parse_oculus_payload():
    matrix = " ".join(str(v) for v in range(16))
    transforms, buttons = parse_oculus_payload(f"r:{matrix}|l:{matrix}&A:True|rightGrip:0.75|rightJS:0.1 0.2")
    assert set(transforms) == {"left", "right"}
    assert transforms["right"].shape == (4, 4)
    np.testing.assert_allclose(transforms["right"].reshape(-1), np.arange(16))
    assert buttons["A"] is True
    assert buttons["rightGrip"] == 0.75
    assert buttons["rightJS"] == (0.1, 0.2)
