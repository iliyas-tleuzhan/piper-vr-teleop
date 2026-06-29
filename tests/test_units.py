from piper_vr.units import degrees_to_piper_rpy, meters_to_piper_xyz, piper_rpy_to_degrees, piper_xyz_to_meters


def test_piper_unit_conversions_round_trip():
    assert meters_to_piper_xyz(0.123456) == 123456
    assert piper_xyz_to_meters(123456) == 0.123456
    assert degrees_to_piper_rpy(12.345) == 12345
    assert piper_rpy_to_degrees(12345) == 12.345
