from piper_vr.buttons import analog_value, is_pressed, normalize_buttons


def test_button_helpers_boolean_and_analog():
    buttons = normalize_buttons({"A": True, "rightGrip": (0.7,), "leftTrig": (0.2,)})
    assert is_pressed(buttons, "A")
    assert is_pressed(buttons, "rightGrip")
    assert not is_pressed(buttons, "leftTrig")
    assert analog_value(buttons, "rightGrip") == 0.7
