# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


from romea_joystick_description import (
    get_joystick_configuration,
    get_joystick_buttons_mapping,
    get_joystick_axes_mapping,
    joystick_remapping,
)


def test_get_buttons_mapping():
    configuration = get_joystick_configuration("dualshock4", "joy")
    mapping = get_joystick_buttons_mapping(configuration)
    assert mapping["Cross"] == 0


def test_get_axes_mapping():
    configuration = get_joystick_configuration("xbox", "joy")
    mapping = get_joystick_axes_mapping(configuration)
    assert mapping["Horizontal_Left_Stick"] == 0
    assert mapping["Horizontal_Directional_Pad"] == 6
    assert mapping["LT"] == 2


def test_remapping():

    user_remapping = {}
    user_remapping["axes"] = {}
    user_remapping["buttons"] = {}
    user_remapping["axes"]["foo"] = "Horizontal_Left_Stick"
    user_remapping["buttons"]["bar"] = "Cross"
    remapping = joystick_remapping("dualshock4", "ds4_driver", user_remapping)
    assert remapping["axes"]["foo"] == 0
    assert remapping["buttons"]["bar"] == 3
