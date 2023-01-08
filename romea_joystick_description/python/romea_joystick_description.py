# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import yaml
from ament_index_python.packages import get_package_share_directory


def get_joystick_configuration_filename(joystick_type, joystick_driver):
    return (
        get_package_share_directory("romea_joystick_description")
        + "/config/"
        + joystick_type
        + "_"
        + joystick_driver
        + ".yaml"
    )


def get_joystick_configuration(joystick_type, joystick_driver):
    with open(get_joystick_configuration_filename(joystick_type, joystick_driver)) as f:
        return yaml.safe_load(f)


def get_joystick_buttons_mapping(joystick_configuration):
    return joystick_configuration["buttons"]["mapping"]


def get_joystick_axes_mapping(joystick_configuration):

    axes_mapping = {}
    for axes_type in joystick_configuration["axes"].keys():
        for mapping in joystick_configuration["axes"][axes_type]["mapping"].items():
            axes_mapping[mapping[0]] = mapping[1]

    return axes_mapping


def joystick_remapping(joystick_type, joystick_driver, user_joystick_remapping):

    joystick_remapping = {}
    joystick_remapping["buttons"] = {}
    joystick_remapping["axes"] = {}

    joystick_configuration = get_joystick_configuration(joystick_type, joystick_driver)
    buttons_mapping = get_joystick_buttons_mapping(joystick_configuration)
    for user_buttons_remapping in user_joystick_remapping["buttons"].items():
        id = buttons_mapping[user_buttons_remapping[1]]
        joystick_remapping["buttons"][user_buttons_remapping[0]] = id

    axes_mapping = get_joystick_axes_mapping(joystick_configuration)
    for user_axes_remapping in user_joystick_remapping["axes"].items():
        id = axes_mapping[user_axes_remapping[1]]
        joystick_remapping["axes"][user_axes_remapping[0]] = id

    return joystick_remapping
