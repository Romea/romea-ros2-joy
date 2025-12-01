# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory
import yaml


def get_joystick_configuration_file_path(config_filename):
    pkg_path = get_package_share_directory('romea_joystick_utils')
    return f'{pkg_path}/config/{config_filename}.yaml'


def get_joystick_configuration(config_filename):
    with open(get_joystick_configuration_file_path(config_filename)) as f:
        return yaml.safe_load(f)


def get_joystick_buttons_mapping(joystick_configuration):
    return joystick_configuration["joy_msg_layout"]["buttons"]["mapping"]


def get_joystick_axes_mapping(joystick_configuration):

    axes_mapping = {}
    axes = joystick_configuration["joy_msg_layout"]["axes"]
    for axes_type in axes.keys():
        for mapping in axes[axes_type]["mapping"].items():
            axes_mapping[mapping[0]] = mapping[1]

    return axes_mapping


def apply_joystick_remapping(joystick_configuration, user_joystick_remapping):

    joystick_remapping = {}
    joystick_remapping["buttons"] = {}
    joystick_remapping["axes"] = {}

    try:
        buttons_mapping = get_joystick_buttons_mapping(joystick_configuration)
        for user_buttons_remapping in user_joystick_remapping["buttons"].items():
            button_id = buttons_mapping[user_buttons_remapping[1]]
            joystick_remapping["buttons"][user_buttons_remapping[0]] = button_id
    except Exception:
        raise LookupError(
            "Cannot define remapping for button "
            + user_buttons_remapping[1]
            + " because this button is not defined in joystick msg layout"
        )

    try:
        axes_mapping = get_joystick_axes_mapping(joystick_configuration)
        for user_axes_remapping in user_joystick_remapping["axes"].items():
            axe_id = axes_mapping[user_axes_remapping[1]]
            joystick_remapping["axes"][user_axes_remapping[0]] = axe_id
    except Exception:
        raise LookupError(
            "Cannot define remapping for axe "
            + user_axes_remapping[1]
            + " because this axe is not defined in joystick msg layout"
        )

    return joystick_remapping


def apply_joystick_buttons_remapping(joystick_configuration, user_buttons_remapping):

    buttons_remapping = {}

    try:
        buttons_mapping = get_joystick_buttons_mapping(joystick_configuration)
        for user_buttons_remapping in user_buttons_remapping.items():
            button_id = buttons_mapping[user_buttons_remapping[1]]
            buttons_remapping[user_buttons_remapping[0]] = button_id
    except Exception:
        raise LookupError(
            "Cannot define remapping for button "
            + user_buttons_remapping[1]
            + " because this button is not defined in joystick msg layout"
        )

    return buttons_remapping
