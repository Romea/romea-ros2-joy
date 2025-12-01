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

from launch.substitutions import LaunchConfiguration
import romea_common_meta_bringup.ros_launch as common


def declare_joystick_model(default_value=None):
    return common.declare_argument(
        {
            "name": "joystick_model",
            "description": "The model of the joystick",
            "choices": ["chengong_hkj60ss", "microsoft_xbox", "pyinpu_teleop", "sony_dualshock4"]
        },
        default_value
    )


def get_joystick_model(context):
    return LaunchConfiguration("joystick_model").perform(context)


def declare_joystick_configuration_file_path(default_value=None):
    return common.declare_argument(
        {
            "name": "joystick_configuration_file_path",
            "description": "Path to the joystick configuration file.",
        },
        default_value
    )


def get_joystick_configuration_file_path(context):
    return LaunchConfiguration("joystick_configuration_file_path").perform(context)


def declare_joystick_topic(default_value=None):
    return common.declare_argument(
        {
            "name": "joystick_topic",
            "description": "The topic name for joystick messages.",
        },
        default_value
    )


def get_joystick_topic(context):
    return LaunchConfiguration("joystick_topic").perform(context)
