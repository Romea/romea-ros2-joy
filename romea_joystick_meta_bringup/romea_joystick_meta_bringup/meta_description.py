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

from romea_common_meta_bringup.meta_description import SensorMetaDescription
from romea_common_meta_bringup.ros_launch import LaunchFileGenerator
import romea_joystick_utils
import yaml


class JoystickMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("joystick", meta_description_file_path, robot_name)

    def get_rate(self):
        return self._get_or("rate", "configuration", None)

    def get_msg_layout(self):
        return self._get_or(
            "msg_layout", "configuration", f"{self.get_manufacturer()}_{self.get_model()}"
        )


def load_meta_description(meta_description_file_path,  robot_name=None):
    return JoystickMetaDescription(meta_description_file_path, robot_name)


def get_complete_configuration(meta_description):
    return romea_joystick_utils.get_joystick_configuration(meta_description.get_msg_layout())


def generate_buttons_mapping_file(meta_description, user_remapping):
    mapping = romea_joystick_utils.apply_joystick_buttons_remapping(
        get_complete_configuration(meta_description), user_remapping)
    return yaml.dump(mapping, default_flow_style=False)


def generate_joystick_mapping_file(meta_description, user_remapping):
    mapping = romea_joystick_utils.apply_joystick_remapping(
        get_complete_configuration(meta_description), user_remapping)
    return yaml.dump(mapping, default_flow_style=False)


def generate_launch_file(meta_description):

    launch_arguments = [{"name": "mode", "default": "live"}]
    namespaces = [meta_description.get_robot_name(), meta_description.get_name()]

    configuration = {
        "frame_id": meta_description.get_link(),
        "joy_msg_layout":  meta_description.get_msg_layout(),
        "rate": meta_description.get_rate()
    }

    return LaunchFileGenerator("joystick").generate(
        meta_description.get_launch_file(), launch_arguments, namespaces, configuration
    )
