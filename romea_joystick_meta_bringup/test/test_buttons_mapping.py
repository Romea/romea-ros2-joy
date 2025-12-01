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

import os
import subprocess

from ament_index_python import get_package_prefix
import yaml


def test_buttons_mapping():

    exe = (
        get_package_prefix("romea_joystick_meta_bringup")
        + "/lib/romea_joystick_meta_bringup/generate_buttons_mapping_file.py"
    )

    meta_description_file_path = os.path.join(os.getcwd(), "test_joystick_meta_bringup.yaml")
    teleop_remapping_file_path = os.path.join(os.getcwd(), "test_buttons_mapping.yaml")

    remapping = yaml.safe_load(
        subprocess.check_output(
            [
                exe,
                "meta_description_file_path:" + meta_description_file_path,
                "teleop_remapping_file_path:" + teleop_remapping_file_path,
            ],
            encoding="utf-8",
        )
    )

    assert remapping["start"] == 2
    assert remapping["stop"] == 1
