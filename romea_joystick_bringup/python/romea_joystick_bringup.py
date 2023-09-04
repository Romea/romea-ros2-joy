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


from romea_common_bringup import MetaDescription


class JoystickMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription(
            "joystick", meta_description_file_path)

    def get_name(self):
        return self.meta_description.get("name")

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_device(self):
        return self.meta_description.get("device", "driver")

    def get_driver_autorepeat_rate(self):
        return self.meta_description.get("autorepeat_rate", "driver")

    def get_driver_dead_zone(self):
        return self.meta_description.get("deadzone", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    # def get_model(self):
    #     return self.meta_description.get("model", "configuration")


def load_meta_description(meta_description_file_path):
    return JoystickMetaDescription(meta_description_file_path)
