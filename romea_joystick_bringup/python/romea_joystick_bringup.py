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

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_package(self):
        return self.meta_description.get("package", "driver")

    def get_driver_executable(self):
        return self.meta_description.get("executable", "driver")

    def get_driver_parameters(self):
        return self.meta_description.get("parameters", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    # def get_model(self):
    #     return self.meta_description.get("model", "configuration")

    def get_records(self):
        return self.meta_description.get_or("records", None, {})

    def get_bridge(self):
        return self.meta_description.get_or("bridge", None, {})


def load_meta_description(meta_description_file_path):
    return JoystickMetaDescription(meta_description_file_path)
