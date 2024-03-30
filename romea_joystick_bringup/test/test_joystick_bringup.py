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
import pytest

from romea_joystick_bringup import JoystickMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_joystick_bringup.yaml")
    return JoystickMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "joystick"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_package(meta_description):
    assert meta_description.get_driver_package() == "joy"


def test_get_driver_executable(meta_description):
    assert meta_description.get_driver_executable() == "joy_node"


def test_get_driver_parameters(meta_description):
    parameters = meta_description.get_driver_parameters()
    assert parameters["device_name"] == "/dev/input/js0"
    assert parameters["autorepeat_rate"] == 10
    assert parameters["deadzone"] == 0.1


def test_get_type(meta_description):
    assert meta_description.get_type() == "xbox"


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["joy"] is True
