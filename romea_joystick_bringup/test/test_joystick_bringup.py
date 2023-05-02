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



pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_joystick_bringup.yaml")
    return JoystickMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "joystick"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "joy"


def test_get_driver_device(meta_description):
    assert meta_description.get_driver_device() == "/dev/input/js0"


def test_get_driver_auto_repeat_rate(meta_description):
    assert meta_description.get_driver_autorepeat_rate() == 10.0


def test_get_driver_dead_zone(meta_description):
    assert meta_description.get_driver_dead_zone() == 0.1


def test_get_type(meta_description):
    assert meta_description.get_type() == "xbox"
