# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


import os
import pytest

from romea_joystick_bringup import JoystickMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(os.getcwd(), "test_joystick_bringup.yaml")
    return JoystickMetaDescription(meta_description_filename)


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
