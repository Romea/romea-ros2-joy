from ament_index_python.packages import get_package_share_directory
import romea_joystick_description
import yaml

def get_joystick_name(joystick_meta_description):
    return joystick_meta_description["name"]

def get_joystick_type(joystick_meta_description):
    return joystick_meta_description["configuration"]["type"]

def has_joystick_driver_configuration(joystick_meta_description):
    return "driver" in joystick_meta_description

def get_joystick_driver_pkg(joystick_meta_description):
    return joystick_meta_description["driver"]["pkg"]

def get_joystick_device(joystick_meta_description):
    return joystick_meta_description["driver"]["device"]

def get_joystick_autorepeat_rate(joystick_meta_description):
    return joystick_meta_description["driver"]["autorepeat_rate"]

def get_joystick_dead_zone(joystick_meta_description):
    return joystick_meta_description["driver"]["deadzone"]
