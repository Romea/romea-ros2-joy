from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from romea_joystick_bringup import (
    get_joystick_name,
    has_joystick_driver_configuration,
    get_joystick_driver_pkg,
    get_joystick_device,
    get_joystick_autorepeat_rate,
    get_joystick_dead_zone,
)

import yaml

def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)

def get_meta_description(context):

    meta_description_yaml_filename = LaunchConfiguration(
        "meta_description_filename"
    ).perform(context)

    with open(meta_description_yaml_filename) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not has_joystick_driver_configuration(meta_description):
        return []

    joystick_name = get_joystick_name(meta_description)

    if robot_namespace != "":
        frame_id = robot_namespace + "_" + joystick_name + "_link"
    else:
        frame_id = joystick_name + "_link"


    print("joystick driver")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_joystick_bringup"),
                        "launch",
                        "drivers/"
                        + get_joystick_driver_pkg(meta_description)
                        + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": get_joystick_device(meta_description),
            "autorepeat_rate": str(get_joystick_autorepeat_rate(meta_description)),
            "dead_zone": str(get_joystick_dead_zone(meta_description)),
            "frame_id": frame_id,
        }.items(),
    )

    actions = [
        PushRosNamespace(robot_namespace),
        PushRosNamespace(joystick_name),
        driver,
    ]

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("meta_description_filename")
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
