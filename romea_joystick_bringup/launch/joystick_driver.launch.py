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

import yaml


def launch_setup(context, *args, **kwargs):

    description_yaml_file = LaunchConfiguration("description_yaml_file").perform(
        context
    )

    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    with open(description_yaml_file) as f:
        device = yaml.safe_load(f)

    if "driver" not in device:
       return []

    if robot_namespace != "":
        frame_id = robot_namespace + "_" + device["name"] + "_link"
    else:
        frame_id = device["name"] + "_link"


    print("joystick driver")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_joystick_bringup"),
                        "launch",
                        "drivers/" + device["driver"]["pkg"] + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": device["driver"]["device"],
            "autorepeat_rate": str(device["driver"]["autorepeat_rate"]),
            "frame_id": frame_id,
        }.items(),
    )

    actions = [
        PushRosNamespace(robot_namespace),
        PushRosNamespace(device["name"]),
        driver,
    ]

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("description_yaml_file"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
