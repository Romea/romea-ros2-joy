from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import SetParameter, PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_common_bringup import device_link_name
from romea_joystick_bringup import JoystickMetaDescription


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):

    meta_description_file_path = LaunchConfiguration(
        "meta_description_file_path"
    ).perform(context)

    return JoystickMetaDescription(meta_description_file_path)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not meta_description.has_driver_configuration():
        return []

    joystick_name = meta_description.get_name()

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_joystick_bringup"),
                        "launch",
                        "drivers/" + meta_description.get_driver_pkg() + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": meta_description.get_driver_device(),
            "autorepeat_rate": str(meta_description.get_driver_autorepeat_rate()),
            "dead_zone": str(meta_description.get_driver_dead_zone()),
            "frame_id": device_link_name(robot_namespace, joystick_name),
        }.items(),
    )

    actions = [
        SetParameter(name="use_sim_time", value=(mode != "live")),
        PushRosNamespace(robot_namespace),
        PushRosNamespace(joystick_name),
        driver,
    ]

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
