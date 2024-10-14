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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import yaml


def launch_setup(context, *args, **kwargs):

    executable = LaunchConfiguration("executable").perform(context)
    config_path = LaunchConfiguration("config_path").perform(context)

    # device = LaunchConfiguration("device").perform(context)
    # dead_zone = LaunchConfiguration("dead_zone").perform(context)
    # autorepeat_rate = LaunchConfiguration("autorepeat_rate").perform(context)
    # parameters=[
    #     # {"device_name": device},
    #     {"autorepeat_rate": float(autorepeat_rate)},
    #     {"deadzone": float(dead_zone)},
    # ],

    driver = LaunchDescription()

    print(f'config_path: {config_path}')
    with open(config_path, 'r') as file:
        config_parameters = yaml.safe_load(file)

    driver_node = Node(
        package="joy",
        executable=executable,
        name="driver",
        parameters=[config_parameters],
        arguments=[],
        output={
            'stdout': 'log',
            'stderr': 'log',
        }
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []

    declared_arguments = [
        DeclareLaunchArgument("executable"),
        DeclareLaunchArgument("config_path")
    ]

    # declared_arguments.append(DeclareLaunchArgument("device"))

    # # just to be compatible
    # declared_arguments.append(DeclareLaunchArgument("frame_id"))

    # declared_arguments.append(DeclareLaunchArgument("autorepeat_rate"))

    # declared_arguments.append(DeclareLaunchArgument("dead_zone"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
