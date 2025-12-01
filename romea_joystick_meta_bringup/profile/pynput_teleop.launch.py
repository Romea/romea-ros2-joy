# Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    launch = LaunchDescription()

    if "replay" not in mode:

        launch.add_action(
            Node(
                package="pynput_teleop",
                executable="pynput_teleop.py",
                name="driver",
                parameters=[
                    {
                        "up_down": {"is_incremental": True, "increment": 0.1, "value_max": 1.0},
                        "left_right": {"is_incremental": True, "increment": 0.1, "value_max": 1.0},
                        "publish": {"rate": int(rate)}
                    }
                ]
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )
