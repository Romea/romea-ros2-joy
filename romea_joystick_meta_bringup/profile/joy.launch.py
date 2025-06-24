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


def launch_setup(context, *args, **kwargs):

    id = LaunchConfiguration("id").perform(context)

    mode = LaunchConfiguration("mode").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    launch = LaunchDescription()

    if "replay" not in mode:
        launch.add_action(
            Node(
                package="joy",
                executable="joy_node",
                name="driver",
                parameters=[
                    {
                        "device_id": int(id),
                        "autorepeat_rate": float(rate),
                        "deadzone": 0.05,
                        "coalesce_interval": 0.001,
                    },
                ]
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("id", default_value="0"),
            OpaqueFunction(function=launch_setup)
        ]
    )
