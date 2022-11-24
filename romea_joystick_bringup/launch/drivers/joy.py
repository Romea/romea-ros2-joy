from launch import LaunchDescription

from launch.actions import (
   IncludeLaunchDescription,
   DeclareLaunchArgument,
   OpaqueFunction,
   SetEnvironmentVariable,
)

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):

   device = LaunchConfiguration("device").perform(context)
   autorepeat_rate = LaunchConfiguration("baudrate").perform(context)

   driver = LaunchDescription()

   driver_node = Node(
        package="joy",
        executable="joy_node",
        name="driver",
        output="screen",
        parameters=[
                {"dev": device},
                {"autorepeat_rate": float(autorepeat_rate)},
        ],
        arguments=[]
    )
    
    driver.add_action(driver_node)

    return [driver]

def generate_launch_description():

   declared_arguments = []
   declared_arguments.append(DeclareLaunchArgument("device"))
   declared_arguments.append(DeclareLaunchArgument("frame_id")) # just to be compatible
   declared_arguments.append(DeclareLaunchArgument("autorepeat_rate")) 


   return LaunchDescription(
       declared_arguments + [OpaqueFunction(function=launch_setup)]
   )
