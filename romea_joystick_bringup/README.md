# 1) Overview #

The romea_joy_bringup package provides  : 

 - launch files able to launch ros2 joystick drivers according a meta-description file provided by user (see next section for joystick meta-description file overview), supported drivers are :

   - [joy](https://github.com/ros-drivers/joystick_drivers)
   - [ds4_driver](https://github.com/naoki-mizuno/ds4_driver)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_joy_bringup gps_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse joystick meta-description file
   


# 2) Joystick meta-description #

As seen below GPS meta-description file is a yaml file constituted by six items. The first item is the name of sensor defined by user. The second one is the configuration of ROS2 driver used to control GPS receiver (see section 4 for more explanations). The third item is the configuration of ROS2 driver used to deal with NTRIP communication in order to broadcast differential corrections to GPS receiver (see section 5 for more explanations). The fourth item provides basics specifications of the GPS receiver and the fifth item specifies where the GPS receiver antenna is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, GPS topics are always the same names for each drivers or simulator plugins.       

Example :
```yaml
  name: "joystick" #name of the joystick
  driver: #joystick driver configuration
    pkg: "joy"  # ros2 driver package choiced by user and its parameters 
    device: "/dev/input/js0"
    autorepeat_rate: 10
    deadzone: 0.1
  configuration: # joystick basic specifications
    type: xbox #  joystick type
  records:
    joy: true # joy topic will be recorded into bag
```

# 4) Supported joystick models

Supported joystick model are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| xbox   | (360 , one)|
| dualshock |         |

You can find specifications of each joystick in config directory of romea_joy_description package.

# 5) Supported GPS joystick ROS2 drivers

Supported drivers are [joy](https://github.com/ros-drivers/joystick_drivers) and  [ds4_driver](https://github.com/naoki-mizuno/ds4_driver). In order to used one of them, you can specify driver item in joystick meta-description file like this:

- joy driver:

```yaml
  pkg: "joy"  # ROS2 package name  
    device:  "/dev/input/js0"  # input device
    baudrate: 115200 # serial baudrate
    autorepeat_rate: 10
    deadzone: 0.1
```

* ds4_driver driver:

```yaml
  pkg: "ds4_driver"  # ROS2  package name  
    device:  "/dev/input/js0"  # input device
    autorepeat_rate: 10
    deadzone: 0.1
```

For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is red by the main launch file called gps_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always pubkish in same topic called:

- joy(sensor_msgs/Joy)
