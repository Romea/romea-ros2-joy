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

As seen below joystick meta-description file is a yaml file constituted by four items. The first item is the name of joystick defined by user. The second one is the configuration of ROS2 driver used to control joystick (see section 4 for more explanations). The third item provides basics specifications of the joystick (type, model). Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, joy topics are always the same for each drivers.       

Example :
```yaml
  name: joystick #name of the joystick
  driver: #joystick driver configuration
    package: joy  # ros2 driver package 
    executable: joy_node  # ros2 node launch
    parameters: # parameters of driver node
      autorepeat_rate: 10.0
      deadzone: 0.1
  configuration: # joystick basic specifications
    type: xbox #  joystick type
  records:
    joy: true # joy topic will be recorded into bag
```

# 3) Supported joystick models

Supported joystick model are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| xbox   | (360 , one)|
| dualshock |         |

You can find specifications of each joystick in config directory of romea_joy_description package.

# 4) Supported joystick ROS2 drivers

Supported drivers are [joy](https://github.com/ros-drivers/joystick_drivers) and  [ds4_driver](https://github.com/naoki-mizuno/ds4_driver). In order to used one of them, you can specify driver item in joystick meta-description file like this:

- joy driver:

```yaml
  package: joy  # ROS2 package name
  executable: joy_node # node to be launched
  parameters: # parameters of driver node
    autorepeat_rate: 10.0
    deadzone: 0.1
```

* ds4_driver driver:

```yaml
  package: ds4_driver  # ROS2  package name
  executable: ds4_driver_node  # node to be launched
    device:  "/dev/input/js0"  # input device
    autorepeat_rate: 10
    deadzone: 0.1
```

For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is read by the main launch file called joystick_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always publish in same topic called:

- joy(sensor_msgs/Joy)
