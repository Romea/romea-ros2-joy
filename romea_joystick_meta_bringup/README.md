# romea_joystick_meta_bringup

## 1) Overview

romea_joystick_meta_bringup provides tools to describe and launch joystick devices using a meta-description approach.

It allows defining joystick configurations in a high-level YAML format and automatically generating consistent ROS 2 artifacts such as:

axis remapping files → convert joystick axis names into sensor_msgs/msg/Joy axis indices
button remapping files → convert button names into sensor_msgs/msg/Joy button indices
launch files → used to start and configure joystick or keyboard input drivers

This package is built on top of romea_common_meta_bringup and specializes it for joystick integration.

## 2) Joystick meta-description concept

A joystick meta-description is a YAML file that defines an input device and how it should be integrated into a ROS 2 system.

It centralizes:

* joystick identification
* hardware configuration
* ROS 2 launch description

### Example meta-description

```yaml
name: joystick

launch:
  - include:
      file: "$(find-pkg-share romea_joystick_meta_bringup)/profile/joy.launch.py"
      arg:
        - name: device
          value: /dev/input/js0

configuration:
  manufacturer: microsoft
  model: xbox
  rate: 10
```


### Launch File Profiles

### Launch File Profiles

The `profile/` directory contains reusable ROS 2 launch files dedicated to input devices.

These launch profiles are used to:

- listen to joystick or keyboard inputs
- publish `sensor_msgs/msg/Joy` messages
- configure the corresponding input drivers

Typical supported joystick devices include:

- Xbox-compatible controllers
- Sony DualShock controllers
- Logitech gamepads

Each profile is intended to be included from the `launch` section of a joystick meta-description. This makes it possible to select the appropriate input device while keeping the meta-description concise and consistent.

## 3) Scripts

romea_joystick_meta_bringup provides several scripts to generate ROS2 artifacts from a joystick meta-description; the usage and generated outputs are described below.

### Generate joystick remapping file
``` bash
generate-joystick-mapping-file \
  meta_description_file_path:=path/to/joystick_meta_description.yaml \
  teleop_configuration_file_path:=path/to/teleop_mapping_configuration.yaml
```

This script generates a joystick remapping configuration file by combining:
- the joystick layout deduced from the joystick model defined in the meta-description
- the teleoperation configuration file, which defines the semantic command names to use

### Example of joystick remapping

The joystick remapping configuration associates semantic command names with joystick axes and buttons.

For example, the following teleop configuration can be used to teleoperate a vehicle with front and rear steering axles:

```yaml
axes: 
  front_steering_angle: Horizontal_Left_Stick
  rear_steering_angle: Horizontal_Right_Stick
  forward_speed: LT
  backward_speed: RT

buttons:
  slow_mode: LB
  turbo_mode: RB
```

The generated remapping converts joystick control names into the corresponding indices of the sensor_msgs/msg/Joy message:

```yaml
axes:
  front_steering_angle: 0
  rear_steering_angle: 3
  forward_speed: 2
  backward_speed: 5

buttons:
  slow_mode: 4
  turbo_mode: 5
```

These indices can then be used to access joystick values directly from the axes and buttons arrays of the sensor_msgs/msg/Joy message.

The remapping process therefore provides an abstraction layer between hardware-specific joystick layouts and high-level teleoperation commands.

### Generate launch file

Generates a ROS 2 launch file from the joystick meta-description.

```bash
generate-launch-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/joystick_meta_description.yaml
```

### Example output
```yaml
launch:
- group:
  - push-ros-namespace: {namespace: robots}
  - push-ros-namespace: {namespace: joystick}
  - let: {name: manufacturer, value: microsoft}
  - let: {name: model, value: xbox}
  - let: {name: rate, value: 10}
  - include:
      file: $(find-pkg-share romea_joystick_meta_bringup)/profile/joy.launch.py
      arg:
        - {name: device, value: /dev/input/js0}
```

### Notes
- the launch file is generated from the launch section of the meta-description
- all configuration values are exposed as let variables
- the selected launch profiles are included automatically

## 4) Usage

The package provides the main launch file:
- joystick.launch.py

### Example
```bash
ros2 launch romea_joystick_meta_bringup joystick.launch.py \
  meta_description_file_path:=path/to/joystick_meta_description.yaml
```

This launch file starts and configures only the selected input driver, such as a joystick driver or the keyboard teleoperation driver.

## 5) Supported input devices

Currently, the package supports the following input devices:

|   Type   | Examples                                            |
| :------: | :-------------------------------------------------- |
| Joystick | Xbox controllers, Sony DualShock, Logitech gamepads |
| Keyboard | standard keyboard devices through `pynput`          |

Support for additional input devices may be added in future releases.