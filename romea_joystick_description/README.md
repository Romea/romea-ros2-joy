# ROMEA Joystick Description #

# 1 Overview #

This package contains :
  - several joystick configurations according joystick models and ros2 driver package use to handle them 
  - a python module able to generate joystick (buttons and axes) remappings according these configurations and user requirements 

# 2 Joystick configuration

In ROS, joystick data (see [sensors_msgs/Joy.msg](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html)) is split in two vectors : one for buttons and one for axes (including sticks, triggers, and occasionally directional pads). The data storage and their ranges are different depending on the joystick model and driver used. A configuration file is provided for each model/driver combination in the config directory containing the data mapping and their ranges, see the example below. This information will be used to configure the teleoperation nodes and all the high level algorithms driven the joystick. Currently, two joystick models (xbox dualshock4) and two driver types (joy,ds4_driver) are supported, others will be added in the future.

```yaml
buttons: # buttons configuration
  mapping: # buttons mapping i.e name of button -> index where button data is stored in buttons vector
    Square: 0
    Triangle: 1
    Right_Directional_Pad: 16
    Down_Directional_Pad: 17
  values: # buttons data ranges
    unpressed: 0
    pressed: 1
axes: # axes configuration
  sticks: # stick configuration
    mapping: # sticks mapping i.e name of stick -> index where stick data is stored in axes vector
      Horizontal_Left_Stick: 0
      Vertical_Left_Stick: 1
      ...
    values: # sticks data ranges
      range: [-1.0, 1.0]
  triggers: # trigger configuration
    mapping:  # triggers mapping i.e name of trigger -> index where stick data is stored in axes vector
      L2: 4
      R2: 5
    values: # trigger data ranges
      range: [0.0, 1.0]
```

# 3 Define joystick remappings

You can define your own remapping rules in using joystick_remapping function from romea_joystick_description module

```python
from romea_joystick_description import joystick_remapping

user_remapping={}
user_remapping["buttons"] = {'start':'Square','stop':'Triangle'}
user_remapping["axes"] = {'linear_speed':'L2','steering_angle':'Horizontal_Left_Stick'}

msg_remapping = joystick_remapping(joystick_type="dualshock4",joystick_driver='ds4_driver',user_joystick_remapping=user_remapping)

print("axes remapping = ",msg_remapping["axes"])
print("buttons remapping = ",msg_remapping["buttons"])
```
which gives the following result
```shell
axes remapping = {'linear_speed':4, 'linear_speed':4}
buttons remapping = {'start':0 , 'stop'=1}
```

