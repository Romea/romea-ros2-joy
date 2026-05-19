# romea_joystick_utils

## 1) Overview

`romea_joystick_utils` provides C++ and Python utilities for handling joystick inputs in ROS 2 applications.

The package provides tools to:

* retrieve joystick axes and button values from `sensor_msgs/msg/Joy`
* apply joystick-specific remappings
* convert joystick layouts into ROS 2 compatible mappings
* trigger callbacks from joystick button events
* simplify the integration of joystick-controlled applications

The package is designed to provide reusable joystick utilities shared across the `romea_*` ecosystem.

---

## 2) Joystick configurations

Joystick layouts are defined in YAML configuration files located in the `config/` directory.

Available configurations include:

* `microsoft_xbox.yaml`
* `sony_dualshock4.yaml`
* `sony_dualshock4_ds4_driver.yaml`
* `chengong_hj60ss.yaml`
* `pynput_teleop.yaml`

These configuration files describe the layout of the `sensor_msgs/msg/Joy` message for each device, including:

* button mappings
* axis mappings
* trigger mappings
* joystick-specific layouts

---

### Example joystick configuration

The following example describes the layout of a Microsoft Xbox controller.

It defines how joystick buttons and axes are mapped into the `sensor_msgs/msg/Joy` message:

```yaml
type: microsoft_xbox
joy_msg_layout:
  buttons:
    mapping: {A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, back: 6, start: 7, power: 8, Left_Stick_Button: 9, Right_Stick_Button: 10}
    values: {unpressed: 0, pressed: 1}
  axes:
    directional_pads:
      mapping: {Horizontal_Directional_Pad: 6,Vertical_Directional_Pad: 7}
      values:
        range: [-1.0, 1.0]

    sticks:
      mapping: {Horizontal_Left_Stick: 0, Vertical_Left_Stick: 1, Horizontal_Right_Stick: 3, Vertical_Right_Stick: 4}
      values:
        range: [-1.0, 1.0]

    triggers:
      mapping: {LT: 2, RT: 5}
      values:
        range: [1.0, -1.0]
```

This configuration defines:

* the indices of buttons and axes in the `sensor_msgs/msg/Joy` message
* the semantic names associated with joystick controls
* the expected value ranges for each input type

---

## 3) Python utilities

The Python module provides utilities to generate joystick remappings from joystick layouts.

Available functions include:

| Function                           | Description                                 |
| ---------------------------------- | ------------------------------------------- |
| `get_joystick_configuration`       | Loads a joystick configuration file         |
| `get_joystick_buttons_mapping`     | Returns joystick button mappings            |
| `get_joystick_axes_mapping`        | Returns joystick axis mappings              |
| `apply_joystick_remapping`         | Applies user remappings to joystick layouts |
| `apply_joystick_buttons_remapping` | Applies button-only remappings              |

These utilities are used by `romea_joystick_meta_bringup` to generate joystick remapping configuration files.

---

### Example joystick remapping

The package can convert semantic joystick command names into the corresponding indices of the `sensor_msgs/msg/Joy` message.

#### Example Python code

```python
import yaml

from romea_joystick_utils import (
    get_joystick_configuration,
    apply_joystick_remapping,
)

joystick_configuration = get_joystick_configuration(
    "microsoft_xbox"
)

teleop_configuration = {
    "axes": {
        "front_steering_angle": "Horizontal_Left_Stick",
        "rear_steering_angle": "Horizontal_Right_Stick",
        "forward_speed": "LT",
        "backward_speed": "RT",
    },

    "buttons": {
        "slow_mode": "LB",
        "turbo_mode": "RB",
    }
}

remapping = apply_joystick_remapping(
    joystick_configuration,
    teleop_configuration,
)

with open("joystick_remapping.yaml", "w") as file:
    yaml.dump(remapping, file, sort_keys=False)
```

#### Generated remapping

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

These indices can then be used directly to access joystick values from the `axes` and `buttons` arrays of the `sensor_msgs/msg/Joy` message.

---

## 4) C++ utilities

The package provides C++ helper classes for handling joystick messages.

### Available classes

| Class            | Description                                         |
| ---------------- | --------------------------------------------------- |
| `Joystick`       | Main class used to access joystick axes and buttons |
| `JoystickButton` | Handles joystick button states and events           |
| `JoystickAxe`    | Handles joystick axis values                        |

The `Joystick` class provides a high-level interface for handling joystick messages.

Main features include:

* retrieving axis values by semantic name
* retrieving button states by semantic name
* registering callbacks on button events
* automatic integration with ROS 2 subscriptions

### Example

```cpp
std::map<std::string, size_t> axes_mapping = {
  {"front_steering_angle", 0},
  {"rear_steering_angle", 3},
  {"forward_speed", 2},
  {"backward_speed", 5}
};

std::map<std::string, size_t> buttons_mapping = {
  {"slow_mode", 4},
  {"turbo_mode", 5}
};

auto joystick = std::make_shared<romea::ros2::Joystick>(
  node,
  axes_mapping,
  buttons_mapping);

double steering =
  joystick->getAxeValue("front_steering_angle");

int turbo =
  joystick->getButtonValue("turbo_mode");

joystick->registerButtonPressedCallback(
  "turbo_mode",
  []()
  {
    std::cout << "Turbo mode enabled" << std::endl;
  });

joystick->registerButtonReleasedCallback(
  "turbo_mode",
  []()
  {
    std::cout << "Turbo mode disabled" << std::endl;
  });
```

`JoystickButton` supports event-based callbacks such as:

* button pressed
* button released
* button hold

This mechanism simplifies the implementation of teleoperation and supervision behaviors.
