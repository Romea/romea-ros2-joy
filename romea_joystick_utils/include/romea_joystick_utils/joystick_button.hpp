// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_
#define ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_

// std
#include <chrono>
#include <functional>
#include <memory>

// ros
#include "sensor_msgs/msg/joy.hpp"


namespace romea
{
namespace ros2
{

class JoystickButton
{
public:
  using CallbackFunction = std::function<void (void)>;

  using Ptr = std::unique_ptr<JoystickButton>;

  enum Event
  {
    PRESSED,
    RELEASED,
    TOGGLED,
    HELD,
    UNHELD,
    DOUBLE_PRESSED
  };

public:
  explicit JoystickButton(const int & button_id);

  void update(const sensor_msgs::msg::Joy & joy_msg);

  void registerCallback(Event event, CallbackFunction && function);

  const int & getValue()const;

  const int & getId()const;

private:
  int id_;
  int value_;

  int hold_counter_;
  int hold_threshold_;
  bool was_held_;

  int double_press_delay_ms_;
  int previous_press_time_ms_;

  CallbackFunction pressed_callback_;
  CallbackFunction released_callback_;
  CallbackFunction toggled_callback_;
  CallbackFunction held_callback_;
  CallbackFunction unheld_callback_;
  CallbackFunction double_pressed_callback_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_
