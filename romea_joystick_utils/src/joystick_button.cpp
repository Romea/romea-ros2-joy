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

// std
#include <limits>

// local
#include "romea_joystick_utils/joystick_button.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
JoystickButton::JoystickButton(const int & button_id)
: id_(button_id),
  value_(std::numeric_limits<int>::max()),
  pressed_callback_(),
  released_callback_(),
  toggled_callback_()
{
}

//-----------------------------------------------------------------------------
void JoystickButton::registerCallback(Event event, CallbackFunction && function)
{
  switch (event) {
    case PRESSED:
      pressed_callback_ = function;
      break;
    case RELEASED:
      released_callback_ = function;
      break;
    case TOGGLED:
      toggled_callback_ = function;
      break;
    default:
      break;
  }
}


//-----------------------------------------------------------------------------
void JoystickButton::update(const sensor_msgs::msg::Joy & joy_msg)
{
  int delta = joy_msg.buttons[id_] - value_;

  if (delta == 1 && pressed_callback_) {
    pressed_callback_();
  }

  if (delta == -1 && released_callback_) {
    released_callback_();
  }

  if (std::abs(delta) == 1 && toggled_callback_) {
    toggled_callback_();
  }

  value_ = joy_msg.buttons[id_];
}

//-----------------------------------------------------------------------------
const int & JoystickButton::getValue()const
{
  return value_;
}

//-----------------------------------------------------------------------------
const int & JoystickButton::getId()const
{
  return id_;
}

}  // namespace ros2
}  // namespace romea
