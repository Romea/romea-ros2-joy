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
#include <chrono>
#include <limits>

// local
#include "romea_joystick_utils/joystick_button.hpp"

namespace
{

//-----------------------------------------------------------------------------
int64_t get_press_time_ms_(const sensor_msgs::msg::Joy & joy_msg)
{
  return joy_msg.header.stamp.sec * 1000 + joy_msg.header.stamp.nanosec / 1000000LL;
}

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
JoystickButton::JoystickButton(const int & button_id)
: id_(button_id),
  value_(std::numeric_limits<int>::max()),
  hold_counter_(0),
  hold_threshold_(10),
  was_held_(false),
  double_press_delay_ms_(300),
  previous_press_time_ms_(-300),
  pressed_callback_(),
  released_callback_(),
  toggled_callback_(),
  held_callback_(),
  double_pressed_callback_()
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
    case HELD:
      held_callback_ = function;
      break;
    case UNHELD:
      unheld_callback_ = function;
      break;
    case DOUBLE_PRESSED:
      double_pressed_callback_ = function;
      break;
    default:
      break;
  }
}


//-----------------------------------------------------------------------------
void JoystickButton::update(const sensor_msgs::msg::Joy & joy_msg)
{
  int64_t current_press_time_ms = get_press_time_ms_(joy_msg);
  int new_value = joy_msg.buttons[id_];
  int delta = new_value - value_;

  if (delta == 1 && pressed_callback_) {
    pressed_callback_();
  }

  if (delta == -1 && released_callback_) {
    released_callback_();
  }

  if (std::abs(delta) == 1 && toggled_callback_) {
    toggled_callback_();
  }

  if (delta == 1 && double_pressed_callback_) {
    if (current_press_time_ms - previous_press_time_ms_ <= double_press_delay_ms_) {
      double_pressed_callback_();
    }
    previous_press_time_ms_ = current_press_time_ms;
  }

  if (new_value == 1) {
    ++hold_counter_;
    if (hold_counter_ >= hold_threshold_) {
      if (!was_held_ && held_callback_) {
        held_callback_();
      }
      was_held_ = true;
    }
  } else {
    if (was_held_ && unheld_callback_) {
      unheld_callback_();
    }
    hold_counter_ = 0;
    was_held_ = false;
  }

  value_ = new_value;
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
