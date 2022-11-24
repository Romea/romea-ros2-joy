#include "romea_joystick_utils/joystick_button.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickButton::JoystickButton(const int & button_id):
  id_(button_id),
  value_(std::numeric_limits<int>::max()),
  pressed_callback_(),
  released_callback_(),
  toggled_callback_()
{
}

//-----------------------------------------------------------------------------
void JoystickButton::registerCallback(Event event, CallbackFunction && function)
{
  switch (event)
  {
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
  int delta = joy_msg.buttons[id_]-value_;

  if (delta == 1 && pressed_callback_)
  {
    pressed_callback_();
  }

  if ( delta == -1 && released_callback_)
  {
    released_callback_();
  }

  if (std::abs(delta) == 1 && toggled_callback_)
  {
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

}  // namespace romea
