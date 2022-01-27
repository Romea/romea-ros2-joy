#include "romea_joy/joystick_directional_pad.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickDirectionalPad::JoystickDirectionalPad(const int & axis_id):
  JoystickAxis(axis_id)
{

}

//-----------------------------------------------------------------------------
void JoystickDirectionalPad::update(const sensor_msgs::msg::Joy & joy_msg)
{
  value_ = joy_msg.axes[id_];
}



}
