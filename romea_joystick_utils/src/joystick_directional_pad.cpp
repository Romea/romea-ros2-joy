#include "romea_joystick_utils/joystick_directional_pad.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickDirectionalPad::JoystickDirectionalPad(const int & axis_id,
                                               const Range & range):
  JoystickAxe(axis_id, JoystickAxe::PAD),
  scale_(2./(range.last-range.first))
{
}

//-----------------------------------------------------------------------------
void JoystickDirectionalPad::update(const sensor_msgs::msg::Joy & joy_msg)
{
  value_ = scale_*joy_msg.axes[id_];
}



}
