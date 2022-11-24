#include "romea_joystick_utils/joystick_stick.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickStick::JoystickStick(const int & axis_id,
                             const double & deadzone,
                             const Range & range):
  JoystickAxe(axis_id, JoystickAxe::STICK),
  deadzone_(deadzone),
  scale_(1./((range.last-range.first)/2.-deadzone))
{
}

//-----------------------------------------------------------------------------
void JoystickStick::update(const sensor_msgs::msg::Joy & joy_msg)
{
  double val = joy_msg.axes[id_];

  if (std::abs(val) < deadzone_)
  {
    value_ = 0;
  } else {
    if (val > 0)
    {
      value_ = (val-deadzone_)*scale_;
    } else {
      value_ = (val+deadzone_)*scale_;
    }
  }
}

}  // namespace romea