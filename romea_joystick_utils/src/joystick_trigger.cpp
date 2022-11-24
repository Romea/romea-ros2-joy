#include "romea_joystick_utils/joystick_trigger.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickTrigger::JoystickTrigger(const int & axis_id,
                                 const Range & range):
  JoystickAxe(axis_id, JoystickAxe::TRIGGER),
  has_been_pressed_(false),
  scale_(1/(range.last-range.first)),
  offset_(range.first)
{
}

//-----------------------------------------------------------------------------
void JoystickTrigger::update(const sensor_msgs::msg::Joy & joy_msg)
{
  value_ = scale_*(joy_msg.axes[id_]-offset_);

  if (!has_been_pressed_)
  {
    if (std::abs(value_) >std::numeric_limits<double>::epsilon())
    {
      has_been_pressed_ = true;
    } else {
      value_ = 0;
    }
  }
}

}  // namespace romea
