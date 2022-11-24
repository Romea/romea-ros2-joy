#ifndef ROMEA_JOY_JOYSTICK_TRIGGER_HPP_
#define ROMEA_JOY_JOYSTICK_TRIGGER_HPP_

#include "romea_joystick_utils/joystick_axe.hpp"

namespace romea {

class JoystickTrigger : public JoystickAxe
{
public :

  JoystickTrigger(const int &axis_id,
                  const Range & range);

  void update(const sensor_msgs::msg::Joy & joy_msg)override;

private :

  double has_been_pressed_;
  double scale_;
  double offset_;
};

}  // namespace romea

#endif  // ROMEA_JOY_JOYSTICK_TRIGGER_HPP_
