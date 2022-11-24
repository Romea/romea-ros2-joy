#ifndef ROMEA_JOY_JOYSTICK_DIRECTIONAL_PAD_HPP_
#define ROMEA_JOY_JOYSTICK_DIRECTIONAL_PAD_HPP_

#include "romea_joystick_utils/joystick_axe.hpp"

namespace romea {

class JoystickDirectionalPad : public JoystickAxe
{
public :

  JoystickDirectionalPad(const int & axis_id,
                         const Range & range);

  virtual ~JoystickDirectionalPad() = default;

  void update(const sensor_msgs::msg::Joy & joy_msg) override;

private:
  double scale_;
};

}
#endif  // ROMEA_JOY_JOYSTICK_DIRECTIONAL_PAD_HPP_
