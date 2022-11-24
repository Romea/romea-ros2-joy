#ifndef ROMEA_JOY_JOYSTICK_STICK_HPP_
#define ROMEA_JOY_JOYSTICK_STICK_HPP_

#include "romea_joystick_utils/joystick_axe.hpp"

namespace romea {

class JoystickStick : public JoystickAxe
{
public :

  JoystickStick(const int & axis_id,
                const double & deadzone,
                const Range & range);

  virtual ~JoystickStick() = default;

  void update(const sensor_msgs::msg::Joy & joy_msg) override;

private :

  double deadzone_;
  double scale_;
};

}  // namespace romea

#endif  // ROMEA_JOY_JOYSTICK_STICK_HPP_
