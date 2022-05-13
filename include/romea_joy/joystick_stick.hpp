#ifndef _JoystickStick_HPP
#define _JoystickStick_HPP

#include "joystick_axe.hpp"

namespace romea {

class JoystickStick : public JoystickAxe
{

public :

  JoystickStick(const int & axis_id,
                const double & deadzone,
                const Range & range);

  virtual ~JoystickStick()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)override;

private :

  double deadzone_;
  double scale_;
};

}
#endif
