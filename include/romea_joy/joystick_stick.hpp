#ifndef _JoystickStick_HPP
#define _JoystickStick_HPP

#include "joystick_axis.hpp"

namespace romea {

class JoystickStick : public JoystickAxis
{

public:

  struct Config
  {
    double left_or_down_value;
    double right_or_up_value;
  };

public :

  JoystickStick(const int & axis_id,
                const double & deadzone);

  virtual ~JoystickStick()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)override;

private :

  double deadzone_;
  double scale_;
};

}
#endif
