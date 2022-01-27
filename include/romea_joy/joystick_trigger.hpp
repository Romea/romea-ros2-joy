#ifndef _JoystickTrigger_HPP
#define _JoystickTrigger_HPP

#include "joystick_axis.hpp"

namespace romea {


class JoystickTrigger : public JoystickAxis
{

public :

  struct Config
  {
    double unpressed_value;
    double pressed_value;
  };

public :

  JoystickTrigger(const int &axis_id,
                  const double & unpressed_value);

  void update(const sensor_msgs::msg::Joy & joy_msg)override;

private :

  double has_been_pressed_;
  double unpressed_value_;

};

}

#endif
