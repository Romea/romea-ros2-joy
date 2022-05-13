#ifndef _JoystickTrigger_HPP
#define _JoystickTrigger_HPP

#include "joystick_axe.hpp"

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

}

#endif
