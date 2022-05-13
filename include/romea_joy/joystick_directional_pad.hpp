#ifndef _JoystickDirectionalPad_HPP
#define _JoystickDirectionalPad_HPP

#include "joystick_axe.hpp"

namespace romea {

class JoystickDirectionalPad : public JoystickAxe
{

public :

  JoystickDirectionalPad(const int & axis_id,
                         const Range & range);

  virtual ~JoystickDirectionalPad()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)override;

private:

  double scale_;
};

}
#endif
