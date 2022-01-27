#ifndef _JoystickDirectionalPad_HPP
#define _JoystickDirectionalPad_HPP

#include "joystick_axis.hpp"

namespace romea {

class JoystickDirectionalPad : public JoystickAxis
{

public :

  JoystickDirectionalPad(const int & axis_id);

  virtual ~JoystickDirectionalPad()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)override;

};

}
#endif
