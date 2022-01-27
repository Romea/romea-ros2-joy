#include "romea_joy/joystick_axis.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickAxis::JoystickAxis(const int & axis_id):
  id_(axis_id),
  value_(0)
{
}

//-----------------------------------------------------------------------------
const double & JoystickAxis::getValue()const
{
  return value_;
}

//-----------------------------------------------------------------------------
const int & JoystickAxis::getId()const
{
  return id_;
}

}
