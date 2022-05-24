  #include "romea_joy/joystick_axe.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickAxe::JoystickAxe(const int & axis_id,
                           const Type & axis_type):
  id_(axis_id),
  type_(axis_type),
  value_(0)
{
}

//-----------------------------------------------------------------------------
const double & JoystickAxe::getValue()const
{
  return value_;
}

//-----------------------------------------------------------------------------
const int & JoystickAxe::getId()const
{
  return id_;
}

//-----------------------------------------------------------------------------
const JoystickAxe::Type & JoystickAxe::getType()const
{
  return type_;
}

}
