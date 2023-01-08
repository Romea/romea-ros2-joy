// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_joystick_utils/joystick_axe.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickAxe::JoystickAxe(const int & axis_id)
: id_(axis_id),
  value_(0)
{
}

//-----------------------------------------------------------------------------
void JoystickAxe::update(const sensor_msgs::msg::Joy & joy_msg)
{
  value_ = joy_msg.axes[id_];
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

}  // namespace romea
