// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_
#define ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_

// std
#include <memory>

// ros
#include "sensor_msgs/msg/joy.hpp"

// romea core
#include "romea_core_common/math/Interval.hpp"


namespace romea
{

class JoystickAxe
{
public:
  using Ptr = std::unique_ptr<JoystickAxe>;

public:
  explicit JoystickAxe(const int & axis_id);

  virtual void update(const sensor_msgs::msg::Joy & joy_msg);

  const int & getId()const;

  const double & getValue()const;

protected:
  int id_;
  double value_;
};

}  // namespace romea

#endif  // ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_
