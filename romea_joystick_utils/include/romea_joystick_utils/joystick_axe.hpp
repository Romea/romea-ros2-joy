#ifndef ROMEA_JOY_JOYSTICK_AXE_HPP_
#define ROMEA_JOY_JOYSTICK_AXE_HPP_

// std
#include <memory>

// romea
#include <sensor_msgs/msg/joy.hpp>
#include <romea_core_common/math/Interval.hpp>

namespace romea {

class JoystickAxe
{
public :
  using Ptr = std::unique_ptr<JoystickAxe>;

public :

  explicit JoystickAxe(const int & axis_id);

  virtual void update(const sensor_msgs::msg::Joy & joy_msg);

  const int & getId()const;

  const double & getValue()const;

protected :

  int id_;
  double value_;
};

}  // namespace romea

#endif  // ROMEA_JOY_JOYSTICK_AXE_HPP_
