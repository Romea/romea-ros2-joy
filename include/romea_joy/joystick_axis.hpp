#ifndef _JoystickAxis_HPP
#define _JoystickAxis_HPP

#include <sensor_msgs/msg/joy.hpp>

namespace romea {

class JoystickAxis
{
public :

  using Ptr = std::unique_ptr<JoystickAxis>;

public :

  JoystickAxis(const int & axis_id);

  virtual ~JoystickAxis()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)=0;

  const double & getValue()const;

  const int & getId()const;


protected :

  int id_;
  double value_;
};

}
#endif
