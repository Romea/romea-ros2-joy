#ifndef _JoystickAxe_HPP
#define _JoystickAxe_HPP

#include <sensor_msgs/msg/joy.hpp>
#include <romea_core_common/math/Interval.hpp>

namespace romea {

class JoystickAxe
{
public :

  using Ptr = std::unique_ptr<JoystickAxe>;

  enum Type {
    PAD,
    STICK,
    TRIGGER,
  };

  struct Range
  {
    double first;
    double last;
  };

public :

  JoystickAxe(const int & axis_id,
               const Type & axis_type);

  virtual ~JoystickAxe()=default;

  virtual void update(const sensor_msgs::msg::Joy & joy_msg)=0;

  const int & getId()const;

  const Type & getType()const;

  const double & getValue()const;

protected :

  int id_;
  Type type_;
  double value_;

};

}
#endif
