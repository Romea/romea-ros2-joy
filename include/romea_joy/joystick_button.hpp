#ifndef _JoystickButton_HPP
#define _JoystickButton_HPP

//ros
#include <sensor_msgs/msg/joy.hpp>

//std
#include <functional>

namespace romea
{

class JoystickButton
{

public :

  using CallbackFunction = std::function<void(void)> ;

  using Ptr = std::unique_ptr<JoystickButton>;

  enum Event
  {
    PRESSED,
    RELEASED,
    TOGGLED
  };

public :

  JoystickButton(const int & button_id);

  void update(const sensor_msgs::msg::Joy & joy_msg);

  void registerCallback(Event event, CallbackFunction &&function);

  const int & getValue()const;

  const int & getId()const;

private :

  int id_;
  int value_;

  CallbackFunction pressed_callback_;
  CallbackFunction released_callback_;
  CallbackFunction toggled_callback_;

};

}

#endif
