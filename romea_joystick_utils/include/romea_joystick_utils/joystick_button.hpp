// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_
#define ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_

// ros
#include <sensor_msgs/msg/joy.hpp>

// std
#include <functional>
#include <memory>

namespace romea
{

class JoystickButton
{
public:
  using CallbackFunction = std::function<void (void)>;

  using Ptr = std::unique_ptr<JoystickButton>;

  enum Event
  {
    PRESSED,
    RELEASED,
    TOGGLED
  };

public:
  explicit JoystickButton(const int & button_id);

  void update(const sensor_msgs::msg::Joy & joy_msg);

  void registerCallback(Event event, CallbackFunction && function);

  const int & getValue()const;

  const int & getId()const;

private:
  int id_;
  int value_;

  CallbackFunction pressed_callback_;
  CallbackFunction released_callback_;
  CallbackFunction toggled_callback_;
};

}  // namespace romea

#endif  // ROMEA_JOYSTICK_UTILS__JOYSTICK_BUTTON_HPP_
