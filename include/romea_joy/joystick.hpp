#ifndef _Joystick_HPP_
#define _Joystick_HPP_

//ros
#include <rclcpp/node.hpp>

//romea
#include <romea_common_utils/params/node_parameters.hpp>
#include "joystick_button.hpp"
#include "joystick_stick.hpp"
#include "joystick_trigger.hpp"
#include "joystick_directional_pad.hpp"
#include "joystick_mapping.hpp"

//std
#include <functional>

namespace  romea
{


class Joystick
{
public :

  using OnReceivedMsgCallback = std::function<void(const Joystick &)>;
  using Remappings = std::map<std::string,std::string>;

public :

  Joystick(std::shared_ptr<rclcpp::Node> node,
           const std::string & ns="");

  Joystick(std::shared_ptr<rclcpp::Node> node,
           const Remappings & name_remappings,
           bool use_only_remapped,
           const std::string & ns="");

  void registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback);

  void registerButtonCallback(const std::string & button_name,
                              const JoystickButton::Event & event_type,
                              JoystickButton::CallbackFunction && callback);

  const int & getButtonValue(const std::string & button_name)const;

  const double & getAxisValue(const std::string & axis_name)const;

  const JoystickStick::Config & getSticksConfiguration()const;

  const JoystickTrigger::Config & getTriggersConfiguration()const;

private :

  void addButtons_(NodeParameters & node_parameters,
                   const JoystickMapping & joystick_mapping);

  void addDirectionalPads_(NodeParameters &node_parameters,
                           const JoystickMapping & joystick_mapping);

  void addSticks_(NodeParameters &node_parameters,
                  const JoystickMapping & joystick_mapping);

  void addTriggers_(NodeParameters &node_parameters,
                    const JoystickMapping & joystick_mapping);

  void processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr  msg);

private :

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  std::map<std::string,JoystickButton::Ptr> buttons_;
  std::map<std::string,JoystickAxis::Ptr> axes_;
  JoystickStick::Config sticks_configuration_;
  JoystickTrigger::Config triggers_configuration_;
  OnReceivedMsgCallback on_received_msg_callback_;
};
}

#endif
