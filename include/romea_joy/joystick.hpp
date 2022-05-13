#ifndef _Joystick_HPP_
#define _Joystick_HPP_

//ros
#include <rclcpp/node.hpp>

//romea
#include <romea_common_utils/params/node_parameters.hpp>
#include "joystick_button.hpp"
#include "joystick_configuration.hpp"
#include "joystick_stick.hpp"
#include "joystick_trigger.hpp"
#include "joystick_directional_pad.hpp"
#include "joystick_remapping.hpp"

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
           const std::string & joystick_type);

  Joystick(std::shared_ptr<rclcpp::Node> node,
           const std::string & joystick_type,
           const Remappings & name_remappings,
           bool use_only_remapped);

  void registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback);

  void registerButtonCallback(const std::string & button_name,
                              const JoystickButton::Event & event_type,
                              JoystickButton::CallbackFunction && callback);

  const int & getButtonValue(const std::string & button_name)const;

  const double & getAxisValue(const std::string & axis_name)const;


private :

  void load_devices_(const YAML::Node & joystick_configuration,
                     const JoystickRemapping & joystick_remapping);

  void load_axes_(const YAML::Node & joystick_configuration,
                  const JoystickRemapping & joystick_remapping);

  void load_buttons_(const YAML::Node & joystick_configuration,
                     const JoystickRemapping & joystick_remapping);

  void load_directional_pads_(const YAML::Node & axes_configuration,
                              const JoystickRemapping & joystick_remapping);

  void load_sticks_(const YAML::Node & axes_configuration,
                    const JoystickRemapping & joystick_remapping);

  void load_triggers_(const YAML::Node & axes_configuration,
                      const JoystickRemapping & joystick_remapping);

  void processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr  msg);


private :

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;

  std::map<std::string,JoystickButton::Ptr> buttons_;
  std::map<std::string,JoystickAxe::Ptr> axes_;

  OnReceivedMsgCallback on_received_msg_callback_;
};
}

#endif
