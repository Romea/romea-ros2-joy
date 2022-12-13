#ifndef ROMEA_JOY_JOYSTICK_HPP_
#define ROMEA_JOY_JOYSTICK_HPP_

// std
#include <functional>
#include <memory>
#include <string>
#include <map>

// ros
#include <rclcpp/node.hpp>

// romea
#include <romea_common_utils/params/node_parameters.hpp>
#include "romea_joystick_utils/joystick_button.hpp"
#include "romea_joystick_utils/joystick_axe.hpp"

namespace  romea
{

class Joystick
{
public :

  using OnReceivedMsgCallback = std::function<void(const Joystick &)>;
  using Remappings = std::map<std::string, std::string>;

public :

  Joystick(std::shared_ptr<rclcpp::Node> node,
           const std::map<std::string, int> & axes_mapping,
           const std::map<std::string, int> & buttons_mapping);


  void registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback);

  void registerButtonCallback(const std::string & button_name,
                              const JoystickButton::Event & event_type,
                              JoystickButton::CallbackFunction && callback);

  const int & getButtonValue(const std::string & button_name)const;

  const double & getAxeValue(const std::string & axe_name)const;

  std::map<std::string, int> get_mapping()const;

private :

  void init_joy_sub_(std::shared_ptr<rclcpp::Node> node);

  void init_axes(const std::map<std::string, int> & axes_mapping);

  void init_buttons(const std::map<std::string, int> & buttons_mapping);

  void processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr  msg);


private :

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  
  std::map<std::string, JoystickAxe::Ptr> axes_;
  std::map<std::string, JoystickButton::Ptr> buttons_;

  OnReceivedMsgCallback on_received_msg_callback_;
};

}  // namespace romea

#endif  // ROMEA_JOY_JOYSTICK_HPP_
