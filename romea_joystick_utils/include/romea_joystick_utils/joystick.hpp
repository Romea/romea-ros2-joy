// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_JOYSTICK_UTILS__JOYSTICK_HPP_
#define ROMEA_JOYSTICK_UTILS__JOYSTICK_HPP_

// std
#include <functional>
#include <memory>
#include <string>
#include <map>


// ros
#include "rclcpp/node.hpp"

// romea ros
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_joystick_utils/joystick_button.hpp"
#include "romea_joystick_utils/joystick_axe.hpp"

namespace  romea
{
namespace  ros2
{

class Joystick
{
public:
  using OnReceivedMsgCallback = std::function<void (const Joystick &)>;
  using Remappings = std::map<std::string, std::string>;

public:
  Joystick(
    std::shared_ptr<rclcpp::Node> node,
    const std::map<std::string, int> & axes_mapping,
    const std::map<std::string, int> & buttons_mapping);


  void registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback);

  void registerButtonCallback(
    const std::string & button_name,
    const JoystickButton::Event & event_type,
    JoystickButton::CallbackFunction && callback);

  const int & getButtonValue(const std::string & button_name)const;

  const double & getAxeValue(const std::string & axe_name)const;

  std::map<std::string, int> get_mapping()const;

private:
  void init_joy_sub_(std::shared_ptr<rclcpp::Node> node);

  void init_axes(const std::map<std::string, int> & axes_mapping);

  void init_buttons(const std::map<std::string, int> & buttons_mapping);

  void processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;

  std::map<std::string, JoystickAxe::Ptr> axes_;
  std::map<std::string, JoystickButton::Ptr> buttons_;

  OnReceivedMsgCallback on_received_msg_callback_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_JOYSTICK_UTILS__JOYSTICK_HPP_
