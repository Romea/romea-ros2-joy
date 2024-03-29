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

// std
#include <map>
#include <memory>
#include <string>
#include <utility>

// local
#include "romea_joystick_utils/joystick.hpp"


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<>
void Joystick::init_joy_sub_<rclcpp::Node>(std::shared_ptr<rclcpp::Node> node)
{
  auto callback = std::bind(&Joystick::processJoyMsg_, this, std::placeholders::_1);
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>("joystick/joy", 1, callback);
}

//-----------------------------------------------------------------------------
template<>
void Joystick::init_joy_sub_<rclcpp_lifecycle::LifecycleNode>(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
  auto callback = std::bind(&Joystick::processJoyMsg_, this, std::placeholders::_1);
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>("joystick/joy", 1, callback);
}

//-----------------------------------------------------------------------------
template<>
Joystick::Joystick<rclcpp::Node>(
  std::shared_ptr<rclcpp::Node> node,
  const std::map<std::string, int> & buttons_mapping)
: joy_sub_(),
  axes_(),
  buttons_(),
  on_received_msg_callback_()
{
  init_buttons(buttons_mapping);
  init_joy_sub_(node);
}

//-----------------------------------------------------------------------------
template<>
Joystick::Joystick<rclcpp_lifecycle::LifecycleNode>(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::map<std::string, int> & buttons_mapping)
: joy_sub_(),
  axes_(),
  buttons_(),
  on_received_msg_callback_()
{
  init_buttons(buttons_mapping);
  init_joy_sub_(node);
}

//-----------------------------------------------------------------------------
template<>
Joystick::Joystick<rclcpp::Node>(
  std::shared_ptr<rclcpp::Node> node,
  const std::map<std::string, int> & axes_mapping,
  const std::map<std::string, int> & buttons_mapping)
: joy_sub_(),
  axes_(),
  buttons_(),
  on_received_msg_callback_()
{
  init_axes(axes_mapping);
  init_buttons(buttons_mapping);
  init_joy_sub_(node);
}

//-----------------------------------------------------------------------------
template<>
Joystick::Joystick<rclcpp_lifecycle::LifecycleNode>(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::map<std::string, int> & axes_mapping,
  const std::map<std::string, int> & buttons_mapping)
: joy_sub_(),
  axes_(),
  buttons_(),
  on_received_msg_callback_()
{
  init_axes(axes_mapping);
  init_buttons(buttons_mapping);
  init_joy_sub_(node);
}


//-----------------------------------------------------------------------------
void Joystick::init_axes(const std::map<std::string, int> & axes_mapping)
{
  for (const auto & [axe_name, axe_id] : axes_mapping) {
    axes_[axe_name] = std::make_unique<JoystickAxe>(axe_id);
  }
}

//-----------------------------------------------------------------------------
void Joystick::init_buttons(const std::map<std::string, int> & buttons_mapping)
{
  for (const auto & [buttom_name, button_id] : buttons_mapping) {
    buttons_[buttom_name] = std::make_unique<JoystickButton>(button_id);
  }
}

//-----------------------------------------------------------------------------
void Joystick::processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (msg->axes.empty() || msg->buttons.empty()) {
    // RCLCPP_ERROR_STREAM(
    //   node_->get_logger(),
    //   "Unavailable joy msg : check if joy node is connected to a proper device");
  } else {
    for (auto & p : buttons_) {
      p.second->update(*msg);
    }

    for (auto & p : axes_) {
      p.second->update(*msg);
    }

    if (on_received_msg_callback_) {
      on_received_msg_callback_(*this);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::registerButtonCallback(
  const std::string & button_name,
  const JoystickButton::Event & event_type,
  JoystickButton::CallbackFunction && callback)
{
  auto it = buttons_.find(button_name);
  if (it == buttons_.end()) {
    std::stringstream ss;
    ss << "No joystick button called ";
    ss << button_name;
    ss << " has been found, register ";
    ss << button_name;
    ss << " callback failed. Check joystick buttons mapping";
    throw(std::runtime_error(ss.str()));
  }

  it->second->registerCallback(event_type, std::move(callback));
}

//-----------------------------------------------------------------------------
void Joystick::registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback)
{
  on_received_msg_callback_ = callback;
}

//-----------------------------------------------------------------------------
const int & Joystick::getButtonValue(const std::string & button_name)const
{
  return buttons_.at(button_name)->getValue();
}

//-----------------------------------------------------------------------------
const double & Joystick::getAxeValue(const std::string & axe_name)const
{
  return axes_.at(axe_name)->getValue();
}

//-----------------------------------------------------------------------------
std::map<std::string, int> Joystick::get_mapping()const
{
  std::map<std::string, int> mapping;
  for (const auto & [axe_id, axe] : axes_) {
    mapping[axe_id] = axe->getId();
  }

  for (const auto & [button_id, button] : buttons_) {
    mapping[button_id] = button->getId();
  }

  return mapping;
}

}  // namespace ros2
}  // namespace romea
