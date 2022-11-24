// romea
#include "romea_joystick_utils/joystick.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const std::string & joystick_type):
  Joystick(node,
           joystick_type,
           std::map<std::string, std::string>(),
           false)
{
}

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const std::string & joystick_type,
                   const Remappings & remappings,
                   bool use_only_remapped):
  node_(node),
  joy_sub_(),
  buttons_(),
  axes_()
{
  YAML::Node joystick_configuration = load_configuration(joystick_type);
  JoystickRemapping joystick_remapping(remappings, use_only_remapped);
  load_devices_(joystick_configuration, joystick_remapping);

  auto callback = std::bind(&Joystick::processJoyMsg_, this, std::placeholders::_1);
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, callback);
}

//-----------------------------------------------------------------------------
void Joystick::processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (msg->axes.empty() || msg->buttons.empty())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      "Unavailable joy msg : check if joy node is connected to a proper device");
  } else {
    for (auto & p : buttons_)
    {
      p.second->update(*msg);
    }

    for (auto & p : axes_)
    {
      p.second->update(*msg);
    }

    if (on_received_msg_callback_)
    {
      on_received_msg_callback_(*this);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::registerButtonCallback(const std::string & button_name,
                                      const JoystickButton::Event & event_type,
                                      JoystickButton::CallbackFunction &&callback)
{
  auto it = buttons_.find(button_name);
  if (it == buttons_.end())
  {
    std::stringstream ss;
    ss << "No joystick button called ";
    ss << button_name;
    ss << " has been found, register ";
    ss << button_name;
    ss << " callback failed. Check joystick buttons mappings or remappings ";
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
void Joystick::load_devices_(const YAML::Node & joystick_configuration,
                             const JoystickRemapping & joystick_remapping)
{
  load_axes_(joystick_configuration, joystick_remapping);
  load_buttons_(joystick_configuration, joystick_remapping);

  if (!joystick_remapping.is_complete())
  {
    std::stringstream msg;
    msg << " Joystisck remapping cannot be completed : ";
    for (const auto & [name, status] : joystick_remapping.get_status())
    {
      msg << " " << name << " " << (status == true ? "assigned" : "not assigned");
    }
    throw std::runtime_error(msg.str());
  }
}

//-----------------------------------------------------------------------------
void Joystick::load_axes_(const YAML::Node & joystick_configuration,
                          const JoystickRemapping & joystick_remapping)
{
  YAML::Node axes_configuration = joystick_configuration["axes"];
  load_directional_pads_(axes_configuration, joystick_remapping);
  load_sticks_(axes_configuration, joystick_remapping);
  load_triggers_(axes_configuration, joystick_remapping);
}

//-----------------------------------------------------------------------------
void Joystick::load_buttons_(const YAML::Node & joystick_configuration,
                             const JoystickRemapping & joystick_remapping)
{
  YAML::Node buttons_configuration = joystick_configuration["buttons"];

  auto buttons_mapping = extract_mappings(buttons_configuration);
  for (const auto & [buttom_name, button_id] : joystick_remapping.apply(buttons_mapping))
  {
    buttons_[buttom_name] = std::make_unique<JoystickButton>(button_id);
  }
}

//-----------------------------------------------------------------------------
void Joystick::load_directional_pads_(const YAML::Node & axes_configuration,
                                      const JoystickRemapping & joystick_remapping)
{
  auto dpads_configuration = axes_configuration["directional_pads"];

  if (dpads_configuration)
  {
    auto dpads_range = extract_range(dpads_configuration);
    auto dpads_mapping = extract_mappings(dpads_configuration);
    for (const auto & [dpad_name, dpad_id] : joystick_remapping.apply(dpads_mapping))
    {
      axes_[dpad_name] = std::make_unique<JoystickDirectionalPad>(dpad_id, dpads_range);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::load_sticks_(const YAML::Node & axes_configuration,
                            const JoystickRemapping & joystick_remapping)

{
  auto sticks_configuration = axes_configuration["sticks"];

  if (sticks_configuration)
  {
    auto sticks_range = extract_range(sticks_configuration);
    auto sticks_mapping = extract_mappings(sticks_configuration);
    double deadzone = sticks_configuration["deadzone"].as<double>();
    for (const auto & [stick_name, stick_id] : joystick_remapping.apply(sticks_mapping))
    {
      axes_[stick_name] = std::make_unique<JoystickStick>(stick_id, deadzone, sticks_range);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::load_triggers_(const YAML::Node & axes_configuration,
                              const JoystickRemapping & joystick_remapping)
{
  auto triggers_configuration = axes_configuration["triggers"];

  if (triggers_configuration)
  {
    auto triggers_range = extract_range(triggers_configuration);
    auto triggers_mapping = extract_mappings(triggers_configuration);
    for (const auto & [trigger_name, trigger_id] : joystick_remapping.apply(triggers_mapping))
    {
      axes_[trigger_name] = std::make_unique<JoystickTrigger>(trigger_id, triggers_range);
    }
  }
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
  for (const auto & [axe_id, axe] : axes_)
  {
    mapping[axe_id] = axe->getId();
  }

  for (const auto & [button_id, button] : buttons_)
  {
    mapping[button_id] = button->getId();
  }

  return mapping;
}

}  // namespace romea
