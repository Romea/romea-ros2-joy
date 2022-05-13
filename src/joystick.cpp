//romea
#include "romea_joy/joystick.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const std::string & joystick_type):
  Joystick(node,
           joystick_type,
           std::map<std::string,std::string>(),
           false)
{
}

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const std::string & joystick_type,
                   const Remappings & name_remappings,
                   bool use_only_remapped):
  node_(node),
  joy_sub_(),
  buttons_(),
  axes_()
{
  YAML::Node joystick_configuration = load_configuration(joystick_type);
  JoystickRemapping joystick_remapping(name_remappings,use_only_remapped);

  addButtons_(joystick_configuration,joystick_remapping);
  addDirectionalPads_(joystick_configuration,joystick_remapping);
  addSticks_(joystick_configuration,joystick_remapping);
  addTriggers_(joystick_configuration,joystick_remapping);

  auto callback = std::bind(&Joystick::processJoyMsg_,this,std::placeholders::_1);
  joy_sub_=node->create_subscription<sensor_msgs::msg::Joy>("joy", 1,callback);
}

//-----------------------------------------------------------------------------
void Joystick::processJoyMsg_(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{

  if(msg->axes.empty() || msg->buttons.empty())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"Unavailable joy msg : check if joy node is connected to a proper device");
  }
  else
  {

    for(auto & p :buttons_)
    {
      p.second->update(*msg);
    }

    for(auto & p : axes_)
    {
      p.second->update(*msg);
    }

    if(on_received_msg_callback_)
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
  if(it == buttons_.end())
  {
    std::stringstream ss;
    ss << "No joystick button called ";
    ss << button_name;
    ss << " has been found, register ";
    ss << button_name;
    ss << " callback failed. Check joystick buttons mappings or remappings ";
    throw(std::runtime_error(ss.str()));
  }

  it->second->registerCallback(event_type,std::move(callback));
}

//-----------------------------------------------------------------------------
void Joystick::registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback)
{
  on_received_msg_callback_=callback;
}


//-----------------------------------------------------------------------------
void Joystick::addButtons_(const YAML::Node & joystick_configuration,
                           const JoystickRemapping & joystick_remapping)
{


  YAML::Node buttons_configuration = joystick_configuration["buttons"];

  auto buttons_mapping = extract_mappings(buttons_configuration);
  for(const auto & [buttom_name, button_id] : joystick_remapping.apply(buttons_mapping))
  {
    buttons_[buttom_name]=std::make_unique<JoystickButton>(button_id);
  }

}

//-----------------------------------------------------------------------------
void Joystick::addDirectionalPads_(const YAML::Node & joystick_configuration,
                                   const JoystickRemapping & joystick_remapping)
{

  auto dpads_configuration= joystick_configuration["axes"]["directional_pads"];

  if(dpads_configuration)
  {
    auto dpads_range = extract_range(dpads_configuration);
    auto dpads_mapping = extract_mappings(dpads_configuration);
    for(const auto & [dpad_name,dpad_id] : joystick_remapping.apply(dpads_mapping))
    {
      axes_[dpad_name]=std::make_unique<JoystickDirectionalPad>(dpad_id,dpads_range);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::addSticks_(const YAML::Node & joystick_configuration,
                          const JoystickRemapping & joystick_remapping)

{
  auto sticks_configuration = joystick_configuration["axes"]["sticks"];

  if(sticks_configuration)
  {
    auto sticks_range = extract_range(sticks_configuration);
    auto sticks_mapping = extract_mappings(sticks_configuration);
    double deadzone = sticks_configuration["deadzone"].as<double>();
    for(const auto & [stick_name, stick_id] : joystick_remapping.apply(sticks_mapping))
    {
      axes_[stick_name]=std::make_unique<JoystickStick>(stick_id,deadzone,sticks_range);
    }
  }
}

//-----------------------------------------------------------------------------
void Joystick::addTriggers_(const YAML::Node & joystick_configuration,
                            const JoystickRemapping & joystick_remapping)
{

  auto triggers_configuration = joystick_configuration["axes"]["triggers"];

  if(triggers_configuration)
  {
    auto triggers_range = extract_range(triggers_configuration);
    auto triggers_mapping = extract_mappings(triggers_configuration);
    for(const auto & [trigger_name,trigger_id] : joystick_remapping.apply(triggers_mapping))
    {
      axes_[trigger_name]=std::make_unique<JoystickTrigger>(trigger_id,triggers_range);
    }
  }
}

//-----------------------------------------------------------------------------
const int & Joystick::getButtonValue(const std::string & button_name)const
{
  return buttons_.at(button_name)->getValue();
}

//-----------------------------------------------------------------------------
const double & Joystick::getAxisValue(const std::string & axis_name)const
{
  return axes_.at(axis_name)->getValue();
}


}
