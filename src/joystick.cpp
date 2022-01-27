#include "romea_joy/joystick.hpp"
#include "romea_joy/joystick_stick.hpp"
#include "romea_joy/joystick_trigger.hpp"
#include "romea_joy/joystick_directional_pad.hpp"
#include "romea_joy/joystick_mapping.hpp"

namespace romea
{
  JoystickStick::Config loadSticksConfiguration(NodeParameters & node_parameters,
                                                const std::string & param_name)
  {
    std::vector<double> interval = node_parameters.loadVector<double>(param_name);
    if(interval.size()!=2)
    {
      throw(std::runtime_error("Unable load sticks configuration from ros parameter " + param_name));
    }
    return {interval[0],interval[1]};
  }

  JoystickTrigger::Config loadTriggersConfiguration(NodeParameters & node_parameters,
                                                    const std::string & param_name)
  {
    std::vector<double> interval = node_parameters.loadVector<double>(param_name);
    if(interval.size()!=2)
    {
      throw(std::runtime_error("Unable load triggers configuration from ros parameter " + param_name));
    }
    return {interval[0],interval[1]};
  }

}

namespace romea
{

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const std::string & ns):
  Joystick(node,
           std::map<std::string,std::string>(),
           false,
           ns)
{
}

//-----------------------------------------------------------------------------
Joystick::Joystick(std::shared_ptr<rclcpp::Node> node,
                   const Remappings & name_remappings,
                   bool use_only_remapped,
                   const std::string & ns):
  node_(node),
  joy_sub_(),
  buttons_(),
  axes_(),
  sticks_configuration_(),
  triggers_configuration_()
{
  JoystickMapping joystick_mapping(name_remappings,use_only_remapped);

  auto node_parameters = NodeParameters(node,ns);
  addButtons_(node_parameters,joystick_mapping);
  addDirectionalPads_(node_parameters,joystick_mapping);
  addSticks_(node_parameters,joystick_mapping);
  addTriggers_(node_parameters,joystick_mapping);

  auto callback = std::bind(&Joystick::processJoyMsg_,this,std::placeholders::_1);
  joy_sub_=node->create_subscription<sensor_msgs::msg::Joy>("joy", 1,callback);
}

//-----------------------------------------------------------------------------
const JoystickStick::Config & Joystick::getSticksConfiguration()const
{
  return sticks_configuration_;
}

//-----------------------------------------------------------------------------
const JoystickTrigger::Config & Joystick::getTriggersConfiguration()const
{
  return triggers_configuration_;
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
void Joystick::addButtons_(NodeParameters & node_parameters,
                           const JoystickMapping & joystick_mapping)
{

  std::cout << " addButtons_ " << std::endl;

  auto mapping = node_parameters.loadMap<int>("buttons.mapping");

  for(const auto & [buttom_name, button_id] : joystick_mapping.get(mapping))
  {
    buttons_[buttom_name]=std::make_unique<JoystickButton>(button_id);
  }

}

//-----------------------------------------------------------------------------
void Joystick::addDirectionalPads_(NodeParameters & node_parameters,
                                   const JoystickMapping & joystick_mapping)
{
  std::cout << " addDirectionalPads_ " << std::endl;

  if(node_parameters.hasParam("axes.directional_pads"))
  {
    std::cout << " addDirectionalPads_ " << std::endl;
    auto mapping = node_parameters.loadMap<int>("axes.directional_pads.mapping");

    for(const auto & [axe_name,axe_id] : joystick_mapping.get(mapping))
    {
      axes_[axe_name]=std::make_unique<JoystickDirectionalPad>(axe_id);
    }

  }
}
//-----------------------------------------------------------------------------
void Joystick::addSticks_(NodeParameters & node_parameters,
                          const JoystickMapping & joystick_mapping)
{
  double deadzone = node_parameters.loadParam<double>("deadzone");

  sticks_configuration_=loadSticksConfiguration(node_parameters,"axes.sticks.scale");

  auto mapping = node_parameters.loadMap<int>("axes.sticks.mapping");

  for(const auto & [stick_name, stick_id] : joystick_mapping.get(mapping))
  {
    axes_[stick_name]=std::make_unique<JoystickStick>(stick_id,deadzone);
  }
}
//-----------------------------------------------------------------------------
void Joystick::addTriggers_(NodeParameters & node_parameters,
                            const JoystickMapping & joystick_mapping)
{

  if(node_parameters.hasParam("axes.triggers"))
  {

    triggers_configuration_=loadTriggersConfiguration(node_parameters,"axes.triggers.scale");

    std::map<std::string,int> mapping = node_parameters.loadMap<int>("axes.triggers.mapping");

    for(const auto & [trigger_name,trigger_id] : joystick_mapping.get(mapping))
    {
      axes_[trigger_name]=std::make_unique<JoystickTrigger>(trigger_id,triggers_configuration_.unpressed_value);
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
