// romea
#include "romea_joystick_utils/joystick_configuration.hpp"

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace romea {

//-----------------------------------------------------------------------------
std::map<std::string, int> extract_mappings(const YAML::Node & axes_configuration)
{
  std::map<std::string, int> mappings;
  for (const auto & mapping : axes_configuration["mapping"])
  {
    auto value = mapping.second.as<int>();
    auto key = mapping.first.as<std::string>();
    mappings.insert(std::make_pair(key, value));
  }
  return mappings;
}

//-----------------------------------------------------------------------------
JoystickAxe::Range extract_range(const YAML::Node & axes_configuration)
{
  auto range_configuration = axes_configuration["values"]["range"];
  return { range_configuration[0].as<double>(), range_configuration[1].as<double>()};
}

//-----------------------------------------------------------------------------
YAML::Node load_configuration(const std::string & joystick_type)
{
  std::string romea_joy_share_directory =
    ament_index_cpp::get_package_share_directory("romea_joystick_description");
  std::string joystick_configuration_file =
    romea_joy_share_directory+"/config/"+joystick_type+".yaml";
  return YAML::LoadFile(joystick_configuration_file);
}

}  // namespace romea

