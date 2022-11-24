#ifndef ROMEA_JOY_JOYSTICK_CONFIGURATION_HPP_
#define ROMEA_JOY_JOYSTICK_CONFIGURATION_HPP_

// yaml
#include <yaml-cpp/yaml.h>

// std
#include <map>
#include <string>

// romea
#include "romea_joystick_utils/joystick_axe.hpp"

namespace romea {

YAML::Node load_configuration(const std::string & joystick_type);

std::map<std::string, int> extract_mappings(const YAML::Node & axes_configuration);

JoystickAxe::Range extract_range(const YAML::Node & axes_configuration);

}  // namespace romea

#endif  // ROMEA_JOY_JOYSTICK_CONFIGURATION_HPP_
