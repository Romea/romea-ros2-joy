#ifndef _JoystickConfiguration_HPP
#define _JoystickConfiguration_HPP

//std
#include <map>
#include <string>

//romea
#include "joystick_axe.hpp"

//yaml
#include <yaml-cpp/yaml.h>

namespace romea {

YAML::Node load_configuration(const std::string & joystick_type);

std::map<std::string,int> extract_mappings(const YAML::Node & axes_configuration);

JoystickAxe::Range extract_range(const YAML::Node & axes_configuration);

}

#endif
