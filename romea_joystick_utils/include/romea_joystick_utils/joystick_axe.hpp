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


#ifndef ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_
#define ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_

// std
#include <memory>

// ros
#include "sensor_msgs/msg/joy.hpp"

// romea core
#include "romea_core_common/math/Interval.hpp"


namespace romea
{

class JoystickAxe
{
public:
  using Ptr = std::unique_ptr<JoystickAxe>;

public:
  explicit JoystickAxe(const int & axis_id);

  virtual void update(const sensor_msgs::msg::Joy & joy_msg);

  const int & getId()const;

  const double & getValue()const;

protected:
  int id_;
  double value_;
};

}  // namespace romea

#endif  // ROMEA_JOYSTICK_UTILS__JOYSTICK_AXE_HPP_
