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

// gtest
#include "gtest/gtest.h"

// local
#include "romea_joystick_utils/joystick_button.hpp"

class TestButton : public ::testing::Test
{
public:
  TestButton()
  : button(0),
    msg(),
    pressed_counter(0),
    released_counter(0),
    toggled_counter(0),
    held_counter(0),
    unheld_counter(0),
    double_pressed_counter(0)
  {
  }

  void pressed_callback()
  {
    pressed_counter++;
  }

  void released_callback()
  {
    released_counter++;
  }

  void toggled_callback()
  {
    toggled_counter++;
  }

  void held_callback()
  {
    held_counter++;
  }

  void unheld_callback()
  {
    unheld_counter++;
  }

  void double_pressed_callback()
  {
    double_pressed_counter++;
  }


  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);

    button.registerCallback(
      romea::ros2::JoystickButton::PRESSED,
      std::bind(&TestButton::pressed_callback, this));

    button.registerCallback(
      romea::ros2::JoystickButton::RELEASED,
      std::bind(&TestButton::released_callback, this));

    button.registerCallback(
      romea::ros2::JoystickButton::TOGGLED,
      std::bind(&TestButton::toggled_callback, this));

    button.registerCallback(
      romea::ros2::JoystickButton::HELD,
      std::bind(&TestButton::held_callback, this));

    button.registerCallback(
      romea::ros2::JoystickButton::UNHELD,
      std::bind(&TestButton::unheld_callback, this));

    button.registerCallback(
      romea::ros2::JoystickButton::DOUBLE_PRESSED,
      std::bind(&TestButton::double_pressed_callback, this));
  }

  void check_counters(
    const int & expected_pressed_counter,
    const int & expected_released_counter,
    const int & expected_toggled_counter,
    const int & expected_held_counter,
    const int & expected_unheld_counter,
    const int & expected_double_pressed_counter)
  {
    EXPECT_EQ(pressed_counter, expected_pressed_counter);
    EXPECT_EQ(released_counter, expected_released_counter);
    EXPECT_EQ(toggled_counter, expected_toggled_counter);
    EXPECT_EQ(held_counter, expected_held_counter);
    EXPECT_EQ(unheld_counter, expected_unheld_counter);
    EXPECT_EQ(double_pressed_counter, expected_double_pressed_counter);
  }

  romea::ros2::JoystickButton button;
  sensor_msgs::msg::Joy msg;
  int pressed_counter;
  int released_counter;
  int toggled_counter;
  int held_counter;
  int unheld_counter;
  int double_pressed_counter;
};

//-----------------------------------------------------------------------------
TEST_F(TestButton, testPressed)
{
  msg.buttons[0] = 0;
  button.update(msg);
  check_counters(0, 0, 0, 0, 0, 0);

  msg.buttons[0] = 1;
  msg.header.stamp.sec = 1;
  button.update(msg);
  check_counters(1, 0, 1, 0, 0, 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestButton, testReleased)
{
  msg.buttons[0] = 1;
  button.update(msg);
  check_counters(0, 0, 0, 0, 0, 0);

  msg.buttons[0] = 0;
  button.update(msg);
  check_counters(0, 1, 1, 0, 0, 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestButton, testHeldUnheld)
{
  msg.buttons[0] = 1;

  for (size_t n = 1; n < 10; ++n) {
    msg.header.stamp.sec++;
    button.update(msg);
    check_counters(0, 0, 0, 0, 0, 0);
  }
  msg.buttons[0] = 1;

  for (size_t n = 1; n < 10; ++n) {
    msg.header.stamp.sec++;
    button.update(msg);
    check_counters(0, 0, 0, 1, 0, 0);
  }

  msg.buttons[0] = 0;
  msg.header.stamp.sec++;
  button.update(msg);
  check_counters(0, 1, 1, 1, 1, 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestButton, testDoublePress)
{
  msg.header.stamp.nanosec = 0;
  msg.buttons[0] = 0;
  button.update(msg);
  check_counters(0, 0, 0, 0, 0, 0);

  msg.header.stamp.nanosec = 300000000;
  msg.buttons[0] = 1;
  button.update(msg);
  check_counters(1, 0, 1, 0, 0, 0);

  msg.header.stamp.nanosec = 400000000;
  msg.buttons[0] = 0;
  button.update(msg);
  check_counters(1, 1, 2, 0, 0, 0);

  msg.header.stamp.nanosec = 500000000;
  msg.buttons[0] = 1;
  button.update(msg);
  check_counters(2, 1, 3, 0, 0, 1);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
