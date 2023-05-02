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

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// local
#include "romea_joystick_utils/joystick.hpp"

class TestJoystick : public ::testing::Test
{
public:
  TestJoystick()
  : node(nullptr),
    joy(nullptr),
    msg(),
    pub(),
    start(false),
    stop(false),
    start_button_value(-1),
    stop_button_value(-1)
  {
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }


  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_joystick");

    std::map<std::string, int> axes_remapping;
    std::map<std::string, int> buttons_remapping;
    buttons_remapping["start"] = 0;
    buttons_remapping["stop"] = 1;

    joy = std::make_unique<romea::Joystick>(
      node,
      axes_remapping,
      buttons_remapping);

    joy->registerButtonCallback(
      "start",
      romea::JoystickButton::PRESSED,
      std::bind(&TestJoystick::start_callback, this));

    joy->registerButtonCallback(
      "stop",
      romea::JoystickButton::RELEASED,
      std::bind(&TestJoystick::stop_callback, this));

    joy->registerOnReceivedMsgCallback(
      std::bind(&TestJoystick::joystick_callback, this, std::placeholders::_1));

    msg.axes.resize(8);
    msg.buttons.resize(11);
    pub = node->create_publisher<sensor_msgs::msg::Joy>("joystick/joy", 0);
  }

  void publish_joy_msg_and_wait()
  {
    pub->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }

  void joystick_callback(const romea::Joystick & joy)
  {
    start_button_value = joy.getButtonValue("start");
    stop_button_value = joy.getButtonValue("stop");
  }

  void start_callback()
  {
    start = true;
  }

  void stop_callback()
  {
    stop = true;
  }

  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<romea::Joystick> joy;
  sensor_msgs::msg::Joy msg;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> pub;
  bool start;
  bool stop;
  int start_button_value;
  int stop_button_value;
};

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testPressedStartButton)
{
  msg.buttons[0] = 0;
  msg.buttons[1] = 0;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value, 0);
  EXPECT_EQ(stop_button_value, 0);
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  msg.buttons[0] = 1;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value, 1);
  EXPECT_EQ(stop_button_value, 0);
  EXPECT_TRUE(start);
  EXPECT_FALSE(stop);
}

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testReleasedStopButton)
{
  msg.buttons[0] = 0;
  msg.buttons[1] = 1;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value, 0);
  EXPECT_EQ(stop_button_value, 1);
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  msg.buttons[1] = 0;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value, 0);
  EXPECT_EQ(stop_button_value, 0);
  EXPECT_FALSE(start);
  EXPECT_TRUE(stop);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
