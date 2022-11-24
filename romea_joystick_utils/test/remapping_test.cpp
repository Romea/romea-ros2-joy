// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/rclcpp.hpp>

// romea
#include "romea_joystick_utils/joystick_remapping.hpp"
#include "romea_joystick_utils/joystick_configuration.hpp"


class TestJoyRemapping : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_joy_configuration");
    joystick_configuration = romea::load_configuration("xbox");
  }

  std::shared_ptr<rclcpp::Node> node;
  YAML::Node joystick_configuration;
};

TEST_F(TestJoyRemapping, checkPartialRemapping)
{
  std::map<std::string, std::string> joystick_remapping;
  joystick_remapping["stop"]="B";
  joystick_remapping["speed"]="RT";
  joystick_remapping["steering"]="Horizontal_Left_Stick";
  joystick_remapping["trigger"]="Horizontal_Directional_Pad";


  romea::JoystickRemapping remapping(joystick_remapping, false);

  auto buttons_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["buttons"]));
  EXPECT_EQ(buttons_mappings.at("A"), 0);
  EXPECT_EQ(buttons_mappings.at("stop"), 1);
  EXPECT_EQ(buttons_mappings.at("X"), 2);
  EXPECT_EQ(buttons_mappings.at("Y"), 3);
  EXPECT_EQ(buttons_mappings.at("LB"), 4);
  EXPECT_EQ(buttons_mappings.at("RB"), 5);
  EXPECT_EQ(buttons_mappings.at("back"), 6);
  EXPECT_EQ(buttons_mappings.at("start"), 7);
  EXPECT_EQ(buttons_mappings.at("power"), 8);
  EXPECT_EQ(buttons_mappings.at("Left_Stick_Button"), 9);
  EXPECT_EQ(buttons_mappings.at("Right_Stick_Button"), 10);
  EXPECT_FALSE(remapping.is_complete());

  auto sticks_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["sticks"]));
  EXPECT_EQ(sticks_mappings.at("steering"), 0);
  EXPECT_EQ(sticks_mappings.at("Vertical_Left_Stick"), 1);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Right_Stick"), 3);
  EXPECT_EQ(sticks_mappings.at("Vertical_Right_Stick"), 4);
  EXPECT_FALSE(remapping.is_complete());

  auto triggers_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["triggers"]));
  EXPECT_EQ(triggers_mappings.at("speed"), 5);
  EXPECT_EQ(triggers_mappings.at("LT"), 2);
  EXPECT_FALSE(remapping.is_complete());

  auto dpad_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["directional_pads"]));
  EXPECT_EQ(dpad_mappings.at("trigger"), 6);
  EXPECT_EQ(dpad_mappings.at("Vertical_Directional_Pad"), 7);

  EXPECT_TRUE(remapping.is_complete());
}

TEST_F(TestJoyRemapping, checkRemappingWhenKeepOnlyRemappedIsEnable)
{
  std::map<std::string, std::string> joystick_remapping;
  joystick_remapping["stop"]="B";
  joystick_remapping["speed"]="RT";
  joystick_remapping["steering"]="Horizontal_Left_Stick";
  joystick_remapping["trigger"]="Horizontal_Directional_Pad";

  romea::JoystickRemapping remapping(joystick_remapping, true);

  auto buttons_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["buttons"]));
  EXPECT_THROW(buttons_mappings.at("A"), std::out_of_range);
  EXPECT_EQ(buttons_mappings.at("stop"), 1);

  auto sticks_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["sticks"]));
  EXPECT_THROW(sticks_mappings.at("Vertical_Left_Stick"), std::out_of_range);
  EXPECT_EQ(sticks_mappings.at("steering"), 0);

  auto triggers_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["triggers"]));
  EXPECT_THROW(triggers_mappings.at("LT"), std::out_of_range);
  EXPECT_EQ(triggers_mappings.at("speed"), 5);

  auto dpad_mappings = remapping.apply(
    romea::extract_mappings(joystick_configuration["axes"]["directional_pads"]));
  EXPECT_THROW(dpad_mappings.at("Vertical_Directional_Pad"), std::out_of_range);
  EXPECT_EQ(dpad_mappings.at("trigger"), 6);

  EXPECT_TRUE(remapping.is_complete());
}

TEST_F(TestJoyRemapping, checkMissingRemap)
{
  std::map<std::string, std::string> joystick_remapping;
  joystick_remapping["stop"] = "Triangle";

  romea::JoystickRemapping remapping(joystick_remapping, true);
  remapping.apply(romea::extract_mappings(joystick_configuration["buttons"]));

  EXPECT_FALSE(remapping.is_complete());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return  RUN_ALL_TESTS();
}
