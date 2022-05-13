//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//romea
#include "romea_joy/joystick_remapping.hpp"
#include "romea_joy/joystick_configuration.hpp"

class TestJoyConfiguration : public ::testing::Test
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

    void init(const std::string & joystick_type)
    {
        node = std::make_shared<rclcpp::Node>("test_joy_configuration");
        joystick_configuration = romea::load_configuration(joystick_type);
    }

    std::shared_ptr<rclcpp::Node> node;
    YAML::Node joystick_configuration;
};


TEST_F(TestJoyConfiguration, checkXboxConfiguration)
{

  init("xbox");

  auto buttons_mappings = romea::extract_mappings(joystick_configuration["buttons"]);
  EXPECT_EQ(buttons_mappings.at("A"),0);
  EXPECT_EQ(buttons_mappings.at("B"),1);
  EXPECT_EQ(buttons_mappings.at("X"),2);
  EXPECT_EQ(buttons_mappings.at("Y"),3);
  EXPECT_EQ(buttons_mappings.at("LB"),4);
  EXPECT_EQ(buttons_mappings.at("RB"),5);
  EXPECT_EQ(buttons_mappings.at("back"),6);
  EXPECT_EQ(buttons_mappings.at("start"),7);
  EXPECT_EQ(buttons_mappings.at("power"),8);
  EXPECT_EQ(buttons_mappings.at("Left_Stick_Button"),9);
  EXPECT_EQ(buttons_mappings.at("Right_Stick_Button"),10);

  auto sticks_mappings = romea::extract_mappings(joystick_configuration["axes"]["sticks"]);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Left_Stick"),0);
  EXPECT_EQ(sticks_mappings.at("Vertical_Left_Stick"),1);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Right_Stick"),3);
  EXPECT_EQ(sticks_mappings.at("Vertical_Right_Stick"),4);

  auto sticks_range =  romea::extract_range(joystick_configuration["axes"]["sticks"]);
  EXPECT_DOUBLE_EQ(sticks_range.first,-1.);
  EXPECT_DOUBLE_EQ(sticks_range.last,1);

  auto triggers_mappings = romea::extract_mappings(joystick_configuration["axes"]["triggers"]);
  EXPECT_EQ(triggers_mappings.at("LT"),2);
  EXPECT_EQ(triggers_mappings.at("RT"),5);

  auto trigger_range =  romea::extract_range(joystick_configuration["axes"]["triggers"]);
  EXPECT_DOUBLE_EQ(trigger_range.first,1.);
  EXPECT_DOUBLE_EQ(trigger_range.last,-1);


  auto dpad_mappings = romea::extract_mappings(joystick_configuration["axes"]["directional_pads"]);
  EXPECT_EQ(dpad_mappings.at("Horizontal_Directional_Pad"),6);
  EXPECT_EQ(dpad_mappings.at("Vertical_Directional_Pad"),7);

  auto dpad_range =  romea::extract_range(joystick_configuration["axes"]["directional_pads"]);
  EXPECT_DOUBLE_EQ(dpad_range.first,-1.);
  EXPECT_DOUBLE_EQ(dpad_range.last,1);

}

TEST_F(TestJoyConfiguration, checkDualShock4Configuration)
{
  init("dualshock4");

  auto buttons_mappings = romea::extract_mappings(joystick_configuration["buttons"]);
  EXPECT_EQ(buttons_mappings.at("Cross"), 3);
  EXPECT_EQ(buttons_mappings.at("Circle"),  2);
  EXPECT_EQ(buttons_mappings.at("Square"),  0);
  EXPECT_EQ(buttons_mappings.at("Triangle"),  1);
  EXPECT_EQ(buttons_mappings.at("L1"),  4);
  EXPECT_EQ(buttons_mappings.at("L2_Button"),  5);
  EXPECT_EQ(buttons_mappings.at("R1"),  6);
  EXPECT_EQ(buttons_mappings.at("R2_Button"),  7);
  EXPECT_EQ(buttons_mappings.at("share"),  8);
  EXPECT_EQ(buttons_mappings.at("option"),  9);
  EXPECT_EQ(buttons_mappings.at("PS_Button"),  10);
  EXPECT_EQ(buttons_mappings.at("touchpad"),  11);
  EXPECT_EQ(buttons_mappings.at("Left_Stick_Button"),  12);
  EXPECT_EQ(buttons_mappings.at("Right_Stick_Button"),  13);
  EXPECT_EQ(buttons_mappings.at("Left_Directional_Pad"),  14);
  EXPECT_EQ(buttons_mappings.at("Up_Directional_Pad"),  15);
  EXPECT_EQ(buttons_mappings.at("Right_Directional_Pad"),  16);
  EXPECT_EQ(buttons_mappings.at("Down_Directional_Pad"),  17);

  auto sticks_mappings = romea::extract_mappings(joystick_configuration["axes"]["sticks"]);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Left_Stick"), 0);
  EXPECT_EQ(sticks_mappings.at("Vertical_Left_Stick"), 1);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Right_Stick"), 2);
  EXPECT_EQ(sticks_mappings.at("Vertical_Right_Stick"), 3);

  auto sticks_range =  romea::extract_range(joystick_configuration["axes"]["sticks"]);
  EXPECT_DOUBLE_EQ(sticks_range.first,-1.);
  EXPECT_DOUBLE_EQ(sticks_range.last,1);

  auto triggers_mappings = romea::extract_mappings(joystick_configuration["axes"]["triggers"]);
  EXPECT_EQ(triggers_mappings.at("L2"),4);
  EXPECT_EQ(triggers_mappings.at("R2"),5);

  auto trigger_range =  romea::extract_range(joystick_configuration["axes"]["triggers"]);
  EXPECT_DOUBLE_EQ(trigger_range.first,0.);
  EXPECT_DOUBLE_EQ(trigger_range.last,1.);

}


int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
