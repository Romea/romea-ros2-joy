//gtest
#include <gtest/gtest.h>

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include "romea_joy/joystick_mapping.hpp"

class TestJoyMapping : public ::testing::Test
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
        rclcpp::NodeOptions no;
        no.arguments(
        {
                        "--ros-args",
                        "--params-file","/home/jeanlaneurit/dev/romea_ros2/src/interfaces/teleoperation/romea_joy/config/xbox.yaml"
                    });
        no.allow_undeclared_parameters(true);
        no.automatically_declare_parameters_from_overrides(true);

        node = std::make_shared<rclcpp::Node>("test_joy_mapping", no);
        node_parameters = std::make_shared<romea::NodeParameters>(node);

    }

    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<romea::NodeParameters> node_parameters;

};


TEST_F(TestJoyMapping, testXboxMapping)
{
  romea::JoystickMapping mapping(node_parameters);

  auto buttons_mappings=mapping.get("buttons");
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

  auto sticks_mappings=mapping.get("axes.sticks");
  EXPECT_EQ(sticks_mappings.at("Horizontal_Left_Stick"),0);
  EXPECT_EQ(sticks_mappings.at("Vertical_Left_Stick"),1);
  EXPECT_EQ(sticks_mappings.at("Horizontal_Right_Stick"),3);
  EXPECT_EQ(sticks_mappings.at("Vertical_Right_Stick"),4);

  auto triggers_mappings=mapping.get("axes.triggers");
  EXPECT_EQ(triggers_mappings.at("LT"),2);
  EXPECT_EQ(triggers_mappings.at("RT"),5);

  auto dpad_mappings=mapping.get("axes.directional_pads");
  EXPECT_EQ(dpad_mappings.at("Horizontal_Directional_Pad"),6);
  EXPECT_EQ(dpad_mappings.at("Vertical_Directional_Pad"),7);
}



int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
