// gtest
#include <gtest/gtest.h>

#include "romea_joystick_utils/joystick_stick.hpp"

class TestStick : public ::testing::Test
{
public :

  TestStick():
    stick1(0, 0.05, {-1, 1}),
    stick2(1, 0.05, {-1, 1}),
    msg()
  {
  }

  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);
  }

  romea::JoystickStick stick1;
  romea::JoystickStick stick2;
  sensor_msgs::msg::Joy msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestStick, testInsideDeadZone)
{
  msg.axes[0] = 0.05;
  msg.axes[1] = -0.05;

  stick1.update(msg);
  EXPECT_NEAR(stick1.getValue(), 0., 0.001);

  stick2.update(msg);
  EXPECT_NEAR(stick1.getValue(), 0., 0.001);
}

//-----------------------------------------------------------------------------
TEST_F(TestStick, testMax)
{
  msg.axes[0] = 1;
  msg.axes[1] = -1;

  stick1.update(msg);
  EXPECT_DOUBLE_EQ(stick1.getValue(), 1.);

  stick2.update(msg);
  EXPECT_DOUBLE_EQ(stick1.getValue(), 1.);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
