// gtest
#include <gtest/gtest.h>

#include "romea_joystick_utils/joystick_trigger.hpp"

class TestTrigger : public ::testing::Test
{
public :

  TestTrigger():
    trigger(nullptr),
    msg()
  {
  }

  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);
  }

  void init(const std::string & joystick_type)
  {
     if (joystick_type == "xbox")
     {
       range.first = 1;
       range.last =- 1;
     } else if (joystick_type == "dualshock4") {
       range.first = 0;
       range.last = 1;
     }

     trigger = std::make_unique<romea::JoystickTrigger>(0, range);
  }

  std::unique_ptr<romea::JoystickTrigger> trigger;
  romea::JoystickAxe::Range range;
  sensor_msgs::msg::Joy msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testXboxUnpressedValue)
{
  init("xbox");
  msg.axes[0] = range.first;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testXboxPressedValue)
{
  init("xbox");
  msg.axes[0] = -0.5;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0.75);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testXboxUnpressedValueAfterHasBeenPressed)
{
  init("xbox");
  msg.axes[0] = range.last;
  trigger->update(msg);
  msg.axes[0] = range.first;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testDualShock4UnpressedValue)
{
  init("dualshock4");
  msg.axes[0] = range.first;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testDualShock4PressedValue)
{
  init("dualshock4");
  msg.axes[0] = 0.5;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0.5);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testDualShock4UnpressedValueAfterHasBeenPressed)
{
  init("dualshock4");
  msg.axes[0] = range.last;
  trigger->update(msg);
  msg.axes[0] = range.first;
  trigger->update(msg);
  EXPECT_DOUBLE_EQ(trigger->getValue(), 0);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
