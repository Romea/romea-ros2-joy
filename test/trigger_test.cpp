// gtest
#include <gtest/gtest.h>

#include "romea_joy/joystick_trigger.hpp"

class TestTrigger : public ::testing::Test
{
public :

  TestTrigger():
    trigger(0,1),
    msg()
  {

  }

  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);
  }

  romea::JoystickTrigger trigger;
  sensor_msgs::msg::Joy msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testUnpressedValue)
{
  msg.axes[0]=0;

  trigger.update(msg);

  EXPECT_DOUBLE_EQ(trigger.getValue(),1);
}

//-----------------------------------------------------------------------------
TEST_F(TestTrigger, testPressedValue)
{
  msg.axes[0]=0.5;

  trigger.update(msg);

  EXPECT_DOUBLE_EQ(trigger.getValue(),0.5);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
