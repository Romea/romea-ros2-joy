// gtest
#include <gtest/gtest.h>

#include "romea_joy/joystick_button.hpp"

class TestButton : public ::testing::Test
{
public :

  TestButton():
    button(0),
    msg(),
    pressed_counter(0),
    released_counter(0),
    toggled_counter(0)
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


  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);

    button.registerCallback(romea::JoystickButton::PRESSED,
                            std::bind(&TestButton::pressed_callback,this));

    button.registerCallback(romea::JoystickButton::RELEASED,
                            std::bind(&TestButton::released_callback,this));

    button.registerCallback(romea::JoystickButton::TOGGLED,
                            std::bind(&TestButton::toggled_callback,this));

  }

  romea::JoystickButton button;
  sensor_msgs::msg::Joy msg;
  int pressed_counter;
  int released_counter;
  int toggled_counter;

};

//-----------------------------------------------------------------------------
TEST_F(TestButton, testPressed)
{
  msg.buttons[0]=0;
  button.update(msg);
  EXPECT_EQ(pressed_counter,0);
  EXPECT_EQ(released_counter,0);
  EXPECT_EQ(toggled_counter,0);

  msg.buttons[0]=1;
  button.update(msg);

  EXPECT_EQ(pressed_counter,1);
  EXPECT_EQ(released_counter,0);
  EXPECT_EQ(toggled_counter,1);
}

//-----------------------------------------------------------------------------
TEST_F(TestButton, testReleased)
{
  msg.buttons[0]=1;
  button.update(msg);
  EXPECT_EQ(pressed_counter,0);
  EXPECT_EQ(released_counter,0);
  EXPECT_EQ(toggled_counter,0);

  msg.buttons[0]=0;
  button.update(msg);

  EXPECT_EQ(pressed_counter,0);
  EXPECT_EQ(released_counter,1);
  EXPECT_EQ(toggled_counter,1);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
