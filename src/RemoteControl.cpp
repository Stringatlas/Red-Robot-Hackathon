#include "../RedRobot.h"
#include "autonomous/autonomous.h"
#include "lift/lift.h"

bool useTankDrive = false; // set true to use separate joysticks for left/right motors

namespace Bluefruit
{
  void Button1_Pressed(bool buttonPressedDown)
  {
    // Button 1 starts the 20-second autonomous period from idle.
    if (buttonPressedDown && !RedRobot::IsRemoteControlled())
      Autonomous::RequestStart();
  }

  void Button2_Pressed(bool buttonPressedDown)
  {
  }

  void Button3_Pressed(bool buttonPressedDown)
  {
  }

  void Button4_Pressed(bool buttonPressedDown)
  {
  }
}

namespace RR_Controller
{
  void Joystick1(int8_t x, int8_t y)
  {
    if (!RedRobot::IsRemoteControlled())
      return;

    if (useTankDrive)
      RedRobot::WriteDCMotor(2, y);
    else
    {
      int leftMotor = y + x;
      int rightMotor = y - x;

      // Normalize to [-100, 100]
      int maxMagnitude = max(abs(leftMotor), abs(rightMotor));

      if (maxMagnitude > 100)
      {
        leftMotor = leftMotor * 75 / maxMagnitude;
        rightMotor = rightMotor * 75 / maxMagnitude;
      }
      RedRobot::WriteDCMotor(2, leftMotor);
      RedRobot::WriteDCMotor(3, rightMotor);
    }
  }

  void Joystick2(int8_t x, int8_t y)
  {
    // Joystick Y is roughly -100 to 100; zero keeps the gravity feedforward.
    const float liftPower = constrain(y / 100.0f, -1.0f, 1.0f);
    Lift::SetPower(liftPower);
    Serial.printf("[RR joystick 2] lift power %.2f\n", liftPower);

    if (!RedRobot::IsRemoteControlled())
      return;

    if (useTankDrive)
      RedRobot::WriteDCMotor(3, y);
  }

  void ButtonX_Pressed(bool buttonPressedDown)
  {
    // Manual driver takeover before autonomous completes.
    if (buttonPressedDown && !RedRobot::IsRemoteControlled())
      RedRobot::StartRemoteControl();
  }

  void ButtonY_Pressed(bool buttonPressedDown)
  {
    // The state machine accepts this request only while idle.
    if (buttonPressedDown && !RedRobot::IsRemoteControlled())
      Autonomous::RequestStart();
  }

  void ButtonB_Pressed(bool buttonPressedDown)
  {
  }

  void ButtonA_Pressed(bool buttonPressedDown)
  {
  }
}
