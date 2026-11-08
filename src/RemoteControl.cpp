#include "../RedRobot.h"
#include "../CONFIGURE-ME.h"
#include "autonomous/autonomous.h"
#include "claw/claw.h"
#include "lift/lift.h"
#include "wrist/wrist.h"

bool useTankDrive = false; // set true to use separate joysticks for left/right motors

namespace Bluefruit
{
  enum class CalibrationTarget { Claw, Wrist };
  CalibrationTarget calibrationTarget = CalibrationTarget::Claw;
  bool calibrationInitialized = false;

  void PrintCalibrationTelemetry()
  {
    Serial.printf("[Bluefruit calibration] selected=%s claw=%d deg wrist=%d deg\n",
                  calibrationTarget == CalibrationTarget::Claw ? "claw" : "wrist",
                  Claw::Position(), Wrist::Position());
  }

  void Button1_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown || calibrationInitialized) return;

    const bool clawReady = Claw::Init(RobotConfig::kClawServoIndex,
                                      RobotConfig::kClawOpenAngle,
                                      RobotConfig::kClawClosedAngle,
                                      RobotConfig::kClawInitialAngle);
    const bool wristReady = Wrist::Init(RobotConfig::kWristServoIndex,
                                        RobotConfig::kWristUpAngle,
                                        RobotConfig::kWristDownAngle,
                                        RobotConfig::kWristInitialAngle);
    calibrationInitialized = clawReady && wristReady;
    if (calibrationInitialized)
      PrintCalibrationTelemetry();
    else
      Serial.println("[Bluefruit calibration] servo initialization failed");
  }

  void Button2_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown || !calibrationInitialized) return;
    calibrationTarget = CalibrationTarget::Claw;
    PrintCalibrationTelemetry();
  }

  void Button3_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown || !calibrationInitialized) return;
    calibrationTarget = CalibrationTarget::Wrist;
    PrintCalibrationTelemetry();
  }

  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!buttonPressedDown || !calibrationInitialized) return;

    const int16_t step = (Button == UP || Button == RIGHT) ? 5 : -5;
    const int16_t current = calibrationTarget == CalibrationTarget::Claw
                                ? Claw::Position() : Wrist::Position();
    const int16_t next = constrain(current + step, 0, 180);
    if (calibrationTarget == CalibrationTarget::Claw)
      Claw::SetPosition(next);
    else
      Wrist::SetPosition(next);

    PrintCalibrationTelemetry();
  }

  void Button4_Pressed(bool buttonPressedDown)
  {
  }
}

namespace RR_Controller
{
  enum class ClawPosition { Initial, Open, Closed };
  enum class WristPosition { Initial, Up, Down };

  ClawPosition clawPosition = ClawPosition::Initial;
  ClawPosition lastClawPosition = ClawPosition::Initial;
  WristPosition wristPosition = WristPosition::Initial;
  WristPosition lastWristPosition = WristPosition::Initial;

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

  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;

    if (Button == UP && !RedRobot::IsRemoteControlled()) {
      Autonomous::RequestStart();
    } else if (Button == LEFT && !RedRobot::IsRemoteControlled()) {
      Autonomous::RequestDriverControl();
    }
  }

  void ButtonX_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;

    if (lastClawPosition == ClawPosition::Open) {
      Claw::Close();
      clawPosition = ClawPosition::Closed;
    } else {
      Claw::Open();
      clawPosition = ClawPosition::Open;
    }
    lastClawPosition = clawPosition;
  }

  void ButtonY_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;

    if (lastWristPosition == WristPosition::Up) {
      Wrist::Down();
      wristPosition = WristPosition::Down;
    } else {
      Wrist::Up();
      wristPosition = WristPosition::Up;
    }
    lastWristPosition = wristPosition;
  }

  void ButtonB_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;

    if (clawPosition != ClawPosition::Initial) {
      Claw::GoToInitial();
      clawPosition = ClawPosition::Initial;
    } else if (lastClawPosition == ClawPosition::Open) {
      Claw::Open();
      clawPosition = ClawPosition::Open;
    } else if (lastClawPosition == ClawPosition::Closed) {
      Claw::Close();
      clawPosition = ClawPosition::Closed;
    }
  }

  void ButtonA_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;

    if (wristPosition != WristPosition::Initial) {
      Wrist::GoToInitial();
      wristPosition = WristPosition::Initial;
    } else if (lastWristPosition == WristPosition::Up) {
      Wrist::Up();
      wristPosition = WristPosition::Up;
    } else if (lastWristPosition == WristPosition::Down) {
      Wrist::Down();
      wristPosition = WristPosition::Down;
    }
  }
}
