/**
 * @brief Red Robot 2026 main code!
 */

#include <Arduino.h> // keep this first
#include "CONFIGURE-ME.h"
#include "RedRobot.h"
#include "src/autonomous/autonomous.h"
#include "src/claw/claw.h"
#include "src/drive/drive.h"
#include "src/lift/lift.h"
#include "src/tape/tape.h"
#include "src/wrist/wrist.h"

void setup() {
  RedRobot::InitRedRobot();
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  Lift::Init(RobotConfig::kLiftServoIndex,
             RobotConfig::kLiftDirectionReversed,
             RobotConfig::kLiftFeedforward);
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();
  Tape::Init();
  Drive::Init(RobotConfig::kEnableMovement);
  Autonomous::Init();
  Serial.println("Initialized.");
}

void AutonomousLoop() {
  Autonomous::Update();
}
