/**
 * @brief Red Robot 2026 main code!
 */

#include <Arduino.h> // keep this first
#include "RedRobot.h"
#include "src/autonomous/autonomous.h"
#include "src/drive/drive.h"
#include "src/lift/lift.h"
#include "src/tape/tape.h"

// Set false to inspect tape readings without moving the robot.
constexpr bool kEnableMovement = true;
constexpr uint8_t kLiftServoIndex = 0;
constexpr bool kLiftDirectionReversed = false;
constexpr float kLiftFeedforward = 0.10f;

void setup() {
  RedRobot::InitRedRobot();
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  Lift::Init(kLiftServoIndex, kLiftDirectionReversed, kLiftFeedforward);
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();
  Tape::Init();
  Drive::Init(kEnableMovement);
  Autonomous::Init();
  Serial.println("Initialized.");
}

void AutonomousLoop() {
  Autonomous::Update();
}
