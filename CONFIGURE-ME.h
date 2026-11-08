#pragma once

#include <stdint.h>

const uint8_t TEAM_NUMBER = 8;
static constexpr char TEAM_NAME[] = "Griffin";

namespace RobotConfig {

// Drive
constexpr bool kEnableMovement = true;

// Lift: continuous rotation servo. Positive power raises the lift.
constexpr uint8_t kLiftServoIndex = 3;
constexpr int16_t kLiftNeutralAngle = 90;
constexpr bool kLiftDirectionReversed = false;
constexpr float kLiftFeedforward = 0.0f;

// Claw: standard position servo. Adjust these angles for your mechanism.
constexpr uint8_t kClawServoIndex = 1;
constexpr int16_t kClawOpenAngle = 110;
constexpr int16_t kClawClosedAngle = 145;
constexpr int16_t kClawInitialAngle = 120;

// Wrist: standard position servo. Adjust these angles for your mechanism.
constexpr uint8_t kWristServoIndex = 2;
constexpr int16_t kWristUpAngle = 140;
constexpr int16_t kWristDownAngle = 50;
constexpr int16_t kWristInitialAngle = 50;

} // namespace RobotConfig

/* Uncomment this to receive more verbose debug packets over serial.
   e.g. when motor 2 is driven, you may see
   [DC_MOTOR] Motor 2 | Target Speed: -50 | Duty: 50% | Phase Pin: HIGH
*/
// #define REDROBOT_DEBUG 1 
