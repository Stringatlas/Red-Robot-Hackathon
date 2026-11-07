#pragma once

#include <stdint.h>

const uint8_t TEAM_NUMBER = 8;
static constexpr char TEAM_NAME[] = "Griffin";

namespace RobotConfig {

// Drive
constexpr bool kEnableMovement = true;

// Lift: continuous rotation servo. Positive power raises the lift.
constexpr uint8_t kLiftServoIndex = 3;
constexpr bool kLiftDirectionReversed = false;
constexpr float kLiftFeedforward = 0.0f;

// Claw: standard position servo. Adjust these angles for your mechanism.
constexpr uint8_t kClawServoIndex = 1;
constexpr int16_t kClawOpenAngle = 0;
constexpr int16_t kClawClosedAngle = 180;
constexpr int16_t kClawInitialAngle = 90;

// Wrist: standard position servo. Adjust these angles for your mechanism.
constexpr uint8_t kWristServoIndex = 2;
constexpr int16_t kWristUpAngle = 0;
constexpr int16_t kWristDownAngle = 180;
constexpr int16_t kWristInitialAngle = 90;

} // namespace RobotConfig

/* Uncomment this to receive more verbose debug packets over serial.
   e.g. when motor 2 is driven, you may see
   [DC_MOTOR] Motor 2 | Target Speed: -50 | Duty: 50% | Phase Pin: HIGH
*/
// #define REDROBOT_DEBUG 1 
