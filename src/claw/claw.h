#pragma once

#include <stdint.h>

namespace Claw {

// Call after RedRobot::InitServo(). Angles are in degrees from 0 to 180.
bool Init(uint8_t servoIndex, int16_t openAngle, int16_t closedAngle,
          int16_t initialAngle);

void Open();
void Close();
void GoToInitial();
bool SetPosition(int16_t angle);
int16_t Position();

} // namespace Claw
