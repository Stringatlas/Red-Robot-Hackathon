#pragma once

#include <stdint.h>

namespace Wrist {

// Call after RedRobot::InitServo(). Angles are in degrees from 0 to 180.
bool Init(uint8_t servoIndex, int16_t upAngle, int16_t downAngle,
          int16_t initialAngle);

void Up();
void Down();
void GoToInitial();
bool SetPosition(int16_t angle);
int16_t Position();

} // namespace Wrist
