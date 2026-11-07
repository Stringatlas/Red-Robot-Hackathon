#pragma once

#include <stdint.h>

namespace Drive {

// Call after the robot's motors have been initialized.
void Init(bool enableMovement);

void Stop();
void SetTankSpeeds(int16_t leftSpeed, int16_t rightSpeed);
void ResetPid();
void FollowTape(float errorMm);

} // namespace Drive
