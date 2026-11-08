#pragma once

#include <stdint.h>

namespace Lift {

// Call after RedRobot::InitServo(). Returns false for an invalid servo index.
// Feedforward is upward power in [0, 1]; tune it to hold the lift against gravity.
// neutralAngle is the stationary RedRobot::SetServo value in [-180, 180].
bool Init(uint8_t servoIndex, bool directionReversed = false,
          float feedforward = 0.0f, int16_t neutralAngle = 90);

void SetDirectionReversed(bool reversed);
void SetFeedforward(float feedforward);

// Signed power in [-1, 1]: positive raises, negative lowers, zero holds.
// The upward feedforward is added before the output is clamped.
void SetPower(float power);

// Power magnitude in [0, 1].
void Up(float power);
void Down(float power);
void Hold();

} // namespace Lift
