#include <math.h>

#include "lift.h"
#include "../../RedRobot.h"

namespace Lift {
namespace {

uint8_t servoIndex = 0;
bool initialized = false;
bool directionReversed = false;
float gravityFeedforward = 0.0f;
float requestedPower = 0.0f;

float Clamp(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void WriteOutput() {
  if (!initialized) return;

  float output = Clamp(requestedPower + gravityFeedforward, -1.0f, 1.0f);
  if (directionReversed) output = -output;

  // This robot's modified servo is stationary at SetServo(90).
  const int16_t angle = static_cast<int16_t>(lroundf(90.0f + 90.0f * output));
  RedRobot::SetServo(servoIndex, angle);
}

} // namespace

bool Init(uint8_t index, bool reversed, float feedforward) {
  if (index > 3) return false;
  servoIndex = index;
  directionReversed = reversed;
  gravityFeedforward = Clamp(feedforward, 0.0f, 1.0f);
  requestedPower = 0.0f;
  initialized = true;
  WriteOutput();
  return true;
}

void SetDirectionReversed(bool reversed) {
  directionReversed = reversed;
  WriteOutput();
}

void SetFeedforward(float feedforward) {
  gravityFeedforward = Clamp(feedforward, 0.0f, 1.0f);
  WriteOutput();
}

void SetPower(float power) {
  requestedPower = Clamp(power, -1.0f, 1.0f);
  WriteOutput();
}

void Up(float power) {
  SetPower(Clamp(power, 0.0f, 1.0f));
}

void Down(float power) {
  SetPower(-Clamp(power, 0.0f, 1.0f));
}

void Hold() {
  SetPower(0.0f);
}

} // namespace Lift
