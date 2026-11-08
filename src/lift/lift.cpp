#include <math.h>

#include "lift.h"
#include "../../RedRobot.h"

namespace Lift {
namespace {

uint8_t servoIndex = 0;
bool initialized = false;
bool directionReversed = false;
int16_t neutralAngle = 90;
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

  const float angle = Clamp(neutralAngle + 90.0f * output, -180.0f, 180.0f);
  RedRobot::SetServo(servoIndex, static_cast<int16_t>(lroundf(angle)));
}

} // namespace

bool Init(uint8_t index, bool reversed, float feedforward, int16_t neutral) {
  if (index > 3 || neutral < -180 || neutral > 180) return false;
  servoIndex = index;
  directionReversed = reversed;
  neutralAngle = neutral;
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
