#include "claw.h"
#include "../../RedRobot.h"

namespace Claw {
namespace {

uint8_t servoIndex = 0;
int16_t openAngle = 0;
int16_t closedAngle = 180;
int16_t initialAngle = 90;
bool initialized = false;

bool ValidAngle(int16_t angle) {
  return angle >= 0 && angle <= 180;
}

void SetAngle(int16_t angle) {
  // SetServo(0) is a 1500 us pulse; standard servo 90 degrees is 1500 us.
  RedRobot::SetServo(servoIndex, angle - 90);
}

} // namespace

bool Init(uint8_t index, int16_t open, int16_t closed, int16_t initial) {
  if (index > 3 || !ValidAngle(open) || !ValidAngle(closed) ||
      !ValidAngle(initial)) return false;

  servoIndex = index;
  openAngle = open;
  closedAngle = closed;
  initialAngle = initial;
  initialized = true;
  SetAngle(initial);
  return true;
}

void Open() {
  if (initialized) SetAngle(openAngle);
}

void Close() {
  if (initialized) SetAngle(closedAngle);
}

void GoToInitial() {
  if (initialized) SetAngle(initialAngle);
}

} // namespace Claw
