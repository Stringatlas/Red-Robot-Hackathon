#include "claw.h"
#include "../../RedRobot.h"

namespace Claw {
namespace {

uint8_t servoIndex = 0;
int16_t openAngle = 0;
int16_t closedAngle = 180;
int16_t initialAngle = 90;
int16_t position = 90;
bool initialized = false;

bool ValidAngle(int16_t angle) {
  return angle >= 0 && angle <= 180;
}

void SetAngle(int16_t angle) {
  // Map 0-180 degrees to the driver's full -180 to 180 range (500-2500 us).
  RedRobot::SetServo(servoIndex, 2 * angle - 180);
  position = angle;
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

bool SetPosition(int16_t angle) {
  if (!initialized || !ValidAngle(angle)) return false;
  SetAngle(angle);
  return true;
}

int16_t Position() {
  return position;
}

} // namespace Claw
