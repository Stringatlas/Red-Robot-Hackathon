#include "wrist.h"
#include "../../RedRobot.h"

namespace Wrist {
namespace {

uint8_t servoIndex = 0;
int16_t upAngle = 0;
int16_t downAngle = 180;
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

bool Init(uint8_t index, int16_t up, int16_t down, int16_t initial) {
  if (index > 3 || !ValidAngle(up) || !ValidAngle(down) ||
      !ValidAngle(initial)) return false;

  servoIndex = index;
  upAngle = up;
  downAngle = down;
  initialAngle = initial;
  initialized = true;
  SetAngle(initial);
  return true;
}

void Up() {
  if (initialized) SetAngle(upAngle);
}

void Down() {
  if (initialized) SetAngle(downAngle);
}

void GoToInitial() {
  if (initialized) SetAngle(initialAngle);
}

} // namespace Wrist
