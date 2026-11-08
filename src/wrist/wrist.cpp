#include "wrist.h"
#include "../../RedRobot.h"

namespace Wrist {
namespace {

uint8_t servoIndex = 0;
int16_t upAngle = 0;
int16_t downAngle = 180;
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

bool SetPosition(int16_t angle) {
  if (!initialized || !ValidAngle(angle)) return false;
  SetAngle(angle);
  return true;
}

int16_t Position() {
  return position;
}

} // namespace Wrist
