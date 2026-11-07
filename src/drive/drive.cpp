#include <Arduino.h>
#include <math.h>

#include "drive.h"
#include "../../RedRobot.h"

namespace Drive {
namespace {

constexpr int kBaseMotorSpeed = 35;
constexpr float kKp = 1.3f; // motor-speed correction per mm of error
constexpr float kKi = 0.15f;
constexpr float kKd = 0.02f;
constexpr float kIntegralLimit = 100.0f;

float integralError = 0.0f;
float previousErrorMm = 0.0f;
unsigned long previousPidMillis = 0;
bool movementEnabled = false;

} // namespace

void Stop() {
  RedRobot::WriteDCMotor(2, 0);
  RedRobot::WriteDCMotor(3, 0);
}

void SetTankSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
  if (!movementEnabled) return;
  RedRobot::WriteDCMotor(2, constrain(leftSpeed, -100, 100));
  RedRobot::WriteDCMotor(3, constrain(rightSpeed, -100, 100));
}

void ResetPid() {
  integralError = 0.0f;
  previousErrorMm = 0.0f;
  previousPidMillis = millis();
}

void Init(bool enableMovement) {
  movementEnabled = enableMovement;
  Stop();
  ResetPid();
}

void FollowTape(float errorMm) {
  const unsigned long nowMillis = millis();
  const float dtSeconds = (nowMillis - previousPidMillis) / 1000.0f;
  previousPidMillis = nowMillis;
  float derivative = 0.0f;
  if (dtSeconds > 0.0f) {
    integralError = constrain(integralError + errorMm * dtSeconds,
                              -kIntegralLimit, kIntegralLimit);
    derivative = (errorMm - previousErrorMm) / dtSeconds;
  }
  previousErrorMm = errorMm;

  const float correction = constrain(kKp * errorMm +
                                     kKi * integralError +
                                     kKd * derivative,
                                     -float(kBaseMotorSpeed),
                                     float(kBaseMotorSpeed));
  // Positive error means tape is right of center: speed up the left motor.
  SetTankSpeeds(roundf(kBaseMotorSpeed + correction),
                roundf(kBaseMotorSpeed - correction));
}

} // namespace Drive
