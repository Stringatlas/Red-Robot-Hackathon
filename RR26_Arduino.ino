/**
 * @brief Red Robot 2026 main code!
 * 
 * Read the comments below to figure out how to get started!
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"
#include <math.h>

// Declare some global variables
bool started; // boolean (true or false) to keep track of if we should start
int counter;  // integer to hold a count value

// Ground readings are at least 6000 us; tape readings are at most 5600 us.
constexpr uint8_t kLineSensorCount = 6;
constexpr uint16_t kGroundMinReadingUs = 6000;
constexpr uint16_t kTapeMaxReadingUs = 5600;
constexpr float kSensorSpacingMm = 9.525f; // center-to-center distance
constexpr float kAngleScale = 1.0f; // steering-angle gain
constexpr float kEstimatedForwardSpeedMmPerSecond = 100.0f;
constexpr uint8_t kCalibrationSamples = 50;
constexpr float kSensorFilterAlpha = 0.35f;
constexpr float kAngleFilterAlpha = 0.25f;
constexpr uint8_t kTrendWindow = 5;

float filteredReadings[kLineSensorCount];
float centerOffsetMm = 0.0f;
float calibrationOffsetSumMm = 0.0f;
float filteredAngleDegrees = 0.0f;
float previousLateralOffsetMm = 0.0f;
float recentTapeSlopes[kTrendWindow] = {};
unsigned long previousSampleMillis = 0;
uint8_t calibrationCount = 0;
uint8_t nextSlopeIndex = 0;
bool tapeCalibrated = false;
bool angleFilterReady = false;
bool previousOffsetReady = false;

// Classify tape using the reading threshold, then find its weighted center.
bool EstimateTapeOffsetMm(const float readings[kLineSensorCount],
                          float &offsetMm) {
  const float threshold =
      (kGroundMinReadingUs + kTapeMaxReadingUs) / 2.0f;
  const float calibrationRange =
      kGroundMinReadingUs - kTapeMaxReadingUs;
  float weightedPositionMm = 0.0f;
  float totalWeight = 0.0f;

  for (uint8_t i = 0; i < kLineSensorCount; ++i) {
    if (readings[i] > threshold) continue;

    // Use the calibrated contrast to weight stronger tape readings more.
    float weight = (kGroundMinReadingUs - readings[i]) / calibrationRange;
    if (weight > 1.0f) weight = 1.0f;
    const float positionMm =
        (i - (kLineSensorCount - 1) / 2.0f) * kSensorSpacingMm;
    weightedPositionMm += positionMm * weight;
    totalWeight += weight;
  }

  if (totalWeight == 0.0f) {
    return false;
  }

  offsetMm = weightedPositionMm / totalWeight;
  return true;
}

// The median rejects a one-sample position jump caused by a quick side shift.
float MedianTapeSlope() {
  float sorted[kTrendWindow];
  for (uint8_t i = 0; i < kTrendWindow; ++i) {
    sorted[i] = recentTapeSlopes[i];
  }
  for (uint8_t i = 1; i < kTrendWindow; ++i) {
    const float value = sorted[i];
    uint8_t j = i;
    while (j > 0 && sorted[j - 1] > value) {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = value;
  }
  return sorted[kTrendWindow / 2];
}

void ResetTapeTrend() {
  for (uint8_t i = 0; i < kTrendWindow; ++i) {
    recentTapeSlopes[i] = 0.0f;
  }
  nextSlopeIndex = 0;
  previousOffsetReady = false;
  angleFilterReady = false;
}

// Keep the robot still and centered on tape for the first valid samples.
// Their average tape position becomes the zero-offset reference.
bool UpdateTapeAngle(const uint16_t raw[kLineSensorCount],
                     float &lateralOffsetMm, float &angleDegrees) {
  float readings[kLineSensorCount];
  for (uint8_t i = 0; i < kLineSensorCount; ++i) {
    readings[i] = raw[i];
  }

  float offsetMm;
  if (!tapeCalibrated) {
    if (!EstimateTapeOffsetMm(readings, offsetMm)) {
      calibrationCount = 0;
      calibrationOffsetSumMm = 0.0f;
      ResetTapeTrend();
      return false;
    }

    calibrationOffsetSumMm += offsetMm;
    ++calibrationCount;
    if (calibrationCount < kCalibrationSamples) return false;

    centerOffsetMm = calibrationOffsetSumMm / kCalibrationSamples;
    tapeCalibrated = true;
    for (uint8_t i = 0; i < kLineSensorCount; ++i) {
      filteredReadings[i] = readings[i];
    }
    filteredAngleDegrees = 0.0f;
    ResetTapeTrend();
    lateralOffsetMm = 0.0f;
    angleDegrees = 0.0f;
    return true;
  }

  // The sensor filter uses past samples before classifying tape and ground.
  for (uint8_t i = 0; i < kLineSensorCount; ++i) {
    filteredReadings[i] +=
        kSensorFilterAlpha * (readings[i] - filteredReadings[i]);
  }
  if (!EstimateTapeOffsetMm(filteredReadings, offsetMm)) {
    ResetTapeTrend();
    return false;
  }

  lateralOffsetMm = offsetMm - centerOffsetMm;
  const unsigned long nowMillis = millis();
  if (!previousOffsetReady) {
    previousLateralOffsetMm = lateralOffsetMm;
    previousSampleMillis = nowMillis;
    previousOffsetReady = true;
    filteredAngleDegrees = 0.0f;
    angleDegrees = 0.0f;
    return true;
  }

  // Estimate the tape's slope from its change in position as the robot
  // moves forward. A short side shift creates one outlier in this history.
  const float travelMm = kEstimatedForwardSpeedMmPerSecond *
                         (nowMillis - previousSampleMillis) / 1000.0f;
  if (travelMm > 0.0f) {
    recentTapeSlopes[nextSlopeIndex] =
        (lateralOffsetMm - previousLateralOffsetMm) / travelMm;
    nextSlopeIndex = (nextSlopeIndex + 1) % kTrendWindow;
  }
  previousLateralOffsetMm = lateralOffsetMm;
  previousSampleMillis = nowMillis;

  const float measuredAngleDegrees =
      atan2f(MedianTapeSlope(), 1.0f) * RAD_TO_DEG * kAngleScale;
  if (!angleFilterReady) {
    filteredAngleDegrees = measuredAngleDegrees;
    angleFilterReady = true;
  } else {
    filteredAngleDegrees +=
        kAngleFilterAlpha * (measuredAngleDegrees - filteredAngleDegrees);
  }
  angleDegrees = filteredAngleDegrees;
  return true;
}

// This function runs ONCE, when the microcontroller boots up
void setup() {
    // Initialize all components on the robot before doing any work
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();            // call this to activate the AutonomousLoop

  Serial.println("Initialized.");
  // Initialize our variables
  started = false;
  counter = 0;
}

// This function runs over and over (in a loop) when you are in autonomous mode.
// As an example, this code moves out of the starting box at start, and 
// Look at RedRobot.h to see the robot interface: what functions you can call and how.
void AutonomousLoop() {
  uint16_t readings[kLineSensorCount];
  RedRobot::ReadLineSensor(readings);
  float lateralOffsetMm;
  float relativeAngleDegrees;
  const bool tapeDetected =
      UpdateTapeAngle(readings, lateralOffsetMm, relativeAngleDegrees);

  for (uint8_t i = 0; i < kLineSensorCount; ++i) {
    Serial.printf("%u: %u  ", i + 1, readings[i]);
  }
  Serial.println();

  if (!tapeCalibrated) {
    Serial.printf("Calibrating: %u/%u centered tape samples\n",
                  calibrationCount, kCalibrationSamples);
  } else if (tapeDetected) {
    Serial.printf("Tape offset: %.2f mm, angle: %.2f degrees\n",
                  lateralOffsetMm, relativeAngleDegrees);
  } else {
    Serial.println("Tape not detected");
  }

  delay(50);
}
