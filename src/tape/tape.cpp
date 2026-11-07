#include <Arduino.h>

#include "tape.h"
#include "../../RedRobot.h"

namespace Tape {
namespace {

// Measured ground: about 1500-2200 us. Measured tape: about 3100-6500 us.
constexpr float kSensorSpacingMm = 9.525f;
constexpr float kGroundMaxReadingUs = 2300.0f;
constexpr float kTapeMinReadingUs = 3000.0f;
constexpr float kTapeThresholdUs =
    (kGroundMaxReadingUs + kTapeMinReadingUs) / 2.0f;
constexpr float kSensorFilterAlpha = 0.35f;
constexpr float kVarianceScaleUs2 = 300.0f;

float filteredReadings[kSensorCount] = {};
float sensorVarianceUs2[kSensorCount] = {};
bool sensorStatisticsReady = false;

// Track recent sensor variance without storing a full sample history.
void UpdateSensorStatistics(const uint16_t raw[kSensorCount]) {
  if (!sensorStatisticsReady) {
    for (uint8_t i = 0; i < kSensorCount; ++i) {
      filteredReadings[i] = raw[i];
    }
    sensorStatisticsReady = true;
    return;
  }

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    const float difference = raw[i] - filteredReadings[i];
    filteredReadings[i] += kSensorFilterAlpha * difference;
    sensorVarianceUs2[i] =
        (1.0f - kSensorFilterAlpha) *
        (sensorVarianceUs2[i] + kSensorFilterAlpha * difference * difference);
  }
}

// Locate tape from its signal. Noisy sensors get less weight in the centroid.
bool EstimateTapePositionMm(const uint16_t readings[kSensorCount],
                            float &positionMm) {
  float weightedPosition = 0.0f;
  float totalWeight = 0.0f;

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (readings[i] < kTapeThresholdUs) continue;
    const float contrastWeight = constrain(
        (readings[i] - kGroundMaxReadingUs) /
            (kTapeMinReadingUs - kGroundMaxReadingUs),
        0.0f, 2.0f);
    const float reliability = kVarianceScaleUs2 /
        (kVarianceScaleUs2 + sensorVarianceUs2[i]);
    const float weight = contrastWeight * reliability;
    weightedPosition +=
        weight * (i - (kSensorCount - 1) / 2.0f) * kSensorSpacingMm;
    totalWeight += weight;
  }

  if (totalWeight == 0.0f) return false;
  positionMm = weightedPosition / totalWeight;
  return true;
}

} // namespace

void Init() {
  sensorStatisticsReady = false;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    filteredReadings[i] = 0.0f;
    sensorVarianceUs2[i] = 0.0f;
  }
}

void Read(Sample &sample) {
  RedRobot::ReadLineSensor(sample.raw);
  UpdateSensorStatistics(sample.raw);
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    sample.varianceUs2[i] = sensorVarianceUs2[i];
  }
  sample.detected = EstimateTapePositionMm(sample.raw, sample.positionMm);
  if (!sample.detected) sample.positionMm = 0.0f;
}

} // namespace Tape
