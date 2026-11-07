#include <Arduino.h>

#include "tape.h"
#include "../../RedRobot.h"

namespace Tape {
namespace {

constexpr float kSensorSpacingMm = 9.525f;
constexpr float kSensorFilterAlpha = 0.35f;
constexpr uint8_t kCalibrationSamples = 24;
constexpr float kMinimumContrastUs = 200.0f;
constexpr float kMinimumVarianceScaleUs2 = 10000.0f;

struct Distribution {
  uint16_t count = 0;
  float mean = 0.0f;
  float m2 = 0.0f;
  uint16_t minimum = UINT16_MAX;
  uint16_t maximum = 0;

  void Add(uint16_t value) {
    ++count;
    const float difference = value - mean;
    mean += difference / count;
    m2 += difference * (value - mean);
    if (value < minimum) minimum = value;
    if (value > maximum) maximum = value;
  }

  float Variance() const { return count > 1 ? m2 / (count - 1) : 0.0f; }
};

float filteredReadings[kSensorCount] = {};
float sensorVarianceUs2[kSensorCount] = {};
bool sensorStatisticsReady = false;
bool calibrated = false;
uint8_t calibrationFrames = 0;
Distribution sensorCalibration[kSensorCount];
Distribution groundCalibration;
Distribution tapeCalibration;
float groundMaxReadingUs = 0.0f;
float tapeMinReadingUs = 0.0f;
float tapeThresholdUs = 0.0f;
float varianceScaleUs2 = kMinimumVarianceScaleUs2;

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
    if (readings[i] < tapeThresholdUs) continue;
    const float contrastWeight = constrain(
        (readings[i] - groundMaxReadingUs) /
            (tapeMinReadingUs - groundMaxReadingUs),
        0.0f, 2.0f);
    const float reliability = varianceScaleUs2 /
        (varianceScaleUs2 + sensorVarianceUs2[i]);
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
  calibrated = false;
  calibrationFrames = 0;
  groundMaxReadingUs = 0.0f;
  tapeMinReadingUs = 0.0f;
  tapeThresholdUs = 0.0f;
  varianceScaleUs2 = kMinimumVarianceScaleUs2;
  groundCalibration = Distribution{};
  tapeCalibration = Distribution{};
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    sensorCalibration[i] = Distribution{};
    filteredReadings[i] = 0.0f;
    sensorVarianceUs2[i] = 0.0f;
  }
}

bool Calibrate() {
  if (calibrated) return true;

  uint16_t raw[kSensorCount];
  RedRobot::ReadLineSensor(raw);
  for (uint8_t i = 0; i < kSensorCount; ++i)
    sensorCalibration[i].Add(raw[i]);
  // With the array centered, sensors 2 and 3 see tape; 0 and 5 see ground.
  // The two neighboring sensors may straddle an edge, so omit them here.
  groundCalibration.Add(raw[0]);
  groundCalibration.Add(raw[5]);
  tapeCalibration.Add(raw[2]);
  tapeCalibration.Add(raw[3]);

  if (++calibrationFrames < kCalibrationSamples) return false;
  if (tapeCalibration.minimum <= groundCalibration.maximum +
                                     kMinimumContrastUs) {
    Serial.printf("Tape calibration retry: ground %u-%u us, tape %u-%u us\n",
                  groundCalibration.minimum, groundCalibration.maximum,
                  tapeCalibration.minimum, tapeCalibration.maximum);
    Init();
    return false;
  }

  groundMaxReadingUs = groundCalibration.maximum;
  tapeMinReadingUs = tapeCalibration.minimum;
  tapeThresholdUs = (groundMaxReadingUs + tapeMinReadingUs) / 2.0f;
  varianceScaleUs2 = max(kMinimumVarianceScaleUs2,
                         groundCalibration.Variance() +
                             tapeCalibration.Variance());
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    filteredReadings[i] = sensorCalibration[i].mean;
    sensorVarianceUs2[i] = sensorCalibration[i].Variance();
  }
  sensorStatisticsReady = true;
  calibrated = true;
  Serial.printf("Tape calibration: ground %u-%u us "
                "(mean %.0f, var %.0f), tape %u-%u us "
                "(mean %.0f, var %.0f), threshold %.0f us\n",
                groundCalibration.minimum, groundCalibration.maximum,
                groundCalibration.mean, groundCalibration.Variance(),
                tapeCalibration.minimum, tapeCalibration.maximum,
                tapeCalibration.mean, tapeCalibration.Variance(),
                tapeThresholdUs);
  return true;
}

void Read(Sample &sample) {
  if (!calibrated) {
    for (uint8_t i = 0; i < kSensorCount; ++i) {
      sample.raw[i] = 0;
      sample.varianceUs2[i] = 0.0f;
    }
    sample.detected = false;
    sample.positionMm = 0.0f;
    return;
  }
  RedRobot::ReadLineSensor(sample.raw);
  UpdateSensorStatistics(sample.raw);
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    sample.varianceUs2[i] = sensorVarianceUs2[i];
  }
  sample.detected = EstimateTapePositionMm(sample.raw, sample.positionMm);
  if (!sample.detected) sample.positionMm = 0.0f;
}

} // namespace Tape
