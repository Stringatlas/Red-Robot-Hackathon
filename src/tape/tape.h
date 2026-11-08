#pragma once

#include <stdint.h>

namespace Tape {

constexpr uint8_t kSensorCount = 6;

struct Sample {
  uint16_t raw[kSensorCount];
  float varianceUs2[kSensorCount];
  bool detected;
  float positionMm;
};

// Reset calibration and running sensor statistics.
void Init();

// Call once per update while stationary. Completes after two samples, using a
// ground-only baseline when no tape is under the center sensors.
bool Calibrate();

// Read the sensors and estimate tape position relative to the array center.
void Read(Sample &sample);

} // namespace Tape
