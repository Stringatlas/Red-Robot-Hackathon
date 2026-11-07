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

// Call once per update while stationary and centered on tape. Returns true
// after a usable ground/tape distribution has been collected.
bool Calibrate();

// Read the sensors and estimate tape position relative to the array center.
void Read(Sample &sample);

} // namespace Tape
