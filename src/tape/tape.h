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

// Reset the running sensor statistics.
void Init();

// Read the sensors and estimate tape position relative to the array center.
void Read(Sample &sample);

} // namespace Tape
