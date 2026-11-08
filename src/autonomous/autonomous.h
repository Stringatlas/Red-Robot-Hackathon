#pragma once

#include <stdint.h>

namespace Autonomous {

// The enum values are also the rightmost 7-segment display digit.
// 0 = idle, 1 = searching for tape, 2 = following tape,
// 3 = parking (open-loop actions), 4 = complete (brief transition),
// 5 = driver control, 6 = stationary tape calibration.
enum class State : uint8_t {
  Idle = 0,
  SearchTape = 1,
  FollowTape = 2,
  Parking = 3,
  Complete = 4,
  DriverControl = 5,
  CalibrateTape = 6,
};

enum class Routine : uint8_t {
  Up,
  Down,
};

// Initialize in idle. The selected routine's timer starts when tape is found
// and the robot enters FollowTape.
void Init();

// Safe to call from the remote-control task; requests outside Idle are ignored.
void RequestStart(Routine routine);

// Request a manual takeover while an autonomous run is active.
void RequestDriverControl();

// Advances the state machine once. Call from the Arduino loop task.
void Update();

State CurrentState();

} // namespace Autonomous
