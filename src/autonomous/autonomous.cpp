#include <Arduino.h>
#include <atomic>

#include "autonomous.h"
#include "../../RedRobot.h"
#include "../drive/drive.h"
#include "../tape/tape.h"

namespace Autonomous {
namespace {

constexpr unsigned long kUpDurationMs = 5500;
constexpr unsigned long kDownDurationMs = 3250;
constexpr uint8_t kStraightMotorSpeed = 25;
constexpr unsigned long kLoopDelayMs = 20;

struct ParkingStep {
  unsigned long durationMs;
  int16_t leftSpeed;
  int16_t rightSpeed;
  void (*onStart)(); // Optional servo or other actuator action.
};

// Add timed actions before the zero-duration end marker. For example:
// {500, 30, 30, nullptr} drives forward for half a second.
const ParkingStep kParkingSteps[] = {
    {0, 0, 0, nullptr},
};

State state = State::Idle;
Routine activeRoutine = Routine::Up;
unsigned long followingStartedAt = 0;
unsigned long stepStartedAt = 0;
size_t parkingStepIndex = 0;
bool parkingStepStarted = false;
// 0 = no request, 1 = UP, 2 = DOWN. The remote task only posts a request;
// the Arduino loop owns all state transitions and motor commands.
std::atomic<uint8_t> startRequested{0};
std::atomic<bool> driverControlRequested{false};

const char *StateName(State value) {
  switch (value) {
    case State::Idle: return "idle";
    case State::SearchTape: return "search tape";
    case State::FollowTape: return "follow tape";
    case State::Parking: return "parking";
    case State::Complete: return "complete";
    case State::DriverControl: return "driver control";
    case State::CalibrateTape: return "calibrate tape";
  }
  return "unknown";
}

void Enter(State next) {
  if (state == next) return;
  if (next == State::Parking || next == State::Complete) Drive::Stop();
  if (next == State::FollowTape) {
    followingStartedAt = millis();
    Drive::ResetPid();
  }
  if (next == State::Parking) {
    parkingStepIndex = 0;
    parkingStepStarted = false;
  }
  state = next;
  RedRobot::SetStateDigit(static_cast<uint8_t>(state));
  Serial.printf("Autonomous state: %s\n", StateName(state));
}

void FinishAutonomous() {
  Enter(State::Complete);
  RedRobot::StartRemoteControl();
  Enter(State::DriverControl);
}

void PrintTapeSample(const Tape::Sample &sample) {
  for (uint8_t i = 0; i < Tape::kSensorCount; ++i) {
    Serial.printf("%u: %u (var %.0f)  ", i + 1, sample.raw[i],
                  sample.varianceUs2[i]);
  }
  Serial.println();
}

void UpdateParking(unsigned long now) {
  const ParkingStep &step = kParkingSteps[parkingStepIndex];
  if (step.durationMs == 0) {
    FinishAutonomous();
    return;
  }

  if (!parkingStepStarted) {
    Drive::SetTankSpeeds(step.leftSpeed, step.rightSpeed);
    if (step.onStart) step.onStart();
    stepStartedAt = now;
    parkingStepStarted = true;
  }
  if (now - stepStartedAt >= step.durationMs) {
    Drive::Stop();
    ++parkingStepIndex;
    parkingStepStarted = false;
  }
}

void UpdateTapeRoutine(unsigned long now, unsigned long durationMs) {
  // Calibration does not count toward the following deadline.
  if (state == State::FollowTape &&
      now - followingStartedAt >= durationMs) {
    FinishAutonomous();
    return;
  }

  switch (state) {
    case State::CalibrateTape:
      if (Tape::Calibrate()) Enter(State::FollowTape);
      break;
    case State::SearchTape: {
      Tape::Sample sample;
      Tape::Read(sample);
      PrintTapeSample(sample);
      if (sample.detected) {
        Enter(State::FollowTape);
        Drive::FollowTape(sample.positionMm);
      } else {
        Drive::SetTankSpeeds(kStraightMotorSpeed, kStraightMotorSpeed);
      }
      break;
    }
    case State::FollowTape: {
      Tape::Sample sample;
      Tape::Read(sample);
      PrintTapeSample(sample);
      if (sample.detected) {
        Drive::FollowTape(sample.positionMm);
        Serial.printf("Tape error: %.2f mm\n", sample.positionMm);
      } else {
        // Cross a gap or intersection without steering until tape returns.
        Drive::SetTankSpeeds(kStraightMotorSpeed, kStraightMotorSpeed);
      }
      break;
    }
    case State::Parking:
      UpdateParking(now);
      break;
    case State::Complete:
      FinishAutonomous();
      break;
    case State::Idle:
    case State::DriverControl:
      break;
  }
}

void UpdateUpRoutine(unsigned long now) {
  UpdateTapeRoutine(now, kUpDurationMs);
}

void UpdateDownRoutine(unsigned long now) {
  UpdateTapeRoutine(now, kDownDurationMs);
}

} // namespace

void Init() {
  state = State::Idle;
  activeRoutine = Routine::Up;
  followingStartedAt = 0;
  parkingStepIndex = 0;
  parkingStepStarted = false;
  startRequested.store(0);
  driverControlRequested.store(false);
  RedRobot::SetStateDigit(static_cast<uint8_t>(state));
  Serial.printf("Autonomous state: %s\n", StateName(state));
}

void RequestStart(Routine routine) {
  startRequested.store(routine == Routine::Up ? 1 : 2);
}

void RequestDriverControl() {
  driverControlRequested.store(true);
}

State CurrentState() {
  return state;
}

void Update() {
  const unsigned long now = millis();
  const uint8_t requestedRoutine = startRequested.exchange(0);
  const bool shouldTakeOver = driverControlRequested.exchange(false);
  if (RedRobot::IsRemoteControlled()) {
    Enter(State::DriverControl);
  } else if (shouldTakeOver && state != State::Idle &&
             state != State::DriverControl) {
    Serial.println("Manual driver takeover requested");
    RedRobot::StartRemoteControl();
    Enter(State::DriverControl);
  } else if (state == State::Idle && requestedRoutine != 0) {
    activeRoutine = requestedRoutine == 1 ? Routine::Up : Routine::Down;
    Serial.printf("Autonomous routine: %s\n",
                  activeRoutine == Routine::Up ? "UP" : "DOWN");
    Tape::Init();
    Drive::Stop();
    Enter(State::CalibrateTape);
  }

  if (state != State::Idle && state != State::DriverControl) {
    if (activeRoutine == Routine::Up)
      UpdateUpRoutine(now);
    else
      UpdateDownRoutine(now);
  }
  delay(kLoopDelayMs);
}

} // namespace Autonomous
