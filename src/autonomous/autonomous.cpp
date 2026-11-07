#include <Arduino.h>
#include <atomic>

#include "autonomous.h"
#include "../../RedRobot.h"
#include "../drive/drive.h"
#include "../tape/tape.h"

namespace Autonomous {
namespace {

constexpr unsigned long kAutonomousDurationMs = 20000;
constexpr uint8_t kSearchMotorSpeed = 25;
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
unsigned long autonomousStartedAt = 0;
unsigned long stepStartedAt = 0;
size_t parkingStepIndex = 0;
bool parkingStepStarted = false;
std::atomic<bool> startRequested{false};
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
  if (next == State::FollowTape) Drive::ResetPid();
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

} // namespace

void Init() {
  state = State::Idle;
  autonomousStartedAt = 0;
  parkingStepIndex = 0;
  parkingStepStarted = false;
  startRequested.store(false);
  driverControlRequested.store(false);
  RedRobot::SetStateDigit(static_cast<uint8_t>(state));
  Serial.printf("Autonomous state: %s\n", StateName(state));
}

void RequestStart() {
  startRequested.store(true);
}

void RequestDriverControl() {
  driverControlRequested.store(true);
}

State CurrentState() {
  return state;
}

void Update() {
  const unsigned long now = millis();
  const bool shouldStart = startRequested.exchange(false);
  const bool shouldTakeOver = driverControlRequested.exchange(false);
  if (RedRobot::IsRemoteControlled()) {
    Enter(State::DriverControl);
  } else if (shouldTakeOver && state != State::Idle &&
             state != State::DriverControl) {
    Serial.println("Manual driver takeover requested");
    RedRobot::StartRemoteControl();
    Enter(State::DriverControl);
  } else if (state == State::Idle && shouldStart) {
    autonomousStartedAt = now;
    Tape::Init();
    Drive::Stop();
    Enter(State::CalibrateTape);
  } else if (state != State::Idle &&
             now - autonomousStartedAt >= kAutonomousDurationMs) {
    FinishAutonomous();
  }

  switch (state) {
    case State::Idle:
      break;
    case State::CalibrateTape:
      if (Tape::Calibrate()) Enter(State::SearchTape);
      break;
    case State::SearchTape: {
      Tape::Sample sample;
      Tape::Read(sample);
      PrintTapeSample(sample);
      if (sample.detected) {
        Enter(State::FollowTape);
        Drive::FollowTape(sample.positionMm);
      } else {
        Drive::SetTankSpeeds(kSearchMotorSpeed, kSearchMotorSpeed);
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
        Enter(State::Parking);
      }
      break;
    }
    case State::Parking:
      UpdateParking(now);
      break;
    case State::Complete:
      FinishAutonomous();
      break;
    case State::DriverControl:
      break;
  }
  delay(kLoopDelayMs);
}

} // namespace Autonomous
