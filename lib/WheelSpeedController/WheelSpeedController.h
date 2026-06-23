#pragma once

#include <Arduino.h>

#include "EncoderReader.h"
#include "PcaMotorDriver.h"

struct WheelTelemetry {
  float targetRpm;
  float measuredRpm;
  int16_t command;
  bool fault;
};

class WheelSpeedController {
public:
  void begin(PcaMotorDriver &driver, EncoderHub &encoders);
  void setEnabled(bool enabled);
  bool enabled() const;
  void setTargetRpm(float fl, float fr, float rl, float rr);
  void setTargetRpm(uint8_t wheel, float rpm);
  void driveRobotPercent(float xPercent, float yPercent, float turnPercent);
  void stop();
  void resetPid();
  void update(uint32_t nowUs);
  WheelTelemetry telemetry(uint8_t wheel) const;

private:
  PcaMotorDriver *_driver = nullptr;
  EncoderHub *_encoders = nullptr;
  uint32_t _lastUpdateUs = 0;
  float _targetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
  float _currentTargetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
  float _measuredRpm[WHEEL_COUNT] = {0, 0, 0, 0};
  float _integral[WHEEL_COUNT] = {0, 0, 0, 0};
  int16_t _command[WHEEL_COUNT] = {0, 0, 0, 0};
  bool _fault[WHEEL_COUNT] = {false, false, false, false};
  bool _enabled = true;

  int16_t computeCommand(uint8_t wheel, float rpm, float measured, float dtSec, bool hasEncoder = true);
  void writeCommands();
};

WheelSpeedController &SpeedController();

void InitWheelSpeedController();
void UpdateWheelSpeedController();
void DriveFLRpm(float rpm);
void DriveFRRpm(float rpm);
void DriveRLRpm(float rpm);
void DriveRRRpm(float rpm);
void SetWheelTargetRpm(float fl, float fr, float rl, float rr);
void DriveRobotPercent(float xPercent, float yPercent, float turnPercent);
void DriveAllRpm(float fl, float fr, float rl, float rr);
void StopRpmDrive();
