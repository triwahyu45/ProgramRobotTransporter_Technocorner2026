#pragma once

#include <Arduino.h>

struct ImuTelemetry {
  bool ready;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float yawDeg;
  float pitchDeg;
  float rollDeg;
};

class ImuManager {
public:
  bool begin(bool calibrateOnBoot = true);
  void update();
  ImuTelemetry telemetry() const;
  void resetYaw();
  void startGyroCalibration();
  bool isCalibrating() const;
  uint16_t calibrationSamples() const;
  void updateBuzzer();  // call from main loop — non-blocking buzzer state machine

private:
  void updateCalibration(uint32_t nowUs);

  bool _ready = false;
  bool _calibrating = false;
  uint32_t _lastUpdateUs = 0;
  uint32_t _lastCalibrationSampleUs = 0;
  uint16_t _calibrationGoodSamples = 0;
  double _calibrationSumX = 0.0;
  double _calibrationSumY = 0.0;
  double _calibrationSumZ = 0.0;
  ImuTelemetry _telemetry = {};
  float _yawOffsetDeg = 0.0f;
  float _lastRawYawDeg = 0.0f;
  float _gyroOffsetX = 0.0f;
  float _gyroOffsetY = 0.0f;
  float _gyroOffsetZ = 0.0f;
  float _rollEstimateDeg = 0.0f;
  float _pitchEstimateDeg = 0.0f;
  // Non-blocking buzzer state machine (no delay())
  uint8_t  _buzzerPhase = 0;   // 0=idle, 1=single beep, 5-7=double beep
  uint32_t _buzzerMs    = 0;
};

ImuManager &Imu();
