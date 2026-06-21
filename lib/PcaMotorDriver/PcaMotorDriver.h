#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

struct PcaTb6612MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
  bool inverted;
};

class PcaTb6612Motor {
public:
  PcaTb6612Motor();
  PcaTb6612Motor(Adafruit_PWMServoDriver *pca, PcaTb6612MotorPins pins);

  void attach(Adafruit_PWMServoDriver *pca, PcaTb6612MotorPins pins);

  // Command range: -32767..32767.
  // Positive = forward, negative = reverse, zero = brake.
  void drive(int16_t value);

  void brake();
  void coast();

private:
  Adafruit_PWMServoDriver *_pca;
  PcaTb6612MotorPins _pins;

  void channelOn(uint8_t channel);
  void channelOff(uint8_t channel);
  void channelDuty(uint8_t channel, uint16_t duty);
};

class PcaMotorDriver {
public:
  explicit PcaMotorDriver(uint8_t address = 0x40);

  bool begin(TwoWire &wire = Wire, uint16_t pwmFreqHz = 1000);

  void driveFL(int16_t value);
  void driveFR(int16_t value);
  void driveRL(int16_t value);
  void driveRR(int16_t value);

  void driveAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
  void brakeAll();
  void coastAll();

  Adafruit_PWMServoDriver &raw();

private:
  Adafruit_PWMServoDriver _pca;
  PcaTb6612Motor _frontLeft;
  PcaTb6612Motor _frontRight;
  PcaTb6612Motor _rearLeft;
  PcaTb6612Motor _rearRight;
};

// Global helper API for main loop convenience.
bool InitDriveMotors(uint16_t pwmFreqHz = 1000);
void DriveFL(int16_t value);
void DriveFR(int16_t value);
void DriveRL(int16_t value);
void DriveRR(int16_t value);
void DriveAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void BrakeAll();
void CoastAll();
PcaMotorDriver &DriveMotorBus();
