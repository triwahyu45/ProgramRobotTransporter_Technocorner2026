#include "PcaMotorDriver.h"
#include "pin_config.h"
#include <Wire.h>

namespace {
constexpr uint16_t PCA_FULL_ON = 4096;
constexpr uint16_t PCA_FULL_OFF = 4096;
constexpr uint16_t PCA_MAX_DUTY = 4095;

PcaMotorDriver driveMotors(ADDR_PCA9685);

uint16_t scaleCommandToDuty(int16_t value) {
  const int32_t clamped = constrain(static_cast<int32_t>(value), -32767, 32767);
  const uint32_t magnitude = static_cast<uint32_t>(abs(clamped));
  return static_cast<uint16_t>((magnitude * PCA_MAX_DUTY) / 32767UL);
}
}

PcaTb6612Motor::PcaTb6612Motor() : _pca(nullptr), _pins{0, 0, 0, false} {}

PcaTb6612Motor::PcaTb6612Motor(Adafruit_PWMServoDriver *pca,
                               PcaTb6612MotorPins pins)
    : _pca(pca), _pins(pins) {}

void PcaTb6612Motor::attach(Adafruit_PWMServoDriver *pca,
                            PcaTb6612MotorPins pins) {
  _pca = pca;
  _pins = pins;
}

void PcaTb6612Motor::channelOn(uint8_t channel) {
  _pca->setPWM(channel, PCA_FULL_ON, 0);
}

void PcaTb6612Motor::channelOff(uint8_t channel) {
  _pca->setPWM(channel, 0, PCA_FULL_OFF);
}

void PcaTb6612Motor::channelDuty(uint8_t channel, uint16_t duty) {
  duty = constrain(duty, 0, PCA_MAX_DUTY);
  _pca->setPWM(channel, 0, duty);
}

void PcaTb6612Motor::drive(int16_t value) {
  if (_pca == nullptr) {
    return;
  }

  if (value == 0) {
    brake();
    return;
  }

  const bool forwardCommand = value > 0;
  const bool forward = _pins.inverted ? !forwardCommand : forwardCommand;
  const uint16_t duty = scaleCommandToDuty(value);

  // ZK-5AD (TA6586): tidak ada pin ENA/ENB terpisah.
  // Speed dikontrol dengan mengirim PWM langsung ke IN1 (forward)
  // atau IN2 (backward). Pin PWM (ENA) harus HIGH agar driver aktif.
  if (forward) {
    channelDuty(_pins.in1, duty);  // IN1 = PWM duty (speed)
    channelOff(_pins.in2);          // IN2 = LOW
  } else {
    channelOff(_pins.in1);          // IN1 = LOW
    channelDuty(_pins.in2, duty);  // IN2 = PWM duty (speed)
  }
  channelOn(_pins.pwm);  // ENA harus HIGH agar ZK-5AD aktif
}

void PcaTb6612Motor::brake() {
  if (_pca == nullptr) {
    return;
  }

  // ZK-5AD (TA6586) brake: IN1=HIGH, IN2=HIGH = short brake.
  // ENA (PWM pin) HARUS HIGH agar brake mode aktif — jika LOW, motor coast!
  channelOn(_pins.in1);
  channelOn(_pins.in2);
  channelOn(_pins.pwm);  // ENA=HIGH agar brake bekerja
}

void PcaTb6612Motor::coast() {
  if (_pca == nullptr) {
    return;
  }

  // TB6612FNG coast/free run: IN1=LOW, IN2=LOW.
  channelOff(_pins.in1);
  channelOff(_pins.in2);
  channelOff(_pins.pwm);
}

PcaMotorDriver::PcaMotorDriver(uint8_t address) : _pca(address) {}

bool PcaMotorDriver::begin(TwoWire &wire, uint16_t pwmFreqHz) {
  _pca = Adafruit_PWMServoDriver(ADDR_PCA9685, wire);
  _pca.begin();
  _pca.setOscillatorFrequency(27000000);
  _pca.setPWMFreq(pwmFreqHz);

  _frontLeft.attach(&_pca, {M1_PWM, M1_IN1, M1_IN2, MOTOR_FL_INVERTED});
  _frontRight.attach(&_pca, {M2_PWM, M2_IN1, M2_IN2, MOTOR_FR_INVERTED});
  _rearLeft.attach(&_pca, {M3_PWM, M3_IN1, M3_IN2, MOTOR_RL_INVERTED});
  _rearRight.attach(&_pca, {M4_PWM, M4_IN1, M4_IN2, MOTOR_RR_INVERTED});

  brakeAll();
  return true;
}

void PcaMotorDriver::driveFL(int16_t value) { _frontLeft.drive(value); }
void PcaMotorDriver::driveFR(int16_t value) { _frontRight.drive(value); }
void PcaMotorDriver::driveRL(int16_t value) { _rearLeft.drive(value); }
void PcaMotorDriver::driveRR(int16_t value) { _rearRight.drive(value); }

void PcaMotorDriver::driveAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  driveFL(fl);
  driveFR(fr);
  driveRL(rl);
  driveRR(rr);
}

void PcaMotorDriver::brakeAll() {
  _frontLeft.brake();
  _frontRight.brake();
  _rearLeft.brake();
  _rearRight.brake();
}

void PcaMotorDriver::coastAll() {
  _frontLeft.coast();
  _frontRight.coast();
  _rearLeft.coast();
  _rearRight.coast();
}

Adafruit_PWMServoDriver &PcaMotorDriver::raw() { return _pca; }

bool InitDriveMotors(uint16_t pwmFreqHz) {
  return driveMotors.begin(Wire, pwmFreqHz);
}

void DriveFL(int16_t value) { driveMotors.driveFL(value); }
void DriveFR(int16_t value) { driveMotors.driveFR(value); }
void DriveRL(int16_t value) { driveMotors.driveRL(value); }
void DriveRR(int16_t value) { driveMotors.driveRR(value); }

void DriveAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  driveMotors.driveAll(fl, fr, rl, rr);
}

void BrakeAll() { driveMotors.brakeAll(); }
void CoastAll() { driveMotors.coastAll(); }
PcaMotorDriver &DriveMotorBus() { return driveMotors; }
