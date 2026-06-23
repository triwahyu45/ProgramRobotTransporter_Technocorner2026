#include "WheelSpeedController.h"

#include "Kinematics.h"
#include "pin_config.h"

namespace {
constexpr uint32_t CONTROL_PERIOD_US = 20000;  // 50 Hz control loop
constexpr float KP = 55.0f;    // Turun dari 120 → lebih smooth, kurang agresif
constexpr float KI = 12.0f;    // Turun dari 40 → integrator lebih pelan, kurang buzz
constexpr float INTEGRAL_LIMIT = 8000.0f;
constexpr float RPM_FAULT_LIMIT = MAX_WHEEL_RPM * 2.0f;
constexpr float RPM_DEADBAND = 1.5f;  // Naik sedikit agar tidak buzz di sekitar nol
constexpr int16_t MAX_COMMAND = 32767;

WheelSpeedController speedController;

float clampRpm(float rpm) {
  return constrain(rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
}

int16_t clampCommand(float command) {
  return static_cast<int16_t>(constrain(command, -MAX_COMMAND, MAX_COMMAND));
}
}

void WheelSpeedController::begin(PcaMotorDriver &driver, EncoderHub &encoders) {
  _driver = &driver;
  _encoders = &encoders;
  _lastUpdateUs = micros();
  _enabled = true;
  resetPid();
  stop();
}

void WheelSpeedController::setEnabled(bool enabled) {
  if (_enabled == enabled) {
    return;
  }
  _enabled = enabled;
  if (enabled) {
    _lastUpdateUs = micros();
    resetPid();
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
      _currentTargetRpm[i] = _targetRpm[i];
    }
  }
}

bool WheelSpeedController::enabled() const {
  return _enabled;
}

void WheelSpeedController::setTargetRpm(float fl, float fr, float rl, float rr) {
  setEnabled(true);
  setTargetRpm(WHEEL_FL, fl);
  setTargetRpm(WHEEL_FR, fr);
  setTargetRpm(WHEEL_RL, rl);
  setTargetRpm(WHEEL_RR, rr);
}

void WheelSpeedController::setTargetRpm(uint8_t wheel, float rpm) {
  if (wheel >= WHEEL_COUNT) {
    return;
  }
  setEnabled(true);
  _targetRpm[wheel] = clampRpm(rpm);
  if (abs(_targetRpm[wheel]) < RPM_DEADBAND) {
    _targetRpm[wheel] = 0.0f;
    _integral[wheel] = 0.0f;
  }
}

void WheelSpeedController::driveRobotPercent(float xPercent, float yPercent, float turnPercent) {
  const auto scaleAxis = [](float value) -> int16_t {
    value = constrain(value, -100.0f, 100.0f);
    return static_cast<int16_t>((value / 100.0f) * 32767.0f);
  };

  const WheelCommand mix = MixOmni4(scaleAxis(xPercent), scaleAxis(yPercent), scaleAxis(turnPercent));
  const float rpmScale = MAX_WHEEL_RPM / 32767.0f;
  setTargetRpm(mix.fl * rpmScale, mix.fr * rpmScale, mix.rl * rpmScale, mix.rr * rpmScale);
}

void WheelSpeedController::stop() {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    _targetRpm[i] = 0.0f;
    _currentTargetRpm[i] = 0.0f;
    _command[i] = 0;
    _integral[i] = 0.0f;
    _fault[i] = false;
  }
  if (_driver != nullptr) {
    _driver->brakeAll();
  }
}

void WheelSpeedController::resetPid() {
  for (float &value : _integral) {
    value = 0.0f;
  }
}

void WheelSpeedController::update(uint32_t nowUs) {
  if (_driver == nullptr || _encoders == nullptr) {
    return;
  }
  if (!_enabled) {
    return;
  }

  const uint32_t elapsedUs = nowUs - _lastUpdateUs;
  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }
  _lastUpdateUs = nowUs;

  const float dtSec = static_cast<float>(elapsedUs) / 1000000.0f;
  
  // Acceleration limit (slew rate limiter) — lebih pelan = lebih smooth
  constexpr float MAX_ACCEL_RPM_PER_SEC = 120.0f;  // Turun dari 250 → 3x lebih smooth
  const float maxChange = MAX_ACCEL_RPM_PER_SEC * dtSec;

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    const EncoderSample sample = _encoders->sample(wheel, elapsedUs);
    _measuredRpm[wheel] = sample.rpm;

    _fault[wheel] = abs(_measuredRpm[wheel]) > RPM_FAULT_LIMIT;
    if (_fault[wheel]) {
      _command[wheel] = 0;
      _integral[wheel] = 0.0f;
      _currentTargetRpm[wheel] = 0.0f;
      continue;
    }

    // Slew rate limit target RPM
    float diff = _targetRpm[wheel] - _currentTargetRpm[wheel];
    if (diff > maxChange) {
      _currentTargetRpm[wheel] += maxChange;
    } else if (diff < -maxChange) {
      _currentTargetRpm[wheel] -= maxChange;
    } else {
      _currentTargetRpm[wheel] = _targetRpm[wheel];
    }

    _command[wheel] = computeCommand(wheel, _currentTargetRpm[wheel], _measuredRpm[wheel], dtSec);
  }

  writeCommands();
}

WheelTelemetry WheelSpeedController::telemetry(uint8_t wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return {0, 0, 0, true};
  }
  return {_targetRpm[wheel], _measuredRpm[wheel], _command[wheel], _fault[wheel]};
}

int16_t WheelSpeedController::computeCommand(uint8_t wheel, float rpm, float measured, float dtSec) {
  if (abs(rpm) < RPM_DEADBAND) {
    _integral[wheel] = 0.0f;
    return 0;
  }

  // Per-wheel max RPM untuk feed-forward yang akurat
  // Roda depan (gearbox baru) punya max RPM lebih rendah dari belakang
  float maxRpmForWheel = MAX_WHEEL_RPM;
  if (wheel == WHEEL_FL) maxRpmForWheel = MOTOR_MAX_RPM_FL;
  else if (wheel == WHEEL_FR) maxRpmForWheel = MOTOR_MAX_RPM_FR;
  else if (wheel == WHEEL_RL) maxRpmForWheel = MOTOR_MAX_RPM_RL;
  else if (wheel == WHEEL_RR) maxRpmForWheel = MOTOR_MAX_RPM_RR;

  // Per-wheel deadband compensation
  float minPwmRaw = 0.0f;
  if (wheel == WHEEL_FL) minPwmRaw = (MOTOR_MIN_PWM_FL / 100.0f) * 32767.0f;
  else if (wheel == WHEEL_FR) minPwmRaw = (MOTOR_MIN_PWM_FR / 100.0f) * 32767.0f;
  else if (wheel == WHEEL_RL) minPwmRaw = (MOTOR_MIN_PWM_RL / 100.0f) * 32767.0f;
  else if (wheel == WHEEL_RR) minPwmRaw = (MOTOR_MIN_PWM_RR / 100.0f) * 32767.0f;

  // Scale deadband untuk RPM kecil agar tidak buzz/jitter di sekitar nol
  constexpr float RPM_START_THRESHOLD = 6.0f;
  float scale = abs(rpm) / RPM_START_THRESHOLD;
  if (scale > 1.0f) scale = 1.0f;
  float effectiveMinPwm = minPwmRaw * scale;

  // Feed-forward berdasarkan per-wheel max RPM
  float feedForward = 0.0f;
  if (rpm > 0.0f) {
    feedForward = effectiveMinPwm + (rpm / maxRpmForWheel) * (32767.0f - effectiveMinPwm);
  } else if (rpm < 0.0f) {
    feedForward = -effectiveMinPwm + (rpm / maxRpmForWheel) * (32767.0f - effectiveMinPwm);
  }

  const float error = rpm - measured;
  _integral[wheel] = constrain(_integral[wheel] + (error * dtSec * KI), -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  return clampCommand(feedForward + (error * KP) + _integral[wheel]);
}

void WheelSpeedController::writeCommands() {
  _driver->driveAll(_command[WHEEL_FL], _command[WHEEL_FR], _command[WHEEL_RL], _command[WHEEL_RR]);
}

WheelSpeedController &SpeedController() { return speedController; }

void InitWheelSpeedController() {
  speedController.begin(DriveMotorBus(), Encoders());
}

void UpdateWheelSpeedController() {
  speedController.update(micros());
}

void DriveFLRpm(float rpm) { speedController.setTargetRpm(WHEEL_FL, rpm); }
void DriveFRRpm(float rpm) { speedController.setTargetRpm(WHEEL_FR, rpm); }
void DriveRLRpm(float rpm) { speedController.setTargetRpm(WHEEL_RL, rpm); }
void DriveRRRpm(float rpm) { speedController.setTargetRpm(WHEEL_RR, rpm); }

void SetWheelTargetRpm(float fl, float fr, float rl, float rr) {
  speedController.setTargetRpm(fl, fr, rl, rr);
}

void DriveRobotPercent(float xPercent, float yPercent, float turnPercent) {
  speedController.driveRobotPercent(xPercent, yPercent, turnPercent);
}

void DriveAllRpm(float fl, float fr, float rl, float rr) {
  SetWheelTargetRpm(fl, fr, rl, rr);
}

void StopRpmDrive() { speedController.stop(); }
