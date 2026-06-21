#include "ServoGripper.h"

// ─── ServoESP32 ──────────────────────────────────────────────────────────────

void ServoESP32::begin(uint8_t pin, uint8_t channel) {
  _channel = channel;
  ledcSetup(channel, FREQ_HZ, RES_BITS);
  ledcAttachPin(pin, channel);
  // Posisi awal 0 derajat
  ledcWrite(channel, TICKS_MIN);
  Serial.printf("[Servo] ch%d gpio%d init OK (50Hz 16-bit)\n", channel, pin);
}

void ServoESP32::setAngle(float angleDeg) {
  if (angleDeg < 0.0f)   angleDeg = 0.0f;
  if (angleDeg > 180.0f) angleDeg = 180.0f;
  uint32_t ticks = (uint32_t)(TICKS_MIN + (angleDeg / 180.0f) * (TICKS_MAX - TICKS_MIN));
  ledcWrite(_channel, ticks);
}

// ─── Gripper ─────────────────────────────────────────────────────────────────

Gripper::Gripper(ServoESP32* claw, ServoESP32* lifter, GripperConfig cfg)
    : _claw(claw), _lifter(lifter), _cfg(cfg), _clawClosed(true) {
  // Posisi awal
  if (_claw)   _claw->setAngle(_cfg.claw_close);
  if (_lifter) _lifter->setAngle(_cfg.lift_up);
}

void Gripper::setClaw(bool gripping) {
  if (!_claw) return;
  _clawClosed = gripping;
  _claw->setAngle(gripping ? _cfg.claw_close : _cfg.claw_open);
}

void Gripper::toggleClaw() {
  setClaw(!_clawClosed);
}

void Gripper::setLifter(float throttle) {
  if (!_lifter) return;
  if (throttle < 0.0f) throttle = 0.0f;
  if (throttle > 1.0f) throttle = 1.0f;
  // Interpolasi: throttle=0 → down, throttle=1 → up
  float angle = _cfg.lift_down + throttle * (_cfg.lift_up - _cfg.lift_down);
  _lifter->setAngle(angle);
}
