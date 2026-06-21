#pragma once
#include <Arduino.h>

/**
 * ServoESP32 — Servo driver menggunakan Arduino LEDC API.
 * Frekuensi: 50Hz, Resolusi: 16-bit (65536 ticks)
 * 0.5ms pulse = 1638 ticks (0°), 2.5ms pulse = 8192 ticks (180°)
 */
class ServoESP32 {
public:
  ServoESP32() = default;

  // pin    : GPIO number (misal 12, 13, 14, 32)
  // channel: LEDC channel (0-15, unik per servo)
  void begin(uint8_t pin, uint8_t channel);

  // Set sudut servo (0.0 - 180.0 derajat)
  void setAngle(float angleDeg);

  uint8_t getChannel() const { return _channel; }

private:
  uint8_t _channel = 0;

  static constexpr uint32_t FREQ_HZ    = 50;
  static constexpr uint8_t  RES_BITS   = 16;
  static constexpr uint32_t TICKS_MIN  = 1638;  // 0.5ms → 0°
  static constexpr uint32_t TICKS_MAX  = 8192;  // 2.5ms → 180°
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * Gripper — Mengontrol 1 pasang servo (claw + lifter) dengan angle yang
 * bisa dikonfigurasi. Tiap servo bisa punya range yang berbeda.
 */
struct GripperConfig {
  float claw_close;  // sudut claw saat tutup/gripping
  float claw_open;   // sudut claw saat buka/release
  float lift_down;   // sudut lifter saat turun/drop
  float lift_up;     // sudut lifter saat naik/angkat
};

class Gripper {
public:
  Gripper() = default;
  Gripper(ServoESP32* claw, ServoESP32* lifter, GripperConfig cfg);

  // gripping=true  → tutup claw (claw_close angle)
  // gripping=false → buka claw  (claw_open angle)
  void setClaw(bool gripping);

  // Langsung toggle state claw
  void toggleClaw();

  // throttle: 0.0 = turun, 1.0 = naik penuh
  void setLifter(float throttle);

  bool isClawClosed() const { return _clawClosed; }

private:
  ServoESP32*  _claw   = nullptr;
  ServoESP32*  _lifter = nullptr;
  GripperConfig _cfg   = {};
  bool _clawClosed     = true;
};
