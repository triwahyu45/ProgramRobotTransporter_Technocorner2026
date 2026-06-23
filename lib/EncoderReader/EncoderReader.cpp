#include "EncoderReader.h"

#include "pin_config.h"
#include <Preferences.h>

namespace {
// inputPullup field:
// true  = INPUT_PULLUP (ESP32 internal 45kOhm pull-up to 3.3V)
// false = INPUT (no pull-up, needed for GPIO 34/35/36/39 which cannot pull-up)
//
// FL GPIO 16/17, FR GPIO 25/26: SUPPORT pull-up → aktifkan!
// Encoder open-collector butuh pull-up untuk bisa menghasilkan HIGH level.
// RL GPIO 34/35, RR GPIO 36/39: input-only, TIDAK bisa pull-up.
constexpr EncoderPins ENCODER_PINS[WHEEL_COUNT] = {
    {PIN_ENC1A, PIN_ENC1B, ENC_FL_INVERTED, true},   // FL GPIO 16/17 → INPUT_PULLUP
    {PIN_ENC2A, PIN_ENC2B, ENC_FR_INVERTED, true},   // FR GPIO 25/26 → INPUT_PULLUP
    {PIN_ENC3A, PIN_ENC3B, ENC_RL_INVERTED, false},  // RL GPIO 34/35 → INPUT (no pullup)
    {PIN_ENC4A, PIN_ENC4B, ENC_RR_INVERTED, false},  // RR GPIO 36/39 → INPUT (no pullup)
};


constexpr int8_t QUAD_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

volatile int32_t counts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile uint8_t states[WHEEL_COUNT] = {0, 0, 0, 0};
int32_t lastSampleCounts[WHEEL_COUNT] = {0, 0, 0, 0};

EncoderHub encoderHub;

uint8_t readState(uint8_t wheel) {
  const EncoderPins &pins = ENCODER_PINS[wheel];
  const uint8_t a = digitalRead(pins.pinA) ? 1 : 0;
  const uint8_t b = digitalRead(pins.pinB) ? 1 : 0;
  return static_cast<uint8_t>((a << 1) | b);
}

void IRAM_ATTR isrFL() { EncoderHub::handleInterrupt(WHEEL_FL); }
void IRAM_ATTR isrFR() { EncoderHub::handleInterrupt(WHEEL_FR); }
void IRAM_ATTR isrRL() { EncoderHub::handleInterrupt(WHEEL_RL); }
void IRAM_ATTR isrRR() { EncoderHub::handleInterrupt(WHEEL_RR); }
}

void EncoderHub::beginChannel(uint8_t wheel, const EncoderPins &pins) {
  pinMode(pins.pinA, pins.inputPullup ? INPUT_PULLUP : INPUT);
  pinMode(pins.pinB, pins.inputPullup ? INPUT_PULLUP : INPUT);
  states[wheel] = readState(wheel);
}

void EncoderHub::begin() {
  // Load custom wheel PPR from NVS Preferences, fall back to per-wheel macro constants
  Preferences prefs;
  prefs.begin("encoder_calib", true); // read-only
  _ppr[WHEEL_FL] = prefs.getFloat("ppr_fl", ENCODER_PPR_FL);
  _ppr[WHEEL_FR] = prefs.getFloat("ppr_fr", ENCODER_PPR_FR);
  _ppr[WHEEL_RL] = prefs.getFloat("ppr_rl", ENCODER_PPR_RL);
  _ppr[WHEEL_RR] = prefs.getFloat("ppr_rr", ENCODER_PPR_RR);
  prefs.end();

  beginChannel(WHEEL_FL, ENCODER_PINS[WHEEL_FL]);
  beginChannel(WHEEL_FR, ENCODER_PINS[WHEEL_FR]);
  beginChannel(WHEEL_RL, ENCODER_PINS[WHEEL_RL]);
  beginChannel(WHEEL_RR, ENCODER_PINS[WHEEL_RR]);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC1A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1B), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2B), isrFR, CHANGE);
  // GPIO 34/35 (RL) dan 36/39 (RR) — user konfirmasi ada pull-up eksternal. Aktifkan.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC3A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC3B), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC4A), isrRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC4B), isrRR, CHANGE);
}

void EncoderHub::reset() {
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    counts[i] = 0;
    lastSampleCounts[i] = 0;
  }
  interrupts();
}

void EncoderHub::reset(uint8_t wheel) {
  if (wheel >= WHEEL_COUNT) {
    return;
  }
  noInterrupts();
  counts[wheel] = 0;
  lastSampleCounts[wheel] = 0;
  interrupts();
}

int32_t EncoderHub::count(uint8_t wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0;
  }
  noInterrupts();
  const int32_t value = counts[wheel];
  interrupts();
  return value;
}

EncoderSample EncoderHub::sample(uint8_t wheel, uint32_t elapsedUs) {
  if (wheel >= WHEEL_COUNT || elapsedUs == 0) {
    return {0, 0, 0.0f};
  }

  const int32_t nowCount = count(wheel);
  const int32_t delta = nowCount - lastSampleCounts[wheel];
  lastSampleCounts[wheel] = nowCount;

  const float minutes = static_cast<float>(elapsedUs) / 60000000.0f;
  const float revolutions = static_cast<float>(delta) / _ppr[wheel];
  return {nowCount, delta, revolutions / minutes};
}

void EncoderHub::setPpr(uint8_t wheel, float pprVal) {
  if (wheel >= WHEEL_COUNT || pprVal <= 0.0f) return;
  _ppr[wheel] = pprVal;

  Preferences prefs;
  prefs.begin("encoder_calib", false); // read-write
  if (wheel == WHEEL_FL) prefs.putFloat("ppr_fl", pprVal);
  else if (wheel == WHEEL_FR) prefs.putFloat("ppr_fr", pprVal);
  else if (wheel == WHEEL_RL) prefs.putFloat("ppr_rl", pprVal);
  else if (wheel == WHEEL_RR) prefs.putFloat("ppr_rr", pprVal);
  prefs.end();
}

float EncoderHub::ppr(uint8_t wheel) const {
  if (wheel >= WHEEL_COUNT) return ENCODER_COUNTS_PER_REV;
  return _ppr[wheel];
}

void EncoderHub::printConfig(Stream &out) const {
  out.println("=== Encoder Config ===");
  out.printf("PPR Settings        : FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f\n",
             _ppr[WHEEL_FL], _ppr[WHEEL_FR], _ppr[WHEEL_RL], _ppr[WHEEL_RR]);
  out.print("Max wheel RPM: ");
  out.println(MAX_WHEEL_RPM, 1);
  out.println("FL/ENC1 A=16 B=17, FR/ENC2 A=25 B=26, RL/ENC3 A=34 B=35, RR/ENC4 A=36 B=39");
  out.println("GPIO 34/35/36/39 are input-only and need external pull-up if encoder output is open collector.");
}

void IRAM_ATTR EncoderHub::handleInterrupt(uint8_t wheel) {
  if (wheel >= WHEEL_COUNT) {
    return;
  }

  const uint8_t previous = states[wheel];
  const uint8_t current = readState(wheel);
  states[wheel] = current;

  int8_t delta = QUAD_TABLE[(previous << 2) | current];
  if (ENCODER_PINS[wheel].inverted) {
    delta = -delta;
  }
  counts[wheel] += delta;
}

EncoderHub &Encoders() { return encoderHub; }
