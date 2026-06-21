#include "EncoderReader.h"

#include "pin_config.h"

namespace {
constexpr EncoderPins ENCODER_PINS[WHEEL_COUNT] = {
    {PIN_ENC1A, PIN_ENC1B, ENC_FL_INVERTED, false},
    {PIN_ENC2A, PIN_ENC2B, ENC_FR_INVERTED, false},
    {PIN_ENC3A, PIN_ENC3B, ENC_RL_INVERTED, false},
    {PIN_ENC4A, PIN_ENC4B, ENC_RR_INVERTED, false},
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
  beginChannel(WHEEL_FL, ENCODER_PINS[WHEEL_FL]);
  beginChannel(WHEEL_FR, ENCODER_PINS[WHEEL_FR]);
  beginChannel(WHEEL_RL, ENCODER_PINS[WHEEL_RL]);
  beginChannel(WHEEL_RR, ENCODER_PINS[WHEEL_RR]);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC1A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1B), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2B), isrFR, CHANGE);
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
  const float revolutions = static_cast<float>(delta) / ENCODER_COUNTS_PER_REV;
  return {nowCount, delta, revolutions / minutes};
}

void EncoderHub::printConfig(Stream &out) const {
  out.println("=== Encoder Config ===");
  out.print("Counts per wheel rev: ");
  out.println(ENCODER_COUNTS_PER_REV, 2);
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
