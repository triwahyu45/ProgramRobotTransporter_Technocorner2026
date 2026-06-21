#pragma once

#include <Arduino.h>

enum WheelIndex : uint8_t {
  WHEEL_FL = 0,
  WHEEL_FR = 1,
  WHEEL_RL = 2,
  WHEEL_RR = 3,
  WHEEL_COUNT = 4
};

struct EncoderPins {
  uint8_t pinA;
  uint8_t pinB;
  bool inverted;
  bool inputPullup;
};

struct EncoderSample {
  int32_t count;
  int32_t delta;
  float rpm;
};

class EncoderHub {
public:
  void begin();
  void reset();
  void reset(uint8_t wheel);

  int32_t count(uint8_t wheel) const;
  EncoderSample sample(uint8_t wheel, uint32_t elapsedUs);
  void printConfig(Stream &out) const;

  static void handleInterrupt(uint8_t wheel);

private:
  void beginChannel(uint8_t wheel, const EncoderPins &pins);
};

EncoderHub &Encoders();
