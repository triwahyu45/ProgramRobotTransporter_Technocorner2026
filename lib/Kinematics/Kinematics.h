#pragma once

#include <Arduino.h>

struct WheelCommand {
  int16_t fl;
  int16_t fr;
  int16_t rl;
  int16_t rr;
};

// Mecanum/omni-style 4 wheel mixing based on the UNY transporter reference.
// x, y, turn ranges: -32767..32767.
WheelCommand MixOmni4(int16_t x, int16_t y, int16_t turn);
