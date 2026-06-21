#include "Kinematics.h"
#include <math.h>

namespace {
double normalizeAxis(int16_t value) {
  return static_cast<double>(value) / 32767.0;
}

int16_t toMotorCommand(double value) {
  value = constrain(value, -1.0, 1.0);
  return static_cast<int16_t>(value * 32767.0);
}
}

WheelCommand MixOmni4(int16_t x, int16_t y, int16_t turn) {
  const double nx = normalizeAxis(x);
  const double ny = normalizeAxis(y);
  const double nt = normalizeAxis(turn);

  double fl = ny + nx + nt;
  double fr = -ny + nx + nt;
  double rl = ny - nx + nt;
  double rr = -ny - nx + nt;

  double maxWheel = fabs(fl);
  maxWheel = max(maxWheel, fabs(fr));
  maxWheel = max(maxWheel, fabs(rl));
  maxWheel = max(maxWheel, fabs(rr));

  if (maxWheel > 1.0) {
    fl /= maxWheel;
    fr /= maxWheel;
    rl /= maxWheel;
    rr /= maxWheel;
  }

  return {
      .fl = toMotorCommand(fl),
      .fr = toMotorCommand(fr),
      .rl = toMotorCommand(rl),
      .rr = toMotorCommand(rr),
  };
}
