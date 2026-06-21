#include "ImuManager.h"

#include <Wire.h>
#include <math.h>

#include "pin_config.h"
#include "imu_calib.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float ACCEL_SCALE_LSB_PER_G = 8192.0f;  // +/-4g
constexpr float GYRO_SCALE_LSB_PER_DPS = 65.5f;   // +/-500 dps
constexpr float COMPLEMENTARY_ALPHA = 0.96f;
constexpr uint16_t GYRO_CAL_SAMPLES = 700;
constexpr uint32_t GYRO_CAL_SAMPLE_PERIOD_US = 2000;

ImuManager imuManager;

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool isKnownMotionChip(uint8_t whoAmI) {
  // 0x68 = MPU6050. 0x70/0x71/0x72/0x73 commonly appear on MPU6500/9250/clone modules.
  return whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x72 || whoAmI == 0x73;
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(ADDR_MPU6050), 1) != 1) return false;
  value = Wire.read();
  return true;
}

int16_t readI16(const uint8_t *buffer, uint8_t index) {
  return static_cast<int16_t>((static_cast<uint16_t>(buffer[index]) << 8) | buffer[index + 1]);
}

bool readMotionRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(static_cast<int>(ADDR_MPU6050), 14) != 14) return false;

  uint8_t buffer[14];
  for (uint8_t i = 0; i < sizeof(buffer); ++i) {
    buffer[i] = Wire.read();
  }

  ax = readI16(buffer, 0);
  ay = readI16(buffer, 2);
  az = readI16(buffer, 4);
  gx = readI16(buffer, 8);
  gy = readI16(buffer, 10);
  gz = readI16(buffer, 12);
  return true;
}
}

bool ImuManager::begin(bool calibrateOnBoot) {
  _ready = false;
  _calibrating = false;
  _lastCalibrationSampleUs = 0;
  _calibrationGoodSamples = 0;
  _calibrationSumX = 0.0;
  _calibrationSumY = 0.0;
  _calibrationSumZ = 0.0;
  _telemetry = {};
  _yawOffsetDeg = 0.0f;
  _lastRawYawDeg = 0.0f;
  _gyroOffsetX = 0.0f;
  _gyroOffsetY = 0.0f;
  _gyroOffsetZ = 0.0f;
  _rollEstimateDeg = 0.0f;
  _pitchEstimateDeg = 0.0f;

  Serial.println("IMU raw-register init...");

  uint8_t whoAmI = 0;
  if (!readReg(REG_WHO_AM_I, whoAmI)) {
    Serial.println("IMU WHO_AM_I read failed.");
    return false;
  }

  Serial.print("IMU WHO_AM_I=0x");
  if (whoAmI < 16) Serial.print('0');
  Serial.println(whoAmI, HEX);

  if (!isKnownMotionChip(whoAmI)) {
    Serial.println("IMU chip ID is not in accepted MPU60x0/MPU65x0/MPU92x0 clone list.");
    return false;
  }

  writeReg(REG_PWR_MGMT_1, 0x00);
  writeReg(REG_CONFIG, 0x03);        // DLPF ~44 Hz gyro / ~42 Hz accel on MPU6050-like chips.
  writeReg(REG_SMPLRT_DIV, 0x04);    // 200 Hz-ish.
  writeReg(REG_GYRO_CONFIG, 0x08);   // +/-500 dps.
  writeReg(REG_ACCEL_CONFIG, 0x08);  // +/-4g.

  _ready = true;
  _telemetry.ready = true;
  _lastUpdateUs = micros();
  update();
  Serial.println("IMU raw-register ready.");

#if IMU_USE_STATIC_OFFSETS
  // Gunakan offset statik hasil kalibrasi — robot tidak perlu diam saat boot.
  _gyroOffsetX = IMU_GYRO_OFFSET_X;
  _gyroOffsetY = IMU_GYRO_OFFSET_Y;
  _gyroOffsetZ = IMU_GYRO_OFFSET_Z;
  _lastRawYawDeg = 0.0f;
  _yawOffsetDeg  = 0.0f;
  _telemetry.yawDeg = 0.0f;
  Serial.print("IMU static offsets loaded: X=");
  Serial.print(_gyroOffsetX, 4);
  Serial.print(" Y=");
  Serial.print(_gyroOffsetY, 4);
  Serial.print(" Z=");
  Serial.println(_gyroOffsetZ, 4);
  _calibrating = false;
#else
  if (calibrateOnBoot) {
    startGyroCalibration();
  }
#endif
  return true;
}

void ImuManager::update() {
  if (!_ready) return;

  const uint32_t nowUs = micros();
  if (_calibrating) {
    updateCalibration(nowUs);
  }

  float dtSec = static_cast<float>(nowUs - _lastUpdateUs) / 1000000.0f;
  _lastUpdateUs = nowUs;
  if (dtSec <= 0.0f || dtSec > 0.2f) dtSec = 0.01f;

  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!readMotionRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    _telemetry.ready = false;
    return;
  }

  _telemetry.ready = true;
  _telemetry.accelX = static_cast<float>(axRaw) / ACCEL_SCALE_LSB_PER_G;
  _telemetry.accelY = static_cast<float>(ayRaw) / ACCEL_SCALE_LSB_PER_G;
  _telemetry.accelZ = static_cast<float>(azRaw) / ACCEL_SCALE_LSB_PER_G;
  _telemetry.gyroX = (static_cast<float>(gxRaw) / GYRO_SCALE_LSB_PER_DPS) - _gyroOffsetX;
  _telemetry.gyroY = (static_cast<float>(gyRaw) / GYRO_SCALE_LSB_PER_DPS) - _gyroOffsetY;
  _telemetry.gyroZ = (static_cast<float>(gzRaw) / GYRO_SCALE_LSB_PER_DPS) - _gyroOffsetZ;

  const float accelRollDeg = atan2f(_telemetry.accelY, _telemetry.accelZ) * RAD_TO_DEG;
  const float accelPitchDeg =
      atan2f(-_telemetry.accelX,
             sqrtf((_telemetry.accelY * _telemetry.accelY) + (_telemetry.accelZ * _telemetry.accelZ))) *
      RAD_TO_DEG;

  _rollEstimateDeg = COMPLEMENTARY_ALPHA * (_rollEstimateDeg + _telemetry.gyroX * dtSec) +
                     (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;
  _pitchEstimateDeg = COMPLEMENTARY_ALPHA * (_pitchEstimateDeg + _telemetry.gyroY * dtSec) +
                      (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;
  float gyroZFiltered = _telemetry.gyroZ;
  if (fabsf(gyroZFiltered) < 0.15f) {
    gyroZFiltered = 0.0f;
  }
  _lastRawYawDeg = wrap180(_lastRawYawDeg + (gyroZFiltered * dtSec));

  _telemetry.rollDeg = _rollEstimateDeg;
  _telemetry.pitchDeg = _pitchEstimateDeg;
  _telemetry.yawDeg = wrap180(_lastRawYawDeg - _yawOffsetDeg);
}

ImuTelemetry ImuManager::telemetry() const { return _telemetry; }

void ImuManager::resetYaw() {
  _yawOffsetDeg = _lastRawYawDeg;
  _telemetry.yawDeg = 0.0f;
}

void ImuManager::startGyroCalibration() {
  if (!_ready) {
    Serial.println("IMU calibration rejected: IMU is not ready.");
    return;
  }

  _calibrating = true;
  _lastCalibrationSampleUs = 0;
  _calibrationGoodSamples = 0;
  _calibrationSumX = 0.0;
  _calibrationSumY = 0.0;
  _calibrationSumZ = 0.0;
  Serial.println("IMU gyro calibration started. Keep robot still.");
}

bool ImuManager::isCalibrating() const {
  return _calibrating;
}

uint16_t ImuManager::calibrationSamples() const {
  return _calibrationGoodSamples;
}

void ImuManager::updateCalibration(uint32_t nowUs) {
  if (_lastCalibrationSampleUs != 0 && nowUs - _lastCalibrationSampleUs < GYRO_CAL_SAMPLE_PERIOD_US) {
    return;
  }
  _lastCalibrationSampleUs = nowUs;

  int16_t ax, ay, az, gx, gy, gz;
  if (!readMotionRaw(ax, ay, az, gx, gy, gz)) {
    return;
  }

  _calibrationSumX += static_cast<float>(gx) / GYRO_SCALE_LSB_PER_DPS;
  _calibrationSumY += static_cast<float>(gy) / GYRO_SCALE_LSB_PER_DPS;
  _calibrationSumZ += static_cast<float>(gz) / GYRO_SCALE_LSB_PER_DPS;
  _calibrationGoodSamples++;

  if (_calibrationGoodSamples < GYRO_CAL_SAMPLES) {
    return;
  }

  _gyroOffsetX = _calibrationSumX / _calibrationGoodSamples;
  _gyroOffsetY = _calibrationSumY / _calibrationGoodSamples;
  _gyroOffsetZ = _calibrationSumZ / _calibrationGoodSamples;
  _lastRawYawDeg = 0.0f;
  _yawOffsetDeg = 0.0f;
  _telemetry.yawDeg = 0.0f;
  _calibrating = false;

  Serial.print("IMU gyro calibration done. Offset dps: ");
  Serial.print(_gyroOffsetX, 4);
  Serial.print(", ");
  Serial.print(_gyroOffsetY, 4);
  Serial.print(", ");
  Serial.println(_gyroOffsetZ, 4);
}

ImuManager &Imu() { return imuManager; }
