/**
 * @file    imu_calib.h
 * @brief   Static gyro calibration offsets for MPU6050 on Robot Transporter TC 2026.
 *
 * Generated: 2026-06-21 via 3x serial `calib imu` command (700 samples each).
 * Robot must be completely STILL on flat surface during calibration.
 *
 * Calibration results (3 runs averaged):
 *   Run 1: X=3.8890  Y=4.4894  Z=-0.6647
 *   Run 2: X=3.8906  Y=4.4894  Z=-0.6589
 *   Run 3: X=3.8926  Y=4.4978  Z=-0.6645
 *   Average used below.
 *
 * HOW TO RECALIBRATE:
 *   1. Place robot flat and still on the floor.
 *   2. Open Serial Monitor (115200 baud).
 *   3. Send: calib imu
 *   4. Wait ~3 sec for "Offset dps: X, Y, Z" to appear.
 *   5. Update the three defines below.
 *   6. Set IMU_USE_STATIC_OFFSETS to true and re-flash.
 *
 * If IMU_USE_STATIC_OFFSETS is false, robot calibrates at every boot
 * (robot must stay still for ~3 sec after power-on).
 */

#pragma once

// Set true to use pre-calibrated static offsets (faster boot, no need to stay still).
// Set false to re-calibrate at every boot (robot must be still for ~3 sec).
#define IMU_USE_STATIC_OFFSETS  true

// Gyro zero-rate offsets in degrees-per-second (dps).
// Applied as: gyro_corrected = gyro_raw - offset
#define IMU_GYRO_OFFSET_X   3.8907f
#define IMU_GYRO_OFFSET_Y   4.4922f
#define IMU_GYRO_OFFSET_Z  -0.6627f

// Accelerometer offsets (raw LSB) — not used yet, reserved for future accel calibration.
// At +/-4g scale, 1g = 8192 LSB.
#define IMU_ACCEL_OFFSET_X   0
#define IMU_ACCEL_OFFSET_Y   0
#define IMU_ACCEL_OFFSET_Z   0
