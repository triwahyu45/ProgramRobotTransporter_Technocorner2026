/**
 * @file    pin_config.h
 * @brief   Main pin configuration for Robot Transporter Technocorner UGM 2026.
 */

#pragma once

// I2C bus: PCA9685, MPU6050, optional I2C sensors.
#define PIN_SDA             21
#define PIN_SCL             22

// 4x JGA25 / Ichibot encoder input.
// Encoder VCC must use 3V3 because ESP32 GPIO is not 5V tolerant.
// Logical wheel mapping stays 1=FL, 2=FR, 3=RL, 4=RR.
// PCB encoder headers are swapped front/back:
//   logical ENC1/ENC2 use physical headers ENC3/ENC4,
//   logical ENC3/ENC4 use physical headers ENC1/ENC2.
#define PIN_ENC1A           16      // Motor Front-Left, physical header ENC3
#define PIN_ENC1B           17
#define PIN_ENC2A           25      // Motor Front-Right, physical header ENC4
#define PIN_ENC2B           26
#define PIN_ENC3A           34      // Motor Rear-Left, physical header ENC1
#define PIN_ENC3B           35
#define PIN_ENC4A           36      // Motor Rear-Right, physical header ENC2
#define PIN_ENC4B           39

// Encoder tuning.
// PPR (Pulses Per Revolution) dikalibrasi manual via 'calib ppr start 20'.
// Nilai ini hanya dipakai sebagai FALLBACK jika NVS belum terisi.
// Hasil kalibrasi 2026-06-24:
//   FL=408.95, FR=408.00 (gearbox depan, rasio tinggi)
//   RL=193.70, RR=195.10 (gearbox belakang, rasio rendah)
// Gunakan rata-rata sebagai fallback default:
#define ENCODER_COUNTS_PER_REV  300.0f

// PPR per-wheel fallback (dipakai jika NVS belum ada nilai kalibrasi).
#define ENCODER_PPR_FL          408.95f
#define ENCODER_PPR_FR          408.00f
#define ENCODER_PPR_RL          193.70f
#define ENCODER_PPR_RR          195.10f

// Max wheel RPM — roda depan vs belakang punya gearbox beda rasio.
// Roda depan lebih lambat (rasio gearbox lebih tinggi → lebih torsi, lebih lambat).
// Roda belakang lebih cepat (gearbox asli).
// Nilai ini ceiling untuk PID; tuning lebih lanjut bisa via ramp test.
#define MAX_WHEEL_RPM           80.0f    // ceiling aman untuk kedua jenis gearbox
#define MAX_WHEEL_RPM_FRONT     60.0f   // roda depan (gearbox baru, lebih torsi)
#define MAX_WHEEL_RPM_REAR      80.0f   // roda belakang (gearbox lama)

// ─── Motor PWM Deadband ────────────────────────────────────────────────────────
// Min PWM% sebelum roda mulai bergerak.
// Setelah gearbox swap 2026-06-24:
//   Roda depan (gearbox baru, torsi lebih tinggi) → perlu PWM lebih besar untuk mulai
//   Roda belakang (gearbox lama) → sama seperti sebelumnya
#define MOTOR_MIN_PWM_FL        15.0f   // percent — gearbox baru, static friction lebih tinggi
#define MOTOR_MIN_PWM_FR        15.0f
#define MOTOR_MIN_PWM_RL        20.0f
#define MOTOR_MIN_PWM_RR        20.0f

// PENTING: FL/FR pakai gearbox BARU (PPR 408 vs 193 untuk RL/RR).
// Rasio PPR: 408/193 = 2.11x → FL/FR max RPM = RL max / 2.11 = 108/2.11 ≈ 51 RPM.
// Feed-forward HARUS pakai nilai ini, bukan nilai lama (119/122)!
// Kalau nilai ini salah → feed-forward kirim PWM berlebih → PID oscillate → patah-patah!
#define MOTOR_MAX_RPM_FL        55.0f   // FL gearbox baru: 108/2.11 ≈ 51 RPM (pakai 55 sbg margin)
#define MOTOR_MAX_RPM_FR        55.0f   // FR gearbox baru: sama dengan FL
#define MOTOR_MAX_RPM_RL        108.0f  // RL gearbox lama, sudah dikalibrasi ramp test
#define MOTOR_MAX_RPM_RR        110.0f  // RR gearbox lama, sudah dikalibrasi ramp test

// Encoder inversion harus MATCH dengan motor inversion!
// MOTOR_INVERTED xor ENC_INVERTED menentukan apakah feedback positif/negatif:
//   Jika MOTOR_INVERTED=true  → ENC_INVERTED harus false (biar sign RPM benar)
//   Jika MOTOR_INVERTED=false → ENC_INVERTED sesuai orientasi encoder fisik
#define ENC_FL_INVERTED     false   // MOTOR_FL_INVERTED=true → flip encoder supaya sign benar
#define ENC_FR_INVERTED     true    // MOTOR_FR_INVERTED=false → orientasi encoder asli
#define ENC_RL_INVERTED     true    // MOTOR_RL_INVERTED=false → orientasi encoder asli
#define ENC_RR_INVERTED     false   // MOTOR_RR_INVERTED=true → flip encoder supaya sign benar

// Flip these if positive RPM drives a wheel backward.
// 2026-06-24: FL dan RR kebalik setelah gearbox swap → di-invert
#define MOTOR_FL_INVERTED   true    // kebalik — difix
#define MOTOR_FR_INVERTED   false
#define MOTOR_RL_INVERTED   false
#define MOTOR_RR_INVERTED   true    // kebalik — difix

// ─── Servo GPIO (langsung dari ESP32, bukan PCA9685) ──────────────────────────
// Sesuai skematik hardware:
//   Servo1 (S1_PWM) = GPIO12 → lifter gripper DEPAN   (L2 tahan)
//   Servo2 (S2_PWM) = GPIO13 → lifter gripper BELAKANG (R2 tahan)
//   Servo3 (S3_PWM) = GPIO14 → claw gripper DEPAN      (L1 toggle)
//   Servo4 (S4_PWM) = GPIO33 → claw gripper BELAKANG   (R1 toggle)
#define PIN_SERVO_LIFT_FRONT    12
#define PIN_SERVO_LIFT_REAR     13
#define PIN_SERVO_CLAW_FRONT    14
#define PIN_SERVO_CLAW_REAR     33

// ─── Servo Angle Config (TUNING DISINI) ──────────────────────────────────────
// Ubah nilai MIN/MAX sesuai posisi fisik servo di robot (0.0 - 180.0 derajat).
// Kalau servo terpasang terbalik, tukar nilai MIN dan MAX.
//
// AngleClaw_Depan   (Servo3, GPIO14) — buka/tutup gripper DEPAN
#define AngleClaw_Depan_MIN     70.0f   // buka / release
#define AngleClaw_Depan_MAX     125.0f   // tutup / gripping

// AngleClaw_Belakang (Servo4, GPIO33) — buka/tutup gripper BELAKANG
#define AngleClaw_Belakang_MIN  70.0f   // buka / release
#define AngleClaw_Belakang_MAX  125.0f   // tutup / gripping

// AngleLifter_Depan  (Servo1, GPIO12) — naik/turun lifter DEPAN
#define AngleLifter_Depan_MIN   30.0f   // naik / angkat (diubah dari 0.0f agar agak kebawah)
#define AngleLifter_Depan_MAX   120.0f   // turun / drop

// AngleLifter_Belakang (Servo2, GPIO13) — naik/turun lifter BELAKANG
#define AngleLifter_Belakang_MIN 40.0f  // naik / angkat (diubah dari 0.0f agar agak kebawah)
#define AngleLifter_Belakang_MAX 130.0f  // turun / drop

// ─── Anti-Tip / Anti-Tumbling Config (MPU6050 Gyro/Accel) ─────────────────────
// Jika robot miring melebihi sudut batas ini, gripper akan otomatis turun
// menekan tanah untuk mencegah robot terbalik/jatuh.
#define ANTITIP_ENABLED                 true
#define ANTITIP_PITCH_FORWARD_LIMIT     45.0f   // Batas kemiringan jatuh ke depan (derajat)
#define ANTITIP_PITCH_BACKWARD_LIMIT   -45.0f   // Batas kemiringan jatuh ke belakang (derajat)
#define ANTITIP_ROLL_LIMIT              45.0f   // Batas kemiringan jatuh ke samping kiri/kanan (derajat)
#define ANTITIP_ROLL_LIFTER_THROTTLE    0.75f   // Posisi lifter saat jatuh samping (1.0 = UP, 0.0 = DOWN) agar tidak kepentok
#define ANTITIP_ROLL_CLAW_ANGLE         0.0f   // Sudut claw saat membuka untuk menahan jatuh samping (derajat)

#define PIN_V_BATT          32
#define PIN_BUZZER          23
#define PIN_WS2812          18
#define PIN_MPU_INT         5

// Optional distance sensor enable pins.
#define PIN_XSHUT_1         4
#define PIN_XSHUT_2         19

// I2C addresses.
#define ADDR_PCA9685        0x40
#define ADDR_MPU6050        0x68

// PCA9685 -> 4x TB6612FNG channel mapping.
// STBY on every TB6612FNG module is tied to VCC/3V3 in hardware.
#define M1_IN2              0       // Front-Left
#define M1_IN1              1
#define M1_PWM              2

#define M2_IN2              3       // Front-Right
#define M2_IN1              4
#define M2_PWM              5

#define M3_IN2              6       // Rear-Left
#define M3_IN1              7
#define M3_PWM              8

#define M4_IN2              9       // Rear-Right
#define M4_IN1              10
#define M4_PWM              11

#define PCA_SPARE_12        12
#define PCA_SPARE_13        13
#define PCA_SPARE_14        14
#define PCA_SPARE_15        15
