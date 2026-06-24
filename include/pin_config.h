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
// Ramp calibration 2026-06-24: FL=1273, FR=1254, RL=2806, RR=2816 RPM @ 100% PWM
// Bottleneck = FR (1254 RPM). Pakai 1260 sebagai ceiling WheelSpeedController.
#define MAX_WHEEL_RPM           1263.0f  // ceiling: limited by FR front wheel
#define MAX_WHEEL_RPM_FRONT     1260.0f  // FL=1273, FR=1254 → pakai 1260
#define MAX_WHEEL_RPM_REAR      2815.0f  // RL=2806, RR=2816 → pakai 2815

// ─── Motor PWM Deadband ────────────────────────────────────────────────────────
// Min PWM% sebelum roda mulai bergerak.
// Hasil ramp calibration 2026-06-24 (ZK-5AD channel fix, 12V, kosong):
//   FL deadband 19.8% (raw=6500), FR 24.4% (raw=8000)
//   RL deadband 19.8% (raw=6500), RR 24.4% (raw=8000)
#define MOTOR_MIN_PWM_FL        18.3f   // ramp test 2026-06-24: 19.8%
#define MOTOR_MIN_PWM_FR        22.9f   // ramp test 2026-06-24: 24.4%
#define MOTOR_MIN_PWM_RL        19.8f   // ramp test 2026-06-24: 19.8%
#define MOTOR_MIN_PWM_RR        22.9f   // ramp test 2026-06-24: 24.4%

// Ramp calibration 2026-06-24 (ZK-5AD channel mapping fix):
//   FL: 1273 RPM @ 100%, FR: 1254 RPM @ 100%
//   RL: 2806 RPM @ 100%, RR: 2816 RPM @ 100%
// Rasio RL/FL = 2806/1273 = 2.21x → sesuai rasio gearbox depan (lebih torsi, lebih lambat)
#define MOTOR_MAX_RPM_FL        1275.0f  // ramp test 2026-06-24
#define MOTOR_MAX_RPM_FR        1263.0f  // ramp test 2026-06-24
#define MOTOR_MAX_RPM_RL        2817.0f  // ramp test 2026-06-24
#define MOTOR_MAX_RPM_RR        2832.0f  // ramp test 2026-06-24

// !! PERHATIAN - ENCODER GPIO LIMITATIONS:
// FL (GPIO 16/17) dan FR (GPIO 25/26): punya internal pull-up → RELIABLE
// RL (GPIO 34/35): input-only, bisa external pull-up tapi SERING noise EMI dari motor
// RR (GPIO 36/39): input-only, TIDAK BISA pull-up apapun → paling tidak reliable
//
// Rule: ENC_INVERTED HARUS kebalikan dari MOTOR_INVERTED!
//   MOTOR_INVERTED=true  + ENC_INVERTED=false → feedback positif ✓
//   MOTOR_INVERTED=false + ENC_INVERTED=true  → feedback positif ✓
//   Kalau sama-sama true/false → NEGATIF FEEDBACK → PID RUNAWAY!
#define ENC_FL_INVERTED     false   // FL: MOTOR_INVERTED=true  → ENC harus false
#define ENC_FR_INVERTED     false   // FR: MOTOR_INVERTED=true  → ENC harus false
#define ENC_RL_INVERTED     true    // RL: MOTOR_INVERTED=false → ENC harus true
#define ENC_RR_INVERTED     true    // RR: MOTOR_INVERTED=false → ENC harus true

// Flip motor jika positive command bikin roda mundur.
// FL dan FR: gearbox baru → arah terbalik → di-invert
// RR: arah terbalik (mounting/wiring) → di-invert
// RL: arah benar → tidak di-invert
#define MOTOR_FL_INVERTED   true    // gearbox baru, arah terbalik
#define MOTOR_FR_INVERTED   true    // gearbox baru (PPR=408), arah terbalik
#define MOTOR_RL_INVERTED   false   // gearbox lama, arah benar
#define MOTOR_RR_INVERTED   false   // DIBALIK 2026-06-24: channel fix → arah berubah

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
#define AngleClaw_Depan_MIN     85.0f   // buka / release (tuned)
#define AngleClaw_Depan_MAX     108.0f  // tutup / gripping (tuned, anti-stall)

// AngleClaw_Belakang (Servo4, GPIO33) — buka/tutup gripper BELAKANG
#define AngleClaw_Belakang_MIN  85.0f   // buka / release (tuned)
#define AngleClaw_Belakang_MAX  118.0f  // tutup / gripping (tuned, anti-stall)

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

// PCA9685 → 2x ZK-5AD (TA6586) channel mapping.
// ZK-5AD hanya butuh 2 pin per motor (D0/D1 = Motor A, D2/D3 = Motor B).
// Driver depan (ZK-5AD #1): CH0-CH3
//   D0=CH0, D1=CH1 → Motor Front-Left
//   D2=CH2, D3=CH3 → Motor Front-Right
// Driver belakang (ZK-5AD #2): CH4-CH7
//   D0=CH4, D1=CH5 → Motor Rear-Left
//   D2=CH6, D3=CH7 → Motor Rear-Right
//
// TIDAK ADA pin PWM/ENA terpisah! Speed dikontrol via duty cycle di IN1 atau IN2.
// PCA_DUMMY_PWM = channel 12 (spare, tidak tersambung) dipakai sebagai placeholder.
#define PCA_DUMMY_PWM       12  // spare channel, tidak tersambung ke apapun

#define M1_IN2              0       // Front-Left  (D0 driver depan)
#define M1_IN1              1       //             (D1 driver depan)
#define M1_PWM              PCA_DUMMY_PWM

#define M2_IN2              2       // Front-Right (D2 driver depan)
#define M2_IN1              3       //             (D3 driver depan)
#define M2_PWM              PCA_DUMMY_PWM

#define M3_IN2              4       // Rear-Left   (D0 driver belakang)
#define M3_IN1              5       //             (D1 driver belakang)
#define M3_PWM              PCA_DUMMY_PWM

#define M4_IN2              6       // Rear-Right  (D2 driver belakang)
#define M4_IN1              7       //             (D3 driver belakang)
#define M4_PWM              PCA_DUMMY_PWM

#define PCA_SPARE_13        13
#define PCA_SPARE_14        14
#define PCA_SPARE_15        15

