# AI Agent Changelog

## 2026-06-24 — Session 1: Initial tuning & heading lock fix

### Masalah
- Robot terlalu pelan, heading tidak lock, osilasi/putar-putar

### Perubahan
- `main.cpp`:
  - MAX_YAW_LOCK_TURN_PERCENT 18→40, IDLE_YAW_MAX_TURN_PERCENT 10→25
  - GYRO_STILL_DPS 1.2→2.5 (kemudian 0.8→1.2)
  - LPF 0.90→0.65, KP 55→40 (kemudian kembali 55)
  - CLOSED_LOOP_DRIVE_RPM_SCALE 0.50→0.75 (kemudian 0.60)
  - Fix `idleYawHoldOrBraze()`: removed yawTargetDeg=imu.yawDeg saat error>deadband
- Yaw PID diset via tune version: kp=1.2 ki=0.003 kd=0.06

### Hasil
- ❌ Heading masih tidak lock, osilasi

## 2026-06-24 — Session 2: Revert parsial & rekonstruksi

### Insiden
- `git checkout -- src/main.cpp lib/WheelSpeedController/WheelSpeedController.cpp lib/ImuManager/ImuManager.cpp` mengembalikan ke git HEAD (versi lawas 1133 line) BUKAN ke versi kerja user (2446 line)

### Rekonstruksi
- `ImuManager.cpp`: GYRO_Z_LPF_ALPHA=0.90, GYRO_Z_INTEGRATE_DEADBAND_DPS=0.9, manual calibration toggle
- `WheelSpeedController.cpp`: KP=55, KI=6, INTEGRAL_LIMIT=2400, CLOSED_LOOP_DRIVE_RPM_SCALE=0.50, wheel multipliers, slip detection
- `main.cpp`: ⚠️ BELUM direkonstruksi — masih 1284 line (git HEAD), hilang ~1162 line (CalibrationSystem, progressive accel, enhanced gamepad, serial commands, dll)

### Fix
- `platformio.ini`: tambah `-Ilib/PcaMotorDriver` + Adafruit PWM Servo Driver include path

### Hasil
- ✅ Build sukses (40.4% flash, 33.4% RAM)
- ✅ Upload sukses
- ❌ Disconnect masih berbahaya (robot tetap jalan)

### Perbaikan disconnect
- `stopWithBrake()`: sekarang force gripper ke safe position (claw tutup, lifter naik)
- Disconnect check: tambah `hasData()` — stop jika controller connected tapi tidak kirim data
- `updateControllerLed()`: LED stik warna berdasarkan level baterai (merah/kuning/hijau)

## Status saat ini
- **Fungsi kontrol dasar jalan**: omni drive, heading lock, PID yaw, wheel multipliers
- **Hilang dari original user**: CalibrationSystem, progressive accel, enhanced gamepad triggers, beberapa serial commands, web server config mode, anti-tip dll.
- **File original user (2446-line main.cpp) tidak bisa direcover** — tidak pernah di-commit ke git, VSS cuma untuk C: drive
