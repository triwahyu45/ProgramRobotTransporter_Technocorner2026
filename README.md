# Main_Program

Program utama robot Transporter Technocorner UGM 2026.

## Struktur

```text
include/pin_config.h
lib/EncoderReader/
lib/ImuManager/
lib/PcaMotorDriver/
lib/Kinematics/
lib/WheelSpeedController/
src/main.cpp
```

## Hardware V1

- ESP32 DOIT DevKit V1.
- PCA9685 khusus untuk sinyal 4x TB6612FNG.
- 1 modul TB6612FNG untuk 1 motor.
- STBY TB6612FNG dijumper langsung ke VCC/3V3.
- Servo MG996R tidak lewat PCA motor; servo pakai PWM ESP32 langsung.
- Encoder motor pakai VCC 3V3 dan masuk GPIO ESP32 langsung.

## API Motor

Ada dua mode:

1. Raw PWM, tetap memakai range `-32767..32767`.
2. Closed-loop RPM, memakai encoder per roda supaya speed lebih konstan.

```cpp
DriveFL(value);  // front-left
DriveFR(value);  // front-right
DriveRL(value);  // rear-left
DriveRR(value);  // rear-right

DriveAll(fl, fr, rl, rr);
BrakeAll();
CoastAll();
```

Nilai:

```text
-32767 = full reverse
0      = short brake
32767  = full forward
```

API RPM:

```cpp
DriveFLRpm(rpm);
DriveFRRpm(rpm);
DriveRLRpm(rpm);
DriveRRRpm(rpm);
SetWheelTargetRpm(fl, fr, rl, rr);
DriveRobotPercent(x, y, turn);
StopRpmDrive();
```

`SetWheelTargetRpm()` bukan direct PWM. Ini target RPM masing-masing roda, jadi tiap roda boleh punya angka berbeda sesuai hasil kinematic.

`DriveRobotPercent()` adalah command level robot. Masukkan arah gerak `x`, `y`, dan `turn` dalam persen `-100..100`; program akan mix ke target RPM FL/FR/RL/RR lalu encoder PID menjaga speed tiap roda.

Target RPM otomatis dibatasi `MAX_WHEEL_RPM` dari `include/pin_config.h`, default awal `1000 RPM`.

## PCA9685 Channel Mapping

```text
FL/A: CH0 IN2, CH1 IN1,  CH2 PWM
FR/B: CH3 IN2, CH4 IN1,  CH5 PWM
RL/C: CH6 IN2, CH7 IN1,  CH8 PWM
RR/D: CH9 IN2, CH10 IN1, CH11 PWM
```

## Encoder

Pin encoder:

```text
FL/ENC1: GPIO16 / GPIO17
FR/ENC2: GPIO25 / GPIO26
RL/ENC3: GPIO34 / GPIO35
RR/ENC4: GPIO36 / GPIO39
```

`GPIO34/35/36/39` adalah input-only dan tidak punya pull-up internal. Kalau encoder output open-collector, tambahkan pull-up eksternal ke 3V3.

Nilai `ENCODER_COUNTS_PER_REV` masih default awal. Pakai project `../Encoder_Test` untuk cek count real satu putaran roda, lalu update nilai itu.

## Serial Command

## Gamepad Control

Panduan lengkap mengenai tombol stik dapat dibaca di: **[KONTROL_ROBOT.md](file:///d:/Data_Lokal/Kuliah/Tri%20Wahyu%20(22518241023)/Lomba%20Technocorner%20UGM/02_Transporter/Program%20Robot/Main_Program/KONTROL_ROBOT.md)**.

Alur kontrol:
```text
Gamepad -> Bluepad32 -> yaw hold MPU -> kinematic kita -> PCA9685 -> TB6612FNG
```

Default `Main_Program` memakai raw/open-loop PCA untuk test movement + MPU, supaya robot tetap jalan walau encoder belum tervalidasi. Mode encoder RPM tetap tersedia lewat `drive closed`.

### Ringkasan Pemetaan Tombol:

```text
Left stick Y  : Maju/Mundur
Left stick X  : Strafe Kiri/Kanan
D-pad         : Override Arah Gerak Digital (100% Power)
Right stick X : Rotasi Manual
Cross / Start : Zero Yaw / Reset Angle ke 0
Square        : Toggle Yaw Lock ON/OFF
Triangle      : Speed Limit 37.5% (Speed multiplier 75%)
Circle        : Speed Limit 20% (Speed multiplier 40%)
Default Speed : Speed Limit 50% (Speed multiplier 100%)

L1            : Toggle Claw/Capitan Depan (Buka/Tutup)
L2 (Ditahan)  : Angkat Lifter Depan ke Atas (Lepas untuk menurunkan)
R1            : Toggle Claw/Capitan Belakang (Buka/Tutup)
R2 (Ditahan)  : Angkat Lifter Belakang ke Atas (Lepas untuk menurunkan)
```

Saat right stick dilepas, heading terakhir langsung jadi angle lock. Saat semua analog nol, robot tetap mengoreksi kalau badan diputar selama yaw lock aktif. Tekan `Square` untuk mematikan/menyalakan yaw lock saat developing.


Command tambahan:

```text
pid 1.15 0 0.035     ubah yaw PID sementara
field on/off         mode field-centric eksperimental
yaw on/off           aktif/nonaktif yaw hold
yawidle on/off       aktif/nonaktif yaw hold saat analog nol
yawdir normal/invert balik arah koreksi yaw kalau robot menjauh dari target
drive closed/open    closed-loop encoder atau raw PCA fallback
telemetry on/off     aktif/nonaktif print telemetry serial
calib imu            kalibrasi gyro, robot harus diam
calib enc            reset count encoder dan PID roda
calib all            calib imu + calib enc
enc                  print count/RPM/command encoder
```

```text
h                    help
p                    print config
stop                 brake
zero                 reset encoder + yaw
move 0 40 0          maju 40 persen lewat mode drive aktif
move 40 0 0          strafe/right 40 persen lewat mode drive aktif
move 0 0 25          rotate 25 persen lewat mode drive aktif
rpm 120 80 120 80    target RPM manual FL FR RL RR
raw 12000 0 0 0      raw PWM debug per roda
rawall 12000         raw PWM sama ke semua roda
```

Test idle yaw lock:

```text
1. Upload Main_Program, robot dibuat aman dulu dengan roda bebas.
2. Tekan Start atau kirim zero untuk set heading target.
3. Jangan sentuh analog, putar robot pelan dengan tangan.
4. Kalau yaw error > 2 derajat, roda harus mencoba balik pelan.
5. Kalau arah koreksi malah menjauh, kirim yawdir invert.
```

## IMU

IMU dibaca lewat raw register di alamat `0x68`, bukan lagi driver DMP MPU6050 khusus. Ini karena modul yang kamu punya terdeteksi `WHO_AM_I = 0x72`, jadi library MPU6050 murni akan menolak walaupun I2C scan menemukan `0x68`.

Saat boot, robot harus diam dan rata sebentar karena kalibrasi gyro berjalan otomatis secara non-blocking. Yaw sekarang berupa integrasi gyro Z, jadi bisa drift pelan, tapi cukup untuk yaw-hold pendek saat test gerak.
