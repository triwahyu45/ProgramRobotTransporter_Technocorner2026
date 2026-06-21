# 🎮 Panduan Kontrol & Pemetaan Tombol Gamepad
**Robot Transporter Technocorner UGM 2026**

Dokumen ini menjelaskan fungsi dari setiap tombol pada gamepad (stik PS4/DualShock) untuk mengendalikan robot transporter menggunakan firmware berbasis ESP32 (Arduino Framework + PlatformIO).

![Panduan Tombol Gamepad](file:///d:/Data_Lokal/Kuliah/Tri%20Wahyu%20(22518241023)/Lomba%20Technocorner%20UGM/02_Transporter/Program%20Robot/Main_Program/gamepad_control_guide.png)

---

## 1. Pemetaan Tombol Utama (Gamepad)

| Tombol / Analog | Kategori | Fungsi Utama | Detail Perilaku |
| :--- | :--- | :--- | :--- |
| **Stick Kiri (L3) - Arah X / Y** | Lokomosi | Pergerakan Holonomic Omnidirectional | Menerjemahkan arah gerak robot secara instan (maju, mundur, geser kiri/kanan, serong). |
| **D-Pad (Atas / Bawah / Kiri / Kanan)** | Lokomosi | Override Arah Gerak Digital (100% Power) | Memindahkan robot ke arah tersebut dengan kecepatan maksimum tanpa batas analog. |
| **Stick Kanan (R3) - Arah X** | Lokomosi | Rotasi Manual | Berputar searah atau berlawanan arah jarum jam. Saat stik dilepas, orientasi terakhir dikunci otomatis (*Yaw Lock*). |
| **L1** | Gripper Depan | Toggle Capitan Depan (Claw) | Menutup / membuka capitan depan secara bergantian (*Edge-triggered*). |
| **L2 (Ditahan)** | Gripper Depan | Angkat Lifter Depan | **Tahan** untuk mengangkat ke atas (`AngleLifter_Depan_MIN` = 0°). **Lepas** untuk menurunkan ke bawah (`AngleLifter_Depan_MAX` = 85°). |
| **R1** | Gripper Belakang | Toggle Capitan Belakang (Claw) | Menutup / membuka capitan belakang secara bergantian (*Edge-triggered*). |
| **R2 (Ditahan)** | Gripper Belakang | Angkat Lifter Belakang | **Tahan** untuk mengangkat ke atas (`AngleLifter_Belakang_MIN` = 0°). **Lepas** untuk menurunkan ke bawah (`AngleLifter_Belakang_MAX` = 85°). |
| **Square (Kotak / X)** | Sistem | Toggle *Yaw Hold / Lock* | Mengaktifkan atau menonaktifkan sistem penguji sudut otomatis (IMU-based). |
| **Cross (Silang / A)** | Sistem | Reset / Zeroing Yaw Target | Mengatur orientasi robot saat ini sebagai sudut target `0` derajat. |
| **Options / Start** | Sistem | Reset / Zeroing Yaw Target | Fungsi alternatif yang sama dengan tombol Cross/Silang untuk zeroing yaw. |
| **Triangle (Segitiga / Y)** | Kecepatan | Batas Kecepatan 37.5% | Mengatur batas kecepatan maksimal pergerakan ke **37.5%** (`speedMultiplier = 0.75`). |
| **Circle (Bulat / B)** | Kecepatan | Batas Kecepatan 20% | Mengatur batas kecepatan maksimal pergerakan ke **20%** (`speedMultiplier = 0.40`). |
| *(Default)* | Kecepatan | Batas Kecepatan 50% | Jika tidak dibatasi tombol Triangle/Circle, kecepatan default adalah **50%** (`speedMultiplier = 1.00`). |

---

## 2. Mekanisme & Konfigurasi Gripper

Sistem gripper dikendalikan langsung menggunakan pin PWM internal ESP32 (bukan chip PCA9685) untuk memastikan respon cepat dan bebas interferensi. Konfigurasi sudut servo didefinisikan dalam [pin_config.h](file:///d:/Data_Lokal/Kuliah/Tri%20Wahyu%20(22518241023)/Lomba%20Technocorner%20UGM/02_Transporter/Program%20Robot/Main_Program/include/pin_config.h).

### A. Gripper Depan (Servo 1 & 3)
* **Servo 3 (GPIO 14) - Claw Depan**:
  - `BUKA / Release` $\rightarrow$ `AngleClaw_Depan_MIN` (Default: `70.0f` derajat)
  - `TUTUP / Gripping` $\rightarrow$ `AngleClaw_Depan_MAX` (Default: `125.0f` derajat)
* **Servo 1 (GPIO 12) - Lifter Depan**:
  - `NAIK / Angkat` $\rightarrow$ `AngleLifter_Depan_MIN` (Default: `30.0f` derajat) - *Dipicu dengan menahan tombol L2*
  - `TURUN / Drop` $\rightarrow$ `AngleLifter_Depan_MAX` (Default: `120.0f` derajat) - *Saat tombol L2 dilepas*

### B. Gripper Belakang (Servo 2 & 4)
* **Servo 4 (GPIO 33) - Claw Belakang**:
  - `BUKA / Release` $\rightarrow$ `AngleClaw_Belakang_MIN` (Default: `70.0f` derajat)
  - `TUTUP / Gripping` $\rightarrow$ `AngleClaw_Belakang_MAX` (Default: `125.0f` derajat)
* **Servo 2 (GPIO 13) - Lifter Belakang**:
  - `NAIK / Angkat` $\rightarrow$ `AngleLifter_Belakang_MIN` (Default: `40.0f` derajat) - *Dipicu dengan menahan tombol R2*
  - `TURUN / Drop` $\rightarrow$ `AngleLifter_Belakang_MAX` (Default: `130.0f` derajat) - *Saat tombol R2 dilepas*

> [!TIP]
> **Cara Tuning Sudut Servo:**
> Jika posisi fisik servo tidak pas atau terbalik, ubah nilai `MIN` dan `MAX` yang sesuai di dalam `include/pin_config.h`. Anda tidak perlu mengedit program utama (`main.cpp`).

---

## 3. Sistem Yaw Hold & Heading Correction (MPU6050)

Robot dilengkapi dengan MPU6050 untuk mempertahankan orientasi heading secara otomatis.
* **Lock Otomatis**: Saat Stick Kanan (R3) dilepas, orientasi robot saat itu otomatis terkunci. Jika robot tertabrak atau tergelincir, roda akan berputar berlawanan untuk mengembalikan orientasi robot ke sudut target.
* **Edge-Triggered Toggle**: Tekan tombol `Square` untuk menghidupkan/mematikan yaw lock. Ini berguna saat melakukan manuver bebas atau kalibrasi.
* **Zeroing Heading**: Tekan `Cross` atau `Start` ketika robot menghadap ke depan lintasan agar orientasi saat itu didefinisikan sebagai sudut 0°.

### 3.1. Sistem Anti-Tip / Anti-Tumbang Otomatis (Gyro-based Self-Righting)
Untuk mencegah robot terbalik atau jatuh saat melewati rintangan/turunan curam, program mendeteksi kemiringan sudut robot secara real-time:
* **Jatuh ke Depan**: Jika sudut Pitch robot melebihi `ANTITIP_PITCH_FORWARD_LIMIT` (Default: `45.0` derajat), **gripper DEPAN** akan dipaksa turun penuh ke bawah untuk menahan sasis agar tidak terjungkal ke depan.
* **Jatuh ke Belakang**: Jika sudut Pitch robot kurang dari `ANTITIP_PITCH_BACKWARD_LIMIT` (Default: `-45.0` derajat), **gripper BELAKANG** akan dipaksa turun penuh ke bawah untuk menahan sasis agar tidak terjungkal ke belakang.
* **Jatuh ke Samping (Kiri/Kanan)**: Jika sudut Roll robot melebihi `ANTITIP_ROLL_LIMIT` (Default: `45.0` derajat), **kedua gripper (depan dan belakang)** akan dipaksa turun sekaligus untuk menstabilkan robot agar tidak terguling ke samping.

> [!NOTE]
> Fitur ini aktif secara otomatis bahkan jika Anda tidak menekan tombol L2/R2, dan akan melepas penekanan kembali ke posisi atas ketika robot sudah stabil/rata. Sudut batas kemiringan dapat Anda ubah pada bagian `#define ANTITIP_...` di dalam `include/pin_config.h`.

---

## 4. Perintah Debug & Telemetri Lewat Serial Monitor

Selain gamepad, robot dapat dipantau dan dikendalikan lewat kabel USB menggunakan serial terminal (baudrate `115200`).

### Perintah Kalibrasi & Konfigurasi
* `calib imu` : Kalibrasi giroskop IMU (Pastikan robot diam dan rata selama kalibrasi).
* `calib enc` : Reset perhitungan jarak encoder dan PID kecepatan roda.
* `calib all` : Menjalankan kalibrasi IMU dan reset encoder sekaligus.
* `drive closed` : Mengaktifkan kontrol berbasis lingkar tertutup (Closed-loop RPM) menggunakan encoder.
* `drive open` / `drive raw` : Menonaktifkan encoder PID dan kembali ke mode manual PWM langsung.
* `telemetry on` / `telemetry off` : Mengaktifkan atau menonaktifkan cetak log status robot secara langsung di serial monitor.
* `yaw on` / `yaw off` : Mengaktifkan atau menonaktifkan koreksi yaw otomatis.
* `pid [kp] [ki] [kd]` : Mengubah nilai konstanta PID yaw (contoh: `pid 1.25 0 0.04`).

### Perintah Gerak Manual (Untuk Pengujian)
* `stop` : Mengerem paksa semua motor.
* `zero` : Reset encoder dan arah hadap target ke 0.
* `move [x] [y] [turn]` : Menggerakkan robot dengan persentase kekuatan (contoh: `move 0 30 0` untuk maju 30%).
* `rpm [fl] [fr] [rl] [rr]` : Menjalankan masing-masing roda pada RPM tertentu (Closed-loop harus aktif).
* `rawall [pwm]` : Memberikan sinyal PWM mentah yang sama ke seluruh roda (contoh: `rawall 15000`).
