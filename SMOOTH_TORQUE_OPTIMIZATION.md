# OPTIMASI SMOOTH MOVEMENT + TORQUE UNTUK NANJAK

## 🎯 TARGET
- ✅ Gerak maju lurus (no rotate)
- ✅ Movement smooth (tidak tersendat)
- ✅ Torque besar untuk nanjak ramp
- ✅ Fast enough untuk ancang sebelum nanjak

---

## 🔧 REKOMENDASI PERUBAHAN CODE

### **1. TUNING PID UNTUK SMOOTH (di WheelSpeedController.cpp)**

**Current KP=70 terlalu agresif → turunkan ke 50-60 untuk smooth**

```cpp
// File: lib/WheelSpeedController/WheelSpeedController.cpp
// Baris: constexpr float KP = 70.0f;

// BEFORE:
constexpr float KP = 70.0f;        // Aggressive
constexpr float KI = 6.0f;
constexpr float INTEGRAL_LIMIT = 2400.0f;

// AFTER (untuk smooth):
constexpr float KP = 55.0f;        // Smooth response
constexpr float KI = 6.0f;
constexpr float INTEGRAL_LIMIT = 2400.0f;
```

**Jika masih tidak smooth, turun ke:**
```cpp
constexpr float KP = 45.0f;        // Extra smooth (tapi responsivity berkurang)
```

**Jika perlu response cepat sambil smooth:**
```cpp
constexpr float KP = 60.0f;
constexpr float KI = 8.0f;         // Naik sedikit untuk integral push
```

---

### **2. ACCELERATION RAMP UNTUK SMOOTH START**

Saat motor dimulai dari 0 RPM → target 600 RPM, butuh smooth acceleration, bukan langsung jump.

**Current di code:**
```cpp
constexpr float MAX_ACCEL_RPM_PER_SEC = 1000000.0f;  // TERLALU CEPAT!
```

**Update untuk smooth:**
```cpp
constexpr float MAX_ACCEL_RPM_PER_SEC = 800.0f;      // Naik 800 RPM per detik
// Jadi 0→600 RPM butuh 0.75 detik (smooth!)

// Jika terlalu smooth, naik ke:
constexpr float MAX_ACCEL_RPM_PER_SEC = 1200.0f;     // 600 RPM dalam 0.5 detik
```

**Edit baris di WheelSpeedController.cpp:**
```cpp
// BEFORE:
constexpr float MAX_ACCEL_RPM_PER_SEC = 1000000.0f;

// AFTER:
constexpr float MAX_ACCEL_RPM_PER_SEC = 800.0f;      // Smooth acceleration
```

---

### **3. MOTOR MAX RPM UNTUK TORQUE**

Saat nanjak, butuh torque banyak. Cara dapat torque = **kurangi kecepatan, naikkan arus**.

**Di pin_config.h atau main.cpp, ada:**
```cpp
constexpr float MANUAL_DRIVE_RPM = 1000.0f;      // Joystick drive
constexpr float MANUAL_ROTATE_RPM = 500.0f;      // Joystick rotate
```

**Untuk nanjak dengan torque, gunakan:**
```cpp
constexpr float RAMP_APPROACH_RPM = 350.0f;      // Slow approach for max torque
constexpr float RAMP_CLIMB_RPM = 400.0f;         // Climb speed (masih ada torque)
```

---

### **4. DEADZONE COMPENSATION UNTUK SMOOTH CREEP**

Jika motor tidak bergerak saat RPM rendah (deadzone), naik minimum PWM:

**Current:**
```cpp
constexpr float DEADZONE_COMP_FULL_RPM = 350.0f;
constexpr float DEADZONE_COMP_MIN_FRACTION = 0.80f;  // 80% dari min PWM
```

**Untuk better low-speed torque:**
```cpp
constexpr float DEADZONE_COMP_FULL_RPM = 400.0f;     // Naikkan threshold
constexpr float DEADZONE_COMP_MIN_FRACTION = 0.90f;  // 90% untuk lebih kuat
```

---

## 🎮 SERIAL COMMANDS UNTUK TESTING

### **Test Setup 1: Smooth Straight Drive**

```bash
# Start monitoring RPM
1                          # RPM Monitor

# Clear previous targets
0                          # Stop

# Drive smooth 300 RPM
4 300

# Expected: Smooth acceleration 0→300 RPM dalam 0.375 detik
#           RPM measured harus smooth naik, bukan jumpy
```

### **Test Setup 2: Maximum Torque (Ramp Climb)**

```bash
# Stop first
0

# Drive 400 RPM (medium speed for max torque)
4 400

# Monitor RPM semua roda harus sama
# Jika ada yaw drift, perlu wheel balance lagi
```

### **Test Setup 3: Motor Test Individual**

```bash
# Jika ada motor stall, test satu per satu
0              # Stop

3 0 400        # Test FL at 400 RPM
# Catat: harus naik smooth

3 1 400        # Test FR
3 2 400        # Test RL
3 3 400        # Test RR

# Jika ada motor yang tidak naik smooth → motor cacat
```

---

## 📝 STEP-BY-STEP OPTIMIZATION

### **Hari 1: Baseline + Balance**

```bash
# 1. Cek RPM balance
calib
1                    # Monitor 30 detik

# 2. Catat result → hitung multiplier
# 3. Apply multiplier
6 FL:X FR:Y RL:Z RR:W

# 4. Verifikasi drive straight
4 400
# Yaw harus stabil di ±5 derajat
```

### **Hari 2: Smooth + Torque**

```bash
# 1. Edit WheelSpeedController.cpp:
#    - Turun KP dari 70 ke 55
#    - Turun MAX_ACCEL dari 1M ke 800

# 2. Compile & upload

# 3. Test smooth movement
4 300           # 300 RPM (should be smooth)
4 500           # 500 RPM (fast but smooth)

# 4. If still jerky → turun KP lebih
#    If too slow → naik MAX_ACCEL
```

### **Hari 3: Ramp Practice**

```bash
# 1. Approach ramp slowly
4 350           # Slow approach

# 2. Check torque
#    Motor harus strong, tidak slip

# 3. Kalau ada slip detection
#    - Turun RPM lebih (300)
#    - Atau naik motor voltage (ganti battery)
```

---

## 🐛 TROUBLESHOOTING

| Problem | Penyebab | Solusi |
|---------|---------|--------|
| Gerak tersendat-sendat | KP terlalu tinggi | Turun KP (70→50) |
| Terlalu lambat | MAX_ACCEL terlalu rendah | Naik MAX_ACCEL (800→1200) |
| Rotate kanan/kiri | Wheel unbalance | Gunakan multiplier (Step 3 di guide) |
| Motor stall saat nanjak | RPM terlalu tinggi | Turun target RPM (500→400) |
| Motor tidak start | Deadzone compensation | Naik DEADZONE_COMP_FULL_RPM |
| Motor panas | Motor overload | Turun RPM atau naik voltage |

---

## 📊 RECOMMENDED VALUES (untuk lomba)

```cpp
// SMOOTH + TORQUE BALANCED:
constexpr float KP = 55.0f;
constexpr float KI = 6.0f;
constexpr float MAX_ACCEL_RPM_PER_SEC = 900.0f;

// RAMP APPROACH:
constexpr float RAMP_SPEED = 350.0f;  // Slow for torque

// NORMAL DRIVE:
constexpr float NORMAL_SPEED = 500.0f;  // Fast approach
```

---

## 🚀 CHECKLIST SEBELUM LOMBA

- [ ] RPM balanced (multiplier sudah di-tune)
- [ ] Smooth movement (no jerky acceleration)
- [ ] Gyro calibrated (yaw stable)
- [ ] Ramp test OK (motor tidak stall)
- [ ] Battery voltage OK (min 7.2V)
- [ ] Motor tidak panas saat idle
- [ ] Encoder reading akurat
- [ ] Gripper ready untuk grab

**SEMOGA SUKSES DI LOMBA! 🏆**
