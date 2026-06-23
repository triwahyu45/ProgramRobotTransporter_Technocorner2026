# ⚡ QUICK START KALIBRASI - STEP BY STEP

## 🎯 MASALAH YANG INGIN DIPECAHKAN

✗ Robot rotate kanan saat maju lurus  
✗ Gerak tidak smooth (tersendat)  
✗ Motor tidak punya torque buat nanjak  

**Solusi**: Kalibrasi RPM + Gyro + Smooth tuning  

---

## 📋 QUICK CHECKLIST (30 MENIT)

- [ ] **5 min**: RPM monitoring & balance
- [ ] **5 min**: Motor test individual  
- [ ] **5 min**: Apply wheel multiplier
- [ ] **5 min**: Gyro calibration
- [ ] **5 min**: Test straight drive + ramp

---

## 🚀 EKSEKUSI CEPAT

### **1. CONNECT SERIAL MONITOR** (Baud 115200)

```
Arduino IDE → Tools → Serial Monitor
```

---

### **2. MONITOR RPM (5 menit)**

**Ketik di Serial Monitor:**

```
calib
1
```

**Output:**
```
RPM: 400/395 | 400/402 | 400/398 | 400/401 | Yaw: 2.3°
RPM: 400/396 | 400/403 | 400/399 | 400/400 | Yaw: 2.5°
RPM: 400/398 | 400/401 | 400/397 | 400/402 | Yaw: 2.8°
```

**Catat hasil → LANJUT STEP 3**

---

### **3. HITUNG MULTIPLIER** (2 menit)

**Rumus:**
```
multiplier = target_rpm / measured_rpm
```

**Contoh dari output Step 2:**
```
FL: 400 / 398 = 1.005
FR: 400 / 401 = 0.998
RL: 400 / 397 = 1.008
RR: 400 / 402 = 0.995
```

---

### **4. APPLY MULTIPLIER** (30 detik)

**Ketik di Serial Monitor:**

```
0                                              # Stop monitoring
6 FL:1.005 FR:0.998 RL:1.008 RR:0.995        # Apply
```

**Output:**
```
Multipliers updated: FL:1.005 FR:0.998 RL:1.008 RR:0.995
```

---

### **5. VERIFY STRAIGHT DRIVE** (2 menit)

**Ketik:**
```
4 400
```

**Monitor output:**
```
RPM: 400/399 | 400/400 | 400/399 | 400/401 | Yaw: 0.2°
RPM: 400/400 | 400/400 | 400/400 | 400/400 | Yaw: 0.1°
RPM: 400/399 | 400/400 | 400/399 | 400/400 | Yaw: 0.0°
```

**✅ Jika Yaw tetap 0-1° = BERHASIL! Robot tidak rotate lagi!**

---

### **6. GYRO CALIBRATION** (5 menit)

**Ketik:**
```
0           # Stop drive
5           # Start gyro calibration
```

**⚠️ PENTING: JANGAN DISENTUH ROBOT SAMPAI SELESAI**

**Output:**
```
Gyro Calib: 45 samples | GX:-0.02 GY:0.01 GZ:-0.01 | Y:0.2 P:-0.1 R:0.0
Gyro Calib: 90 samples | GX:0.00 GY:0.00 GZ:0.00 | Y:0.1 P:0.0 R:0.0
✓ Gyro calibration complete!
```

---

### **7. FINAL TEST - SMOOTH MOVEMENT** (3 menit)

**Ketik:**
```
1           # Monitor RPM
```

**Catat behavior:**
- Smooth acceleration? Y/N
- Motor jerky? Y/N
- Ramp climb OK? Y/N

**Jika smooth ✅ = BERHASIL!**  
**Jika jerky ✗ = Go to OPTIMIZATION section di file terpisah**

---

## 📊 HASIL YANG DIHARAPKAN

| Metric | Target | Status |
|--------|--------|--------|
| RPM Balance Error | < 5 RPM | ✓ Checked |
| Yaw Drift (straight) | < 2°/10s | ✓ Checked |
| Acceleration | Smooth | ✓ Checked |
| Motor Stall | None | ✓ Checked |

---

## 💾 CATATAN HASIL KALIBRASIKU

**Tanggal**: ___________

**1. RPM Baseline** (sebelum multiplier):
```
FL: ___/400   FR: ___/400   RL: ___/400   RR: ___/400
```

**2. Multiplier yang diapply**:
```
FL: _____   FR: _____   RL: _____   RR: _____
```

**3. Setelah multiplier** (measured):
```
FL: ___/400   FR: ___/400   RL: ___/400   RR: ___/400
```

**4. Yaw stability** (saat drive straight):
```
Initial: ____° | After 5s: ____° | Drift: ____°
```

**5. Motor behavior**:
- [ ] Smooth start
- [ ] No stall
- [ ] Good torque
- [ ] Can climb ramp

---

## 🐛 PROBLEM? LIHAT INI

**Q: Command "1" not responding?**  
A: Type "calib" dulu, baru "1"

**Q: RPM tetap tidak balance setelah multiplier?**  
A: Motor mungkin rusak. Test individual dengan "3 0 500"

**Q: Gyro calib stuck?**  
A: Robot bergerak/bergetar. Letakkan di meja rata yang stabil

**Q: Masih rotate kanan?**  
A: Multiplier tidak cukup. Perlu increase lebih:
```
6 FL:1.02 FR:0.97 RL:1.02 RR:0.97  # Naikkan FL/RL, turun FR/RR
```

---

## ✨ NEXT LEVEL (OPTIONAL)

Setelah step basic selesai:

1. **TUNING PID** untuk extra smooth:
   - Buka WheelSpeedController.cpp
   - Ubah KP dari 55 ke 50 untuk lebih smooth
   - Compile & upload

2. **PRACTICE RAMP DRIVE**:
   ```
   4 350      # Slow approach for max torque
   ```

3. **SPEED TEST**:
   ```
   4 600      # Fast drive
   4 800      # Max speed
   ```

---

## 🏆 SEKARANG SIAP LOMBA!

Kalau sudah semua green ✅ → Rebuild robot, practice run, **GO WIN!** 🎉

**Questions? Check:**
- `CALIBRATION_GUIDE.md` (detailed)
- `SMOOTH_TORQUE_OPTIMIZATION.md` (advanced tuning)

