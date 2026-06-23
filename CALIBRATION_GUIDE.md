# PANDUAN KALIBRASI ROBOT

## 🎯 Masalah: Robot Rotasi Kanan Saat Maju Lurus

Penyebab utama: **Ketidakseimbangan RPM antar roda** atau **Gyro offset yang tidak tepat**

---

## 🔧 LANGKAH-LANGKAH KALIBRASI

### **STEP 1: Monitor RPM Per Roda (Cek Keseimbangan)**

**Tujuan**: Lihat apakah semua roda memiliki RPM yang sama saat diberi perintah yang sama.

**Di Serial Monitor, ketik:**
```
calib        # Tampilkan menu
1            # Pilih RPM Monitor
```

**Output akan seperti:**
```
RPM: 400/395 | 400/402 | 400/398 | 400/401 | Yaw: 2.3°
RPM: 400/396 | 400/403 | 400/399 | 400/400 | Yaw: 2.5°
...
```

**Format**: `Target/Measured | Target/Measured | Target/Measured | Target/Measured`

**Analisis**:
- Jika FL & FR jauh berbeda = Roda depan tidak seimbang
- Jika RL & RR jauh berbeda = Roda belakang tidak seimbang
- Jika Robot terus rotate (Yaw naik/turun) = Ada ketidakseimbangan

---

### **STEP 2: Test Motor Individual (Cek Motor Cacat)**

**Tujuan**: Tes setiap motor satu per satu untuk cek apakah ada yang bermasalah.

**Di Serial Monitor, ketik:**
```
3 0 500      # Test motor 0 (FL) pada 500 RPM
```

Motor index:
- `0` = FL (Front Left)
- `1` = FR (Front Right)
- `2` = RL (Rear Left)
- `3` = RR (Rear Right)

**Contoh sequence:**
```
3 0 500    # Test FL -> catat RPM measured
3 1 500    # Test FR -> bandingkan dengan FL
3 2 500    # Test RL -> bandingkan dengan FL
3 3 500    # Test RR -> bandingkan dengan FL
```

**Jika salah satu motor:**
- Target: 500 RPM, Measured: 200 RPM → Motor lemah/cacat
- Target: 500 RPM, Measured: -50 RPM → Motor reversed

---

### **STEP 3: Kalibrasi Wheel Balance (Sesuaikan Multiplier)**

**Tujuan**: Sesuaikan kecepatan roda agar saat maju lurus tidak rotate.

**Rumus sederhana:**
```
multiplier_baru = target_rpm / measured_rpm
```

**Contoh**: Dari Step 1 hasil monitoring:
```
FL: 400/395 → multiplier = 400/395 = 1.013
FR: 400/402 → multiplier = 400/402 = 0.995
RL: 400/398 → multiplier = 400/398 = 1.005
RR: 400/401 → multiplier = 400/401 = 0.998
```

**Kalibrasi:**
```
6 FL:1.013 FR:0.995 RL:1.005 RR:0.998
```

**Verifikasi:**
```
4 400        # Drive straight 400 RPM
```

Jika Yaw tetap stabil (tidak naik/turun) = Berhasil! ✓

---

### **STEP 4: Kalibrasi Gyro/IMU**

**Tujuan**: Kalibrasi offset gyro agar yaw measurement akurat.

**Di Serial Monitor, ketik:**
```
5            # Start gyro calibration
```

**Penting:**
- ⚠️ **Letakkan robot di tempat datar dan JANGAN DISENTUH**
- Tunggu sampai muncul pesan: `✓ Gyro calibration complete!`

---

## 📊 COMMAND REFERENCE

| Command | Tujuan |
|---------|--------|
| `calib` atau `m` | Tampilkan menu kalibrasi |
| `1` | RPM Monitor (real-time) |
| `2` | Encoder Check (raw values) |
| `3 <motor> <rpm>` | Test single motor |
| `4 <rpm>` | Straight drive test |
| `5` | Gyro calibration |
| `6 FL:X FR:Y RL:Z RR:W` | Set wheel multipliers |
| `0` | Stop/Exit calibration |

---

## 🎬 CONTOH WORKFLOW LENGKAP

### **Skenario: Robot rotate kanan saat maju**

```bash
# 1. Cek keseimbangan RPM
calib
1                          # Monitor RPM
# Biarkan 10 detik, lihat pattern → catat hasil

# 2. Test motor individual
0                          # Stop monitoring
3 0 500                    # Test FL
# Catat: FL measured = 395 RPM
3 1 500                    # Test FR  
# Catat: FR measured = 410 RPM ← LEBIH CEPAT (ini penyebab rotate kanan!)

# 3. Hitung multiplier
# FL: 400/395 = 1.013
# FR: 400/410 = 0.976 ← Kurangi ini!

# 4. Apply multiplier
0                          # Stop test
6 FL:1.013 FR:0.976 RL:1.00 RR:1.00

# 5. Verifikasi
4 400                      # Drive straight
# Monitor Yaw → seharusnya stabil sekarang

# 6. Kalibrasi Gyro (opsional tapi recommended)
5                          # Start gyro calib
# Tunggu selesai
```

---

## ⚡ TIPS UNTUK SMOOTH MOVEMENT

### **Masalah: Gerak tersendat-sendat**

1. **Naik voltage motor** → Motor akan lebih responsif
2. **Turun KP PID** (dari 70 ke 60) untuk smooth:
   ```
   pid 60 6 0   # KP=60, KI=6, KD=0
   ```
3. **Smooth acceleration**: Sudah ada di code (MAX_ACCEL_RPM_PER_SEC)

### **Masalah: Motor stall saat nanjak**

1. **Naik target RPM** untuk dapat torque lebih:
   ```
   4 600        # 600 RPM instead of 400
   ```
2. **Check gearing ratio** di motor

---

## 💾 MENYIMPAN KONFIGURASI

Setelah kalibrasi berhasil, multiplier **TIDAK otomatis tersimpan**.  
Anda perlu:

1. Catat multiplier akhir yang bekerja baik
2. **Update di code** (jika ingin permanent) atau
3. **Simpan manual** di Preferences (NVS)

---

## 📱 SERIAL MONITOR SETTINGS

- **Baud Rate**: 115200
- **Line Ending**: Newline (`\n`)

---

## 🚀 NEXT STEPS

1. **Selesaikan calibration** sesuai panduan di atas
2. **Test straight drive** dengan `4 500`
3. **Jika berhasil** → Ready untuk lomba! 🏆
4. **Jika masih rotate** → Cek:
   - Motor cacat/rusak?
   - Gearbox tidak seimbang?
   - Encodernya baca dengan benar?

**Good luck! 💪**
