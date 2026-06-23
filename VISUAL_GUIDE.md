# 🎨 VISUAL REFERENCE GUIDE

## 🤖 ROBOT PROBLEM & SOLUTION

### **PROBLEM: Robot Rotate Right + Jerky Movement**

```
┌──────────────────────────────────┐
│  PROBLEM SYMPTOMS                │
├──────────────────────────────────┤
│                                  │
│  User Input:    "Maju lurus!"    │
│        ↓                         │
│  Robot Path:    ↗ (rotasi kanan) │
│        ↓                         │
│  Movement:     jerky, tersendat  │
│        ↓                         │
│  Nanjak Ramp:   Motor stall      │
│                                  │
└──────────────────────────────────┘
```

### **ROOT CAUSES**

```
Cause 1: RPM Unbalanced
┌─────────────────────────────┐
│  FL: 400 RPM target         │
│  FR: 410 RPM measured (fast)│  ← Lebih cepat
│  RL: 400 RPM                │
│  RR: 400 RPM                │
└─────────────────────────────┘
Result: Rotate kanan karena FR lebih cepat

Cause 2: KP Too High (70)
┌──────────────────────────────┐
│  Error small ─→ Big correction│  ← Overshoot
│  Then settle back             │  ← Settle
│  Then overshoot again         │  ← Repeat
└──────────────────────────────┘
Result: Jerky movement

Cause 3: RPM Too High (Nanjak)
┌──────────────────────────────┐
│  High RPM = Low torque        │  Motor cannot push hard
│  Motor slips on ramp          │  Cannot climb
└──────────────────────────────┘
Result: Stall on ramp
```

---

## ✅ SOLUTION IMPLEMENTED

### **Solution 1: Wheel Multiplier Calibration**

```
BEFORE:
  FL: 400 target → 395 measured (slow)
  FR: 400 target → 410 measured (fast) ← Problem!
  RL: 400 target → 398 measured (slow)
  RR: 400 target → 401 measured (normal)

CALCULATION:
  Multiplier = Target / Measured
  FR_multiplier = 400 / 410 = 0.976

AFTER:
  FR: 400 target × 0.976 = 390.4 command
      390.4 command → 410 measured × 0.976 = 400 effective
  Result: All wheels = 400 RPM effectively!
```

### **Solution 2: KP Optimization (55 instead of 70)**

```
KP = 70 (Aggressive)           KP = 55 (Smooth)
───────────────────────────   ──────────────────────────
│                             │
│ ↗ Overshoot (big jump)      │ Smooth curve up
│ ↓ Settle                    │ ↗ Gradual climb
│ ↗ Overshoot again           │ Reaches target
│ → Result: Jerky             │ → Result: Smooth
│                             │
```

### **Solution 3: Smooth Acceleration (MAX_ACCEL = 900)**

```
MAX_ACCEL = 1,000,000 RPM/s    MAX_ACCEL = 900 RPM/s
(Instant jump)                 (Gradual ramp)
───────────────────────────    ──────────────────────────
│                              │
│ ┐ Instant to target          │ ┌ Gradual climb
│ │ (jerky start)              │ │ (smooth start)
└─┘                            └─────────
```

---

## 🎮 COMMAND USAGE VISUALIZATION

### **Flow Chart: How to Calibrate**

```
START
  │
  ├─→ Type "calib" → Show menu ✓
  │
  ├─→ Type "1" → RPM Monitor starts
  │    │
  │    ├─→ See: "RPM: 400/395 | 400/410 | 400/398 | 400/401"
  │    │    (target / measured for each wheel)
  │    │
  │    └─→ Type "0" → Stop monitoring
  │
  ├─→ Calculate multipliers:
  │    FL: 400/395 = 1.013
  │    FR: 400/410 = 0.976  ← Key correction!
  │    RL: 400/398 = 1.005
  │    RR: 400/401 = 0.998
  │
  ├─→ Type "6 FL:1.013 FR:0.976 RL:1.005 RR:0.998"
  │    → Apply multipliers
  │
  ├─→ Type "4 400" → Straight drive test
  │    │
  │    ├─→ See: "Yaw: 0.1°" (stable!)
  │    │
  │    └─→ Type "0" → Stop
  │
  ├─→ Type "5" → Gyro calibration
  │    │
  │    ├─→ Keep robot STILL for 30 seconds
  │    │
  │    └─→ See: "✓ Gyro calibration complete!"
  │
  └─→ DONE! Robot ready! ✅
```

---

## 📊 SERIAL MONITOR OUTPUT EXAMPLES

### **Example 1: RPM Monitor Output**

```
>>> Command: 1
► RPM Monitor started. Press '0' to exit.
  Columns: FL(RPM) | FR(RPM) | RL(RPM) | RR(RPM) | Yaw(deg)

RPM: 400/395 | 400/402 | 400/398 | 400/401 | Yaw: 2.3°
RPM: 400/396 | 400/403 | 400/399 | 400/400 | Yaw: 2.5°
RPM: 400/397 | 400/402 | 400/398 | 400/401 | Yaw: 2.8°

Format: Target/Measured | Target/Measured | ...
```

### **Example 2: Motor Test Output**

```
>>> Command: 3 1 500
► Testing motor FR at 500 RPM. Press '0' to exit.

RPM: 0/0 | 500/48 | 0/0 | 0/0 | Yaw: 0.0°
RPM: 0/0 | 500/156 | 0/0 | 0/0 | Yaw: 0.0°
RPM: 0/0 | 500/298 | 0/0 | 0/0 | Yaw: 0.0°
RPM: 0/0 | 500/425 | 0/0 | 0/0 | Yaw: 0.0°
RPM: 0/0 | 500/498 | 0/0 | 0/0 | Yaw: 0.0°

Analysis: Motor FR accelerates smoothly from 0→500 RPM ✓
```

### **Example 3: Multiplier Application**

```
>>> Command: 6 FL:1.013 FR:0.976 RL:1.005 RR:0.998
Multipliers updated: FL:1.013 FR:0.976 RL:1.005 RR:0.998
```

### **Example 4: After Calibration - Straight Drive**

```
>>> Command: 4 400
► Straight drive at 400 RPM (with multipliers). Press '0' to exit.
  Columns: FL(M) | FR(M) | RL(M) | RR(M) | Yaw(d) | Time(s)

RPM: 400/400 | 400/410 | 400/398 | 400/401 | Yaw: 0.1° | Time: 0.0s
RPM: 400/399 | 400/410 | 400/399 | 400/400 | Yaw: 0.0° | Time: 0.3s
RPM: 400/400 | 400/410 | 400/398 | 400/401 | Yaw: 0.1° | Time: 0.6s

Analysis: Yaw stable at ~0° (with multipliers applied) ✓
```

---

## 🎯 EXPECTED IMPROVEMENTS

### **Before vs After Comparison**

```
┌─────────────────────────────────────────────────────┐
│  METRIC          BEFORE         AFTER               │
├─────────────────────────────────────────────────────┤
│  Straight Line   ↗ (rotate)     → (straight) ✓     │
│  Yaw Drift      ±10°/10s        ±0-2°/10s ✓        │
│  Acceleration    Jerky          Smooth ✓            │
│  Ramp Climb     Stall           Success ✓           │
│  Torque (low)    Weak           Strong ✓            │
│  KP Gain        70 (aggr)       55 (optim) ✓        │
│  Multipliers    None (1,1,1,1)  Calibrated ✓       │
└─────────────────────────────────────────────────────┘
```

---

## 🔍 DIAGNOSTIC FLOWCHART

```
Robot not moving straight?
    │
    ├─→ Check RPM Monitor (command 1)
    │   │
    │   ├─→ All RPM ≈ same?
    │   │   NO: Apply wheel multiplier (command 6)
    │   │   YES: Go next step
    │   │
    │   └─→ Yaw drifts?
    │       YES: Gyro calibration (command 5)
    │       NO: Go next step
    │
    ├─→ Test individual motors (command 3)
    │   │
    │   ├─→ Any motor respond slow?
    │       YES: Motor/gear problem → Replace
    │       NO: All motors OK
    │
    ├─→ Straight drive test (command 4)
    │   │
    │   ├─→ Yaw stable < 2°?
    │       YES: CALIBRATION SUCCESS ✓
    │       NO: Repeat steps
    │
    └─→ [If still failing]
        Check battery voltage, encoder wiring, motor connection
```

---

## 📱 SERIAL MONITOR CHEAT SHEET

```
╔═══════════════════════════════════════════════════════╗
║         SERIAL MONITOR QUICK REFERENCE               ║
╠═══════════════════════════════════════════════════════╣
║ calib                     → Show calibration menu    ║
║ m, ?                      → Show menu again          ║
║ 1                         → RPM Monitor              ║
║ 2                         → Encoder Check            ║
║ 3 <motor> <rpm>           → Test single motor        ║
║   Example: 3 1 500        → Test FR motor at 500 RPM ║
║ 4 <rpm>                   → Straight drive           ║
║   Example: 4 400          → Drive 400 RPM            ║
║ 5                         → Gyro calibration         ║
║ 6 FL:x FR:y RL:z RR:w     → Set multipliers          ║
║   Example: 6 FL:1.01 FR:0.99 RL:1.00 RR:1.00        ║
║ 0                         → Stop/Exit                ║
╚═══════════════════════════════════════════════════════╝

Motor index: 0=FL, 1=FR, 2=RL, 3=RR
Baud Rate: 115200
```

---

## 🎬 VIDEO WALKTHROUGH (Text Version)

### **Scene 1: Initial Problem**

```
Robot: Halo! 👋
User:  Maju lurus dong!
Robot: OK! *bergerak ke kanan* ↗️
User:  Kenapa ke kanan?? 😫
```

### **Scene 2: Diagnosis**

```
User:  Type "calib" → "1"
       Monitor: "RPM: 400/395 | 400/410 | 400/398 | 400/401"
User:  Ah! FR too fast (410 vs 395)! 🔍
```

### **Scene 3: Solution**

```
User:  Type "6 FL:1.013 FR:0.976 RL:1.005 RR:0.998"
       Apply multipliers ✓
```

### **Scene 4: Verification**

```
User:  Type "4 400"
       Monitor: "Yaw: 0.1° | Yaw: 0.0° | Yaw: 0.1°"
       Yaw stabil! ✅
```

### **Scene 5: Victory**

```
Robot: Masuk! 🏆
User:  Yaaaay! 🎉
```

---

## 🏁 STEP-BY-STEP VISUAL

```
STEP 1: RPM MONITOR              STEP 2: CALCULATE MULTIPLIER
┌─────────────────┐             ┌──────────────────────┐
│ 1. Type "calib" │             │ 400/395 = 1.013      │
│ 2. Type "1"     │             │ 400/410 = 0.976 ⬅️   │
│                 │             │ 400/398 = 1.005      │
│ See:            │             │ 400/401 = 0.998      │
│ 400/395 │400... │             └──────────────────────┘
│ 400/410 │400... │
│ 400/398 │400... │
│ 400/401 │400... │             STEP 3: APPLY
└─────────────────┘             ┌──────────────────────┐
                                │ Type:                │
                                │ 6 FL:1.013 ...       │
                                │ 6 FR:0.976 ...       │
                                │ 6 RL:1.005 ...       │
                                │ 6 RR:0.998 ...       │
                                └──────────────────────┘

STEP 4: VERIFY                  STEP 5: GYRO CALIBRATE
┌─────────────────┐             ┌──────────────────────┐
│ Type "4 400"    │             │ Type "5"             │
│                 │             │ Wait 30 seconds      │
│ See:            │             │ (robot STILL!)       │
│ Yaw: 0.1°       │             │                      │
│ Yaw: 0.0°  ✓    │             │ Done: "✓ complete!" │
│ Yaw: 0.1°       │             └──────────────────────┘
└─────────────────┘

Result: Robot maju LURUS! 🎉
```

---

**For more details, see:**
- QUICK_START.md - Fast execution (recommended)
- CALIBRATION_GUIDE.md - Complete reference
- SMOOTH_TORQUE_OPTIMIZATION.md - Advanced tuning

