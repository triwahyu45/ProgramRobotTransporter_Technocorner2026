# 🖨️ PRINTABLE QUICK REFERENCE CARD

## ✂️ CUT & KEEP THIS BY YOUR WORKBENCH

```
╔═══════════════════════════════════════════════════════════════════╗
║         ROBOT CALIBRATION QUICK REFERENCE CARD                   ║
║                                                                   ║
║  Problem: Robot rotates right + jerky movement                  ║
║  Solution: Calibrate RPM + optimize tuning                      ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  STEP 1: Open Serial Monitor                                    ║
║  ────────────────────────────────────────────────────────────── ║
║  • Baud Rate: 115200                                            ║
║  • Tools → Serial Monitor                                       ║
║                                                                   ║
║  STEP 2: Start Calibration                                      ║
║  ────────────────────────────────────────────────────────────── ║
║  Type:  calib                   (show menu)                    ║
║  Type:  1                       (RPM monitor)                  ║
║  Wait:  30 seconds              (collect data)                 ║
║                                                                   ║
║  STEP 3: Record Results                                         ║
║  ────────────────────────────────────────────────────────────── ║
║  Output will be:  RPM: 400/XXX | 400/XXX | 400/XXX | 400/XXX   ║
║                         └─ target/measured for each wheel        ║
║                                                                   ║
║  Record measured values:                                        ║
║  • FL: 400/_______    • FR: 400/_______                         ║
║  • RL: 400/_______    • RR: 400/_______                         ║
║                                                                   ║
║  STEP 4: Calculate Multiplier                                   ║
║  ────────────────────────────────────────────────────────────── ║
║  Formula: Target ÷ Measured = Multiplier                       ║
║                                                                   ║
║  • FL: 400 ÷ _______ = _______                                 ║
║  • FR: 400 ÷ _______ = _______                                 ║
║  • RL: 400 ÷ _______ = _______                                 ║
║  • RR: 400 ÷ _______ = _______                                 ║
║                                                                   ║
║  STEP 5: Apply Multiplier                                       ║
║  ────────────────────────────────────────────────────────────── ║
║  Type:  0                       (stop monitor)                  ║
║  Type:  6 FL:X FR:Y RL:Z RR:W   (apply)                        ║
║  Example: 6 FL:1.01 FR:0.99 RL:1.00 RR:1.00                   ║
║                                                                   ║
║  STEP 6: Test Straight Drive                                    ║
║  ────────────────────────────────────────────────────────────── ║
║  Type:  4 400                   (drive straight)                ║
║  Monitor: Yaw should be < 2° and stable                        ║
║  If stable: ✅ SUCCESS!                                         ║
║  If rotating: Increase failing wheel multiplier                ║
║                                                                   ║
║  STEP 7: Gyro Calibration (Optional but Recommended)            ║
║  ────────────────────────────────────────────────────────────── ║
║  Type:  5                       (start calibration)             ║
║  KEEP ROBOT STILL FOR 30 SECONDS - DO NOT TOUCH!               ║
║  Wait for: "✓ Gyro calibration complete!"                      ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  SERIAL COMMAND REFERENCE                                        ║
╠═══════════════════════════════════════════════════════════════════╣
║  calib               Show calibration menu                       ║
║  1                   RPM Monitor (30 sec collection)             ║
║  2                   Encoder Check (raw ticks)                  ║
║  3 <motor> <rpm>     Test single motor                          ║
║                      Example: 3 0 500 (test FL at 500 RPM)      ║
║  4 <rpm>             Straight drive test                        ║
║                      Example: 4 400 (drive 400 RPM)             ║
║  5                   Gyro calibration                           ║
║  6 FL:x FR:y RL:z    Set wheel multipliers                      ║
║  0                   Stop/Exit                                  ║
║  m, ?                Show menu again                            ║
║                                                                   ║
║  Motor Index: 0=FL (Front Left), 1=FR (Front Right)             ║
║              2=RL (Rear Left),  3=RR (Rear Right)               ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  TROUBLESHOOTING                                                 ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  Problem: Still rotating after calibration                      ║
║  ─────────────────────────────────────────────────────────────── ║
║  Solution: Increase multiplier of slow wheel(s)                 ║
║  Example: If FR still fast, change 0.99 → 0.95                 ║
║                                                                   ║
║  Problem: Motor test shows low measured RPM                     ║
║  ─────────────────────────────────────────────────────────────── ║
║  Solution: Check motor wiring, connections, or motor damaged    ║
║                                                                   ║
║  Problem: Gyro calibration timeout                              ║
║  ─────────────────────────────────────────────────────────────── ║
║  Solution: Robot must be completely STILL on flat surface       ║
║                                                                   ║
║  Problem: Movement still jerky after calibration                ║
║  ─────────────────────────────────────────────────────────────── ║
║  Solution: Already optimized in code (KP: 55, MAX_ACCEL: 900)   ║
║           Just rebuild & upload                                 ║
║                                                                   ║
║  Problem: Motor stalls on ramp                                  ║
║  ─────────────────────────────────────────────────────────────── ║
║  Solution: Try lower RPM for ramp (350-400 instead of 500-600)  ║
║           More RPM = less torque. Lower RPM = more leverage     ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  EXPECTED RESULTS                                                ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  ✅ Robot drives STRAIGHT (Yaw < 2°)                            ║
║  ✅ Movement is SMOOTH (no jerky acceleration)                  ║
║  ✅ Has TORQUE for ramp (no stall)                              ║
║  ✅ Gyro is CALIBRATED (heading stable)                         ║
║  ✅ Ready for COMPETITION! 🏆                                    ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  PRE-COMPETITION CHECKLIST                                       ║
╠═══════════════════════════════════════════════════════════════════╣
║  ☐ Calibration complete (multipliers set)                       ║
║  ☐ Straight drive test: Yaw stable < 2°                         ║
║  ☐ Motor test: No stall on any wheel                            ║
║  ☐ Ramp test: Motor has good torque                             ║
║  ☐ Gyro calibration: Done                                       ║
║  ☐ Movement: Smooth (no jerky)                                  ║
║  ☐ Battery voltage: Min 7.2V                                    ║
║  ☐ Multipliers recorded: FL:___ FR:___ RL:___ RR:___            ║
║  ☐ Practice runs: Done multiple times                           ║
║  ☐ Gripper ready: Yes                                           ║
║                                                                   ║
║  When all ✅: READY FOR COMPETITION! 🎉                         ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  FINAL MULTIPLIER VALUES (Save this!)                           ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  Best Working Multipliers:                                      ║
║  FL: ________________                                            ║
║  FR: ________________                                            ║
║  RL: ________________                                            ║
║  RR: ________________                                            ║
║                                                                   ║
║  Date: ________________                                          ║
║  Conditions: ________________                                    ║
║                                                                   ║
║  (Keep this for next calibration session)                       ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
```

---

## 📱 MOBILE VERSION (for your phone)

```
🤖 ROBOT CALIBRATION CHEAT SHEET

⚙️ STEPS:
1. Serial Monitor (115200 baud)
2. calib → 1 (monitor 30 sec)
3. Record: FL/FR/RL/RR measured RPM
4. Calculate: 400 ÷ measured = multiplier
5. 0 → 6 FL:X FR:Y RL:Z RR:W (apply)
6. 4 400 (straight drive test)
7. Check: Yaw < 2° ✓
8. 5 (gyro calibration)

🎮 COMMANDS:
• calib - menu
• 1 - RPM monitor
• 2 - Encoder check
• 3 M R - test motor M at RPM R
• 4 R - drive straight R rpm
• 5 - gyro calib
• 6 - set multipliers
• 0 - stop

✅ SUCCESS:
• Robot straight (no rotate)
• Movement smooth
• Motor good torque
• Gyro calibrated
```

---

## 🎯 MINI FLOWCHART

```
START
  ↓
Serial Monitor: 115200 ✓
  ↓
Type: calib, 1 (wait 30s)
  ↓
Record measured RPM
  ↓
Calculate: 400 ÷ measured
  ↓
Type: 0, then 6 FL:X FR:Y RL:Z RR:W
  ↓
Type: 4 400 (test)
  ↓
Yaw < 2°?
├─ YES: Type 5 (gyro calib)
├─ NO: Adjust multiplier, retry
  ↓
Gyro done?
├─ YES: SUCCESS ✅
├─ NO: Keep robot still, wait
  ↓
READY FOR COMPETITION! 🏆
```

---

## 📋 CALIBRATION DATA SHEET

**Robot Name/ID**: ______________________  
**Date**: ______________________  
**Battery Voltage**: ______________________  

### Baseline Measurement (Command 1)
| Wheel | Target | Measured | Multiplier |
|-------|--------|----------|-----------|
| FL    | 400    | ______   | _________ |
| FR    | 400    | ______   | _________ |
| RL    | 400    | ______   | _________ |
| RR    | 400    | ______   | _________ |

### After Calibration (Command 4)
| Metric | Value | Status |
|--------|-------|--------|
| Yaw Stability | _____° | ✓/✗ |
| Movement Feel | _____ | ✓/✗ |
| Motor Stall | _____ | ✓/✗ |
| Ramp Test | _____ | ✓/✗ |

### Gyro Calibration (Command 5)
| Item | Status | Notes |
|------|--------|-------|
| Calibration Time | ____s | ✓/✗ |
| Yaw Reading | ____° | ✓/✗ |
| Heading Stable | Y/N | ✓/✗ |

### Final Multipliers (Applied)
```
Command: 6 FL:_______ FR:_______ RL:_______ RR:_______
```

### Notes:
_________________________________________________________________

_________________________________________________________________

---

## 🔔 REMINDER CHECKLIST

Sebelum setiap testing session:
- [ ] Check battery voltage (min 7.2V)
- [ ] Clear serial monitor
- [ ] Start fresh calibration
- [ ] Take notes of multipliers
- [ ] Test multiple times
- [ ] Record best results

---

**GOOD LUCK! 🚀**

