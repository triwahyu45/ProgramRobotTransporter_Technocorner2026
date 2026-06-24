#!/usr/bin/env python3
"""
calibrate_rpm.py — Kalibrasi deadband & max RPM tiap roda robot.

Cara pakai:
    python calibration/calibrate_rpm.py

Setelah selesai:
  1. Hasil ditampilkan di layar
  2. Nilai otomatis diupdate ke include/pin_config.h
  3. Firmware otomatis dikompile & diupload ke ESP32

Persyaratan:
  - pip install pyserial
  - platformio terinstall (pio command tersedia)
  - Robot terhubung ke COM7 (ubah PORT di bawah jika beda)
"""

import serial
import time
import re
import sys
import io
import os
import subprocess
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# ─── Konfigurasi ─────────────────────────────────────────────────────────────
PORT        = 'COM7'
BAUD        = 115200
PROJECT_DIR = Path(__file__).parent.parent          # root project (satu level di atas calibration/)
PIN_CONFIG  = PROJECT_DIR / 'include' / 'pin_config.h'
PPR         = {'FL': 408.95, 'FR': 408.00, 'RL': 193.70, 'RR': 195.10}
MOTORS      = ['FL', 'FR', 'RL', 'RR']

# Deadband threshold: RPM > ini = motor sudah bergerak
DEADBAND_RPM_THRESHOLD = 5.0
# ─────────────────────────────────────────────────────────────────────────────


def send(ser, cmd, wait=0.08):
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    out = []
    while ser.in_waiting:
        out.append(ser.readline().decode(errors='replace').strip())
    return out


def read_counts(ser):
    """Baca encoder counts via 'enc' command. Return dict {FL,FR,RL,RR: count}."""
    while ser.in_waiting:
        ser.readline()
    ser.write(b'enc\n')
    time.sleep(0.3)
    counts = {}
    deadline = time.time() + 0.8
    while time.time() < deadline:
        if ser.in_waiting:
            l = ser.readline().decode(errors='replace').strip()
            m = re.match(r'^(FL|FR|RL|RR)\s+count=(-?\d+)', l)
            if m:
                counts[m.group(1)] = int(m.group(2))
    return counts


def compute_rpm(delta_count, elapsed_sec, motor):
    ppr = PPR[motor]
    revolutions = abs(delta_count) / ppr
    minutes     = elapsed_sec / 60.0
    return revolutions / minutes if minutes > 0 else 0.0


def raw_cmd(motor_idx, value):
    vals = [0, 0, 0, 0]
    vals[motor_idx] = int(value)
    return f"raw {vals[0]} {vals[1]} {vals[2]} {vals[3]}"


def calibrate_motor(ser, motor_idx):
    motor = MOTORS[motor_idx]
    ppr   = PPR[motor]
    print(f"\n{'='*54}")
    print(f"  Kalibrasi {motor}  (PPR={ppr})")
    print(f"{'='*54}")

    results      = []
    deadband_raw = None
    deadband_pct = None
    max_rpm      = 0.0
    max_raw      = 0

    steps = (
        list(range(2000,  8000,  500)) +
        list(range(8000,  20000, 1000)) +
        list(range(20000, 32767, 2000)) +
        [32767]
    )

    for raw_val in steps:
        pct = raw_val / 32767 * 100

        # Spin motor, tunggu stabil
        send(ser, raw_cmd(motor_idx, raw_val), 0.05)
        time.sleep(0.5)

        # Ukur RPM: snapshot sebelum & sesudah 1 detik
        c0 = read_counts(ser);  t0 = time.time()
        time.sleep(1.0)
        c1 = read_counts(ser);  t1 = time.time()

        elapsed = t1 - t0
        delta   = abs(c1.get(motor, 0) - c0.get(motor, 0))
        rpm     = compute_rpm(delta, elapsed, motor)

        results.append((raw_val, pct, rpm))
        marker = ''
        if deadband_raw is None and rpm > DEADBAND_RPM_THRESHOLD:
            deadband_raw = raw_val
            deadband_pct = pct
            marker = '  <<< DEADBAND'
        if rpm > max_rpm:
            max_rpm = rpm
            max_raw = raw_val

        print(f"    raw={raw_val:5d} ({pct:5.1f}%)  delta={delta:6d}  RPM={rpm:7.1f}{marker}")

    # Stop motor
    send(ser, 'raw 0 0 0 0', 0.3)
    time.sleep(0.5)

    print(f"\n  >> {motor}: Deadband={deadband_pct:.1f}%  MaxRPM={max_rpm:.0f}")
    return deadband_pct, max_rpm


def update_pin_config(summary):
    """Update nilai di pin_config.h secara otomatis."""
    print(f"\n{'='*54}")
    print(f"  Update pin_config.h ...")
    print(f"{'='*54}")

    with open(PIN_CONFIG, encoding='utf-8') as f:
        content = f.read()

    replacements = {
        'MOTOR_MIN_PWM_FL': f"{summary['FL']['deadband']:.1f}f",
        'MOTOR_MIN_PWM_FR': f"{summary['FR']['deadband']:.1f}f",
        'MOTOR_MIN_PWM_RL': f"{summary['RL']['deadband']:.1f}f",
        'MOTOR_MIN_PWM_RR': f"{summary['RR']['deadband']:.1f}f",
        'MOTOR_MAX_RPM_FL': f"{summary['FL']['max_rpm']:.0f}.0f",
        'MOTOR_MAX_RPM_FR': f"{summary['FR']['max_rpm']:.0f}.0f",
        'MOTOR_MAX_RPM_RL': f"{summary['RL']['max_rpm']:.0f}.0f",
        'MOTOR_MAX_RPM_RR': f"{summary['RR']['max_rpm']:.0f}.0f",
    }

    # Update MAX_WHEEL_RPM: ambil min dari FL dan FR (front gearbox bottleneck)
    max_wheel_rpm = min(summary['FL']['max_rpm'], summary['FR']['max_rpm'])
    replacements['MAX_WHEEL_RPM'] = f"{max_wheel_rpm:.0f}.0f"

    for define, new_val in replacements.items():
        # Match: #define DEFINE_NAME    <old_value>f   // optional comment
        pattern     = rf'(#define\s+{define}\s+)[\d.]+f'
        replacement = rf'\g<1>{new_val}'
        new_content, n = re.subn(pattern, replacement, content)
        if n:
            print(f"  {define:25s} = {new_val}")
            content = new_content
        else:
            print(f"  WARNING: {define} tidak ditemukan di pin_config.h!")

    with open(PIN_CONFIG, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"\n  pin_config.h updated OK.")


def upload_firmware():
    """Compile & upload via PlatformIO."""
    print(f"\n{'='*54}")
    print(f"  Compile & Upload firmware ...")
    print(f"{'='*54}")

    cmd = ['pio', 'run', '--target', 'upload', '--upload-port', PORT]
    print(f"  Menjalankan: {' '.join(cmd)}")
    print()

    result = subprocess.run(
        cmd,
        cwd=str(PROJECT_DIR),
        capture_output=False,   # tampilkan output langsung
        text=True
    )

    if result.returncode == 0:
        print("\n  UPLOAD SUKSES!")
    else:
        print(f"\n  UPLOAD GAGAL (exit={result.returncode})")
    return result.returncode == 0


def main():
    print("=" * 54)
    print("  ZK-5AD RPM CALIBRATION")
    print("  Otomatis update pin_config.h + upload firmware")
    print("=" * 54)
    print(f"  Port      : {PORT}")
    print(f"  PPR       : FL={PPR['FL']} FR={PPR['FR']} RL={PPR['RL']} RR={PPR['RR']}")
    print(f"  Config    : {PIN_CONFIG}")
    print()

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"ERROR: Tidak bisa buka {PORT}: {e}")
        sys.exit(1)

    print("Menunggu board stabil (4 detik)...")
    time.sleep(4)
    while ser.in_waiting:
        ser.readline()

    send(ser, 'raw 0 0 0 0', 0.3)
    while ser.in_waiting:
        ser.readline()

    # ─── Sweep semua motor ───────────────────────────────────────────────────
    summary = {}
    for i, motor in enumerate(MOTORS):
        db, max_rpm = calibrate_motor(ser, i)
        if db is None:
            print(f"  WARNING: {motor} deadband tidak terdeteksi! Gunakan nilai lama.")
            db = 20.0   # fallback
        summary[motor] = {'deadband': round(db, 1), 'max_rpm': round(max_rpm)}
        time.sleep(1.0)

    send(ser, 'raw 0 0 0 0', 0.5)
    ser.close()

    # ─── Tampilkan summary ────────────────────────────────────────────────────
    print(f"\n{'='*54}")
    print(f"  HASIL KALIBRASI")
    print(f"{'='*54}")
    print(f"  {'Motor':<6} {'Deadband%':<12} {'MaxRPM':<10}")
    print(f"  {'-'*28}")
    for m, r in summary.items():
        print(f"  {m:<6} {r['deadband']:<12.1f} {r['max_rpm']:<10.0f}")

    # ─── Update pin_config.h ──────────────────────────────────────────────────
    update_pin_config(summary)

    # ─── Compile + Upload ─────────────────────────────────────────────────────
    ok = upload_firmware()

    if ok:
        print(f"\n{'='*54}")
        print("  SELESAI! Firmware sudah terupdate dengan kalibrasi terbaru.")
        print(f"{'='*54}")
    else:
        print("\n  Cek error di atas. pin_config.h sudah diupdate, tapi upload gagal.")
        print("  Coba: pio run --target upload --upload-port COM7")

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
