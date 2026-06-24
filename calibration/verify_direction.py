#!/usr/bin/env python3
"""
verify_direction.py — Verifikasi arah & pairing encoder-motor tiap roda.

Cara pakai:
    python calibration/verify_direction.py

Test:
  - Spin tiap motor satu per satu
  - Cek encoder mana yang nambah (pairing test)
  - Cek arah putaran (inverted atau tidak)
  - Jika ada mismatch/swap → laporkan dan minta konfirmasi fix

Tidak otomatis mengubah INVERTED di pin_config.h karena butuh konfirmasi
arah yang "benar" secara fisik (tergantung orientasi roda di robot).
"""

import serial
import time
import re
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT   = 'COM7'
BAUD   = 115200
MOTORS = ['FL', 'FR', 'RL', 'RR']
TEST_RAW = 20000    # ~61% PWM, cukup kuat untuk semua roda

def send(ser, cmd, wait=0.1):
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    out = []
    while ser.in_waiting:
        out.append(ser.readline().decode(errors='replace').strip())
    return out

def read_counts(ser):
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

def test_motor(ser, motor_idx):
    motor = MOTORS[motor_idx]
    print(f"\n  --- {motor} (raw=+{TEST_RAW}) ---")

    # Baca counts sebelum
    c_before = read_counts(ser)
    send(ser, 'raw 0 0 0 0', 0.1)

    # Spin maju (positive raw) 2.5 detik
    vals = [0, 0, 0, 0]; vals[motor_idx] = TEST_RAW
    send(ser, f"raw {' '.join(map(str, vals))}", 0.1)
    time.sleep(2.5)
    c_fwd = read_counts(ser)
    send(ser, 'raw 0 0 0 0', 0.3)
    time.sleep(0.5)

    # Spin mundur (negative raw) 2.5 detik
    vals = [0, 0, 0, 0]; vals[motor_idx] = -TEST_RAW
    send(ser, f"raw {' '.join(map(str, vals))}", 0.1)
    time.sleep(2.5)
    c_bwd = read_counts(ser)
    send(ser, 'raw 0 0 0 0', 0.3)
    time.sleep(0.5)

    # Hitung delta per encoder
    deltas_fwd = {m: abs(c_fwd.get(m, 0) - c_before.get(m, 0)) for m in MOTORS}
    deltas_bwd = {m: abs(c_bwd.get(m, 0) - c_fwd.get(m, 0)) for m in MOTORS}

    # Encoder yang paling banyak berubah = pairing
    primary_fwd = max(deltas_fwd, key=deltas_fwd.get)
    primary_bwd = max(deltas_bwd, key=deltas_bwd.get)

    fwd_delta = deltas_fwd[primary_fwd]
    bwd_delta = deltas_bwd[primary_bwd]

    # Arah encoder (count naik atau turun saat forward)
    fwd_signed = (c_fwd.get(motor, 0) - c_before.get(motor, 0))
    bwd_signed = (c_bwd.get(motor, 0) - c_fwd.get(motor, 0))

    print(f"    Pairing : motor cmd {motor} → encoder {primary_fwd} (delta fwd={fwd_delta})")
    print(f"    Forward signed delta on {motor} encoder: {fwd_signed:+d}")
    print(f"    Backward signed delta on {motor} encoder: {bwd_signed:+d}")

    # Analisis
    pairing_ok = (primary_fwd == motor) and (primary_bwd == motor)
    # Jika positive raw → count bertambah positif → TIDAK inverted (atau inverted sudah tercompensasi)
    direction_positive = fwd_signed > 0

    result = {
        'motor'          : motor,
        'encoder_paired' : primary_fwd,
        'pairing_ok'     : pairing_ok,
        'fwd_delta'      : fwd_signed,
        'bwd_delta'      : bwd_signed,
    }

    status = "OK" if pairing_ok else f"SWAP -> encoder {primary_fwd}"
    print(f"    Status  : {status}")
    if fwd_delta > 10:
        print(f"    Arah   : raw(+{TEST_RAW}) → encoder {'naik ↑' if fwd_signed>0 else 'turun ↓'}")
    else:
        print(f"    WARNING: tidak ada gerakan signifikan! (delta={fwd_delta})")

    return result

def main():
    print("=" * 54)
    print("  WHEEL DIRECTION & ENCODER PAIRING VERIFICATION")
    print("=" * 54)
    print(f"  Port: {PORT}")
    print(f"  Test PWM: {TEST_RAW} ({TEST_RAW/32767*100:.0f}%)")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"ERROR: {e}"); sys.exit(1)

    print("\nMenunggu board stabil (4 detik)...")
    time.sleep(4)
    while ser.in_waiting:
        ser.readline()

    send(ser, 'raw 0 0 0 0', 0.3)
    while ser.in_waiting:
        ser.readline()

    print("\nMulai test... (tiap motor maju 2.5s lalu mundur 2.5s)\n")

    results = []
    for i in range(4):
        r = test_motor(ser, i)
        results.append(r)
        time.sleep(1.0)

    send(ser, 'raw 0 0 0 0', 0.5)
    ser.close()

    # ─── Summary ─────────────────────────────────────────────────────────────
    print(f"\n{'='*54}")
    print(f"  SUMMARY PAIRING & ARAH")
    print(f"{'='*54}")
    print(f"  {'Motor':<6} {'Enc':<6} {'Pairing':<10} {'Fwd delta':<12} {'Bwd delta'}")
    print(f"  {'-'*46}")

    any_issue = False
    for r in results:
        pairing = "OK" if r['pairing_ok'] else f"SWAP->{r['encoder_paired']}"
        if not r['pairing_ok']:
            any_issue = True
        print(f"  {r['motor']:<6} {r['encoder_paired']:<6} {pairing:<10} {r['fwd_delta']:+8d}     {r['bwd_delta']:+8d}")

    if not any_issue:
        print(f"\n  Semua pairing BENAR!")
        print(f"  Note: Jika arah roda terbalik saat robot jalan,")
        print(f"  ubah MOTOR_xx_INVERTED di pin_config.h.")
    else:
        print(f"\n  Ada pairing yang SALAH! Cek wiring encoder.")

    print(f"\n  Selesai.")

if __name__ == '__main__':
    main()
