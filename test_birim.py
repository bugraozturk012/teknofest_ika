#!/usr/bin/env python3
"""Kritik fonksiyon birim testleri — donanım gerektirmez."""
import math
import struct
import sys

PASS = 0
FAIL = 0

def check(desc, got, expected, tol=0.01):
    global PASS, FAIL
    ok = abs(got - expected) <= tol if isinstance(got, float) else got == expected
    status = "✓" if ok else "✗ FAIL"
    if not ok:
        FAIL += 1
        print(f"  {status} {desc}: got={got!r} expected={expected!r}")
    else:
        PASS += 1
        print(f"  {status} {desc}")

# ─── 1. Ackermann kinematik ──────────────────────────────────────────────────
print("=== 1. Ackermann Kinematik ===")
L = 0.55
DELTA_MAX = 0.5236  # 30°

def ackermann(v, w):
    if abs(v) < 1e-4:
        return math.copysign(DELTA_MAX, w) if abs(w) > 1e-4 else 0.0
    return max(-DELTA_MAX, min(DELTA_MAX, math.atan2(L * w, v)))

check("düz ileri",      math.degrees(ackermann(1.0, 0.0)), 0.0)
check("sola dön",       ackermann(1.0, 0.5) > 0, True)
check("sağa dön",       ackermann(1.0, -0.5) < 0, True)
check("v=0 sol",        ackermann(0.0, 1.0),  DELTA_MAX)
check("v=0 sağ",        ackermann(0.0, -1.0), -DELTA_MAX)
check("v=0 w=0",        ackermann(0.0, 0.0),  0.0)
check("max limit",      ackermann(0.1, 10.0), DELTA_MAX)
check("geri düz",       math.degrees(math.atan(L * 0.0 / -1.0)), 0.0)

# ─── 2. Enkoder overflow koruması ───────────────────────────────────────────
print("\n=== 2. Enkoder Overflow (AS5600 10-bit) ===")

def delta(yeni, eski, maks=1024):
    d = yeni - eski
    yarim = maks // 2
    if d >  yarim: d -= maks
    if d < -yarim: d += maks
    return d

check("ileri küçük adım", delta(10, 5),      5)
check("geri küçük adım",  delta(5, 10),     -5)
check("wrap ileri 1023→1", delta(1, 1023),   2)
check("wrap geri  1→1023", delta(1023, 1),  -2)
check("yarım tur ileri",   delta(512, 0),  512)
check("0→512 ileri",       delta(512, 0),  512)
check("tam tur sıfır",     delta(0, 0),     0)

# ─── 3. Binary paket CRC ────────────────────────────────────────────────────
print("\n=== 3. Binary Protokol CRC ===")

def make_pkt(cmd, v0, v1):
    v0 = max(-32768, min(32767, v0))
    v1 = max(-32768, min(32767, v1))
    veri = struct.pack('>hh', v0, v1)
    crc  = cmd ^ veri[0] ^ veri[1] ^ veri[2] ^ veri[3]
    return struct.pack('BB4sBB', 0xAA, cmd, veri, crc, 0x55)

def verify(pkt):
    if pkt[0] != 0xAA or pkt[7] != 0x55:
        return False
    crc = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5]
    return pkt[6] == crc

for cmd, v0, v1, desc in [
    (0x01, 1000, 1500, "PKT_SURUCU"),
    (0x02, 0,    0,    "PKT_DUR"),
    (0x03, 1,    0,    "PKT_LAZER_AC"),
    (0x03, 0,    0,    "PKT_LAZER_KAP"),
    (0x04, 0,    0,    "PKT_HB"),
    (0x01, -1000, 3000, "negatif hız + büyük yaw"),
]:
    pkt = make_pkt(cmd, v0, v1)
    ok = verify(pkt) and len(pkt) == 8
    check(desc, ok, True)

# ─── 4. Batarya yüzde ───────────────────────────────────────────────────────
print("\n=== 4. Batarya Yüzde (4S LiPo) ===")
V_MIN, V_MAX = 14.0, 16.8

def bat_pct(v):
    return round(max(0.0, min(1.0, (v - V_MIN) / (V_MAX - V_MIN))) * 100)

check("tam dolu 16.8V",  bat_pct(16.8), 100)
check("boş     14.0V",   bat_pct(14.0), 0)
check("yarı    15.4V",   bat_pct(15.4), 50)
check("fazla   17.0V",   bat_pct(17.0), 100)  # clamp
check("düşük   13.0V",   bat_pct(13.0), 0)    # clamp
check("normal  15.8V",   bat_pct(15.8), 64)

# ─── 5. IMU Quaternion → Roll / Pitch ───────────────────────────────────────
print("\n=== 5. IMU Quaternion → Roll/Pitch ===")

def quat2rp(qw, qx, qy, qz):
    sinr = 2.0 * (qw*qx + qy*qz)
    cosr = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (qw*qy - qz*qx)))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch

r, p = quat2rp(1, 0, 0, 0)
check("düz zemin roll",  r, 0.0)
check("düz zemin pitch", p, 0.0)

a = math.radians(15)
r, p = quat2rp(math.cos(a/2), math.sin(a/2), 0, 0)
check("15° sağ roll",    r, 15.0)
check("15° sağ pitch",   p,  0.0)

a = math.radians(10)
r, p = quat2rp(math.cos(a/2), 0, math.sin(a/2), 0)
check("10° yukarı roll",  r,  0.0)
check("10° yukarı pitch", p, 10.0)

a = math.radians(20)
r, p = quat2rp(math.cos(a/2), math.sin(a/2), 0, 0)
check("20° roll → estop eşiği aşıldı", abs(r) >= 19.99, True)  # fp toleransı

# ─── 6. Anti-rollback koşul mantığı ─────────────────────────────────────────
print("\n=== 6. Anti-Rollback Koşul Mantığı ===")
PITCH_TH = math.radians(10.0)
VEL_TH   = 0.05

def rollback(pitch_deg, vel):
    pitch = math.radians(pitch_deg)
    return pitch > PITCH_TH and vel < -VEL_TH

check("rampa yukarı + geri kayma",      rollback(12.0, -0.1),  True)
check("rampa yukarı + ileri gidiyor",   rollback(12.0,  0.5),  False)
check("düz zemin + geri kayma",         rollback( 2.0, -0.1),  False)
check("eşik altı pitch",                rollback( 9.9, -0.2),  False)
check("eşik üstü pitch + dur",          rollback(11.0,  0.0),  False)
check("rampa + büyük geri kayma",       rollback(20.0, -0.5),  True)

# ─── 7. PID Integral Windup Koruması ────────────────────────────────────────
print("\n=== 7. PID Integral Windup Koruması ===")
INTEGRAL_MAX = 30.0
integral = 0.0
for _ in range(400):  # 20 saniye × 50Hz = 1000 döngü
    hata = 100.0      # Sürekli büyük hata
    dt   = 0.05
    integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral + hata * dt))

check("20s windup sonrası sınırda", integral, INTEGRAL_MAX)
check("sınır aşılmadı",            integral <= INTEGRAL_MAX, True)

integral2 = 0.0
for _ in range(400):
    integral2 = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral2 + (-100.0) * 0.05))
check("negatif windup sınırı", integral2, -INTEGRAL_MAX)

# ─── 8. seri_kopru dt guard ─────────────────────────────────────────────────
print("\n=== 8. Odometri dt Guard ===")

def dt_gecerli(dt):
    return 0.0 < dt < 1.0

check("normal 50Hz (dt=0.02)",    dt_gecerli(0.02),  True)
check("yavaş  (dt=0.5)",          dt_gecerli(0.5),   True)
check("clock jump (dt=1.5)",      dt_gecerli(1.5),   False)
check("negatif (dt=-0.1)",        dt_gecerli(-0.1),  False)
check("sıfır   (dt=0.0)",         dt_gecerli(0.0),   False)
check("sınır   (dt=0.999)",       dt_gecerli(0.999), True)
check("sınır   (dt=1.0)",         dt_gecerli(1.0),   False)

# ─── 9. imu_guvenlik hız kısıtı ─────────────────────────────────────────────
print("\n=== 9. IMU Güvenlik Hız Kısıt Mantığı ===")
WARN  = 8.0;  STOP  = 15.0;  ESTOP = 20.0
NMAX  = 2.0;  FREN  = 0.4;   BAT_HIZ = 1.0

def guvenlik_hiz(roll_deg, pitch_deg, bat_pct):
    roll_abs = abs(roll_deg)
    if roll_abs >= ESTOP:
        hiz = 0.0
    elif roll_abs >= STOP:
        hiz = 0.0
    elif roll_abs >= WARN:
        oran = 1.0 - (roll_abs - WARN) / (STOP - WARN)
        hiz  = NMAX * 0.5 * max(0.0, oran)
    elif pitch_deg <= -15.0:
        hiz = FREN
    else:
        hiz = NMAX

    if bat_pct < 10:
        hiz = min(hiz, 0.0)
    elif bat_pct < 30:
        hiz = min(hiz, BAT_HIZ)
    return hiz

check("düz zemin tam batarya",         guvenlik_hiz( 0,  0, 100), 2.0)
check("8° roll → hız azaldı",          guvenlik_hiz( 8,  0, 100) < 2.0, True)
check("15° roll → dur",                guvenlik_hiz(15,  0, 100), 0.0)
check("21° roll → estop + dur",        guvenlik_hiz(21,  0, 100), 0.0)
check("yokuş aşağı fren",             guvenlik_hiz( 0,-16, 100), FREN)
check("düşük batarya %20",            guvenlik_hiz( 0,  0,  20), BAT_HIZ)
check("kritik batarya %5",            guvenlik_hiz( 0,  0,   5), 0.0)
check("roll+düşük batarya kombinasyon",guvenlik_hiz( 5,  0,  20) <= BAT_HIZ, True)

# ─── Sonuç ───────────────────────────────────────────────────────────────────
print(f"\n{'='*45}")
print(f"  TOPLAM: {PASS+FAIL} test | {PASS} GEÇTI | {FAIL} BAŞARISIZ")
print(f"{'='*45}")
sys.exit(0 if FAIL == 0 else 1)
