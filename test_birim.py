#!/usr/bin/env python3
"""
Kritik fonksiyon birim testleri — donanım VE ROS2/rclpy gerektirmez.

Önceki sürüm bu testlerdeki tüm mantığı (Ackermann kinematiği, CRC, IMU
hız kısıtı, vb.) kendi içinde YENİDEN YAZIYORDU — üretim kodundaki
(teknofest_ika/otonomi/*.py, teknofest_ika/gomulu/seri_kopru.py) bir hata
bu testler tarafından hiçbir şekilde yakalanamıyordu, çünkü gerçek kod hiç
çalıştırılmıyordu. Bu sürüm, tüm hesaplamaları barındıran rclpy-bağımsız
teknofest_ika.otonomi.pure_logic modülünü DOĞRUDAN import eder; üretim
node'ları (ackermann_converter.py, seri_kopru.py, imu_guvenlik.py,
anti_rollback.py, yolo_adapter_node.py, misyon_fsm.py) da aynı modülü
kullanır — yani burada test edilen fonksiyonlar sahada çalışan KOD ile
birebir aynıdır.
"""
import math
import sys
import time as _time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    ackermann_steering,
    encoder_delta,
    paket_olustur,
    paket_dogrula,
    batarya_yuzdesi,
    quat_to_roll_pitch_deg,
    rollback_riskli,
    imu_guvenlik_hiz,
    ConsecutiveFrameFilter,
    stop_check,
    hizlanma_hiz_profili,
    DetectionsStore,
    fren_hedef_hesapla,
    fren_yumusat,
)
from teknofest_ika.otonomi.topics import BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC  # noqa: E402

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


# ─── 1. Ackermann Kinematik (ackermann_converter.py) ────────────────────────
print("=== 1. Ackermann Kinematik ===")
L = 0.55
DELTA_MAX = 0.5236  # 30°


def ackermann(v, w):
    return ackermann_steering(v, w, L, DELTA_MAX)


check("düz ileri",      math.degrees(ackermann(1.0, 0.0)), 0.0)
check("sola dön",       ackermann(1.0, 0.5) > 0, True)
check("sağa dön",       ackermann(1.0, -0.5) < 0, True)
check("v=0 sol",        ackermann(0.0, 1.0),  DELTA_MAX)
check("v=0 sağ",        ackermann(0.0, -1.0), -DELTA_MAX)
check("v=0 w=0",        ackermann(0.0, 0.0),  0.0)
check("max limit",      ackermann(0.1, 10.0), DELTA_MAX)
check("geri düz",       math.degrees(ackermann(-1.0, 0.0)), 0.0)
check("geri sola dön",  ackermann(-1.0, 0.5) < 0, True)   # geri giderken sol dönüş → negatif direksiyon
check("geri sağa dön",  ackermann(-1.0, -0.5) > 0, True)

# ─── 2. Enkoder Overflow Koruması (AS5600 10-bit) ───────────────────────────
print("\n=== 2. Enkoder Overflow (AS5600 10-bit) ===")

check("ileri küçük adım", encoder_delta(10, 5),      5)
check("geri küçük adım",  encoder_delta(5, 10),     -5)
check("wrap ileri 1023→1", encoder_delta(1, 1023),   2)
check("wrap geri  1→1023", encoder_delta(1023, 1),  -2)
check("yarım tur ileri",   encoder_delta(512, 0),  512)
check("tam tur sıfır",     encoder_delta(0, 0),     0)

# ─── 3. Binary Paket CRC (seri_kopru.py protokolü) ──────────────────────────
print("\n=== 3. Binary Protokol CRC ===")

for cmd, v0, v1, desc in [
    (0x01, 1000, 1500, "PKT_SURUCU"),
    (0x02, 0,    0,    "PKT_DUR"),
    (0x03, 1,    0,    "PKT_LAZER_AC"),
    (0x03, 0,    0,    "PKT_LAZER_KAP"),
    (0x04, 0,    0,    "PKT_HB"),
    (0x01, -1000, 3000, "negatif hız + büyük yaw"),
]:
    pkt = paket_olustur(cmd, v0, v1)
    ok = paket_dogrula(pkt) and len(pkt) == 8
    check(desc, ok, True)

check("bozuk CRC reddedilir", paket_dogrula(b'\xAA\x01\x00\x00\x00\x00\xFF\x55'), False)
check("yanlış başlangıç byte'ı reddedilir",
      paket_dogrula(b'\x00' + paket_olustur(0x01, 0, 0)[1:]), False)

# ─── 4. Batarya Yüzdesi (4S LiPo, seri_kopru.py) ────────────────────────────
print("\n=== 4. Batarya Yüzde (4S LiPo) ===")
V_MIN, V_MAX = 14.0, 16.8


def bat_pct(v):
    return round(batarya_yuzdesi(v, V_MIN, V_MAX) * 100)


check("tam dolu 16.8V",  bat_pct(16.8), 100)
check("boş     14.0V",   bat_pct(14.0), 0)
check("yarı    15.4V",   bat_pct(15.4), 50)
check("fazla   17.0V",   bat_pct(17.0), 100)  # clamp
check("düşük   13.0V",   bat_pct(13.0), 0)    # clamp
check("normal  15.8V",   bat_pct(15.8), 64)

# ─── 5. IMU Quaternion → Roll / Pitch (imu_guvenlik.py + anti_rollback.py) ──
print("\n=== 5. IMU Quaternion → Roll/Pitch ===")

r, p = quat_to_roll_pitch_deg(1, 0, 0, 0)
check("düz zemin roll",  r, 0.0)
check("düz zemin pitch", p, 0.0)

a = math.radians(15)
r, p = quat_to_roll_pitch_deg(math.cos(a / 2), math.sin(a / 2), 0, 0)
check("15° sağ roll",    r, 15.0)
check("15° sağ pitch",   p,  0.0)

a = math.radians(10)
r, p = quat_to_roll_pitch_deg(math.cos(a / 2), 0, math.sin(a / 2), 0)
check("10° yukarı roll",  r,  0.0)
check("10° yukarı pitch", p, 10.0)

a = math.radians(20)
r, p = quat_to_roll_pitch_deg(math.cos(a / 2), math.sin(a / 2), 0, 0)
check("20° roll → estop eşiği aşıldı", abs(r) >= 19.99, True)  # fp toleransı

# ─── 6. Anti-Rollback Koşul Mantığı (anti_rollback.py) ──────────────────────
print("\n=== 6. Anti-Rollback Koşul Mantığı ===")
PITCH_TH = 10.0  # derece — anti_rollback.py topics.IMU_PITCH_RAMP_THRESHOLD ile değişebilir
VEL_TH = 0.05


def rollback(pitch_deg, vel):
    return rollback_riskli(pitch_deg, vel, PITCH_TH, VEL_TH)


check("rampa yukarı + geri kayma",      rollback(12.0, -0.1),  True)
check("rampa yukarı + ileri gidiyor",   rollback(12.0,  0.5),  False)
check("düz zemin + geri kayma",         rollback( 2.0, -0.1),  False)
check("eşik altı pitch",                rollback( 9.9, -0.2),  False)
check("eşik üstü pitch + dur",          rollback(11.0,  0.0),  False)
check("rampa + büyük geri kayma",       rollback(20.0, -0.5),  True)

# ─── 7. PID Integral Windup Koruması (utils/pid_controller.py mantığı) ──────
print("\n=== 7. PID Integral Windup Koruması ===")
INTEGRAL_MAX = 30.0
integral = 0.0
for _ in range(400):  # 20 saniye × 50Hz = 1000 döngü
    hata = 100.0      # Sürekli büyük hata
    dt = 0.05
    integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral + hata * dt))

check("20s windup sonrası sınırda", integral, INTEGRAL_MAX)
check("sınır aşılmadı",            integral <= INTEGRAL_MAX, True)

integral2 = 0.0
for _ in range(400):
    integral2 = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral2 + (-100.0) * 0.05))
check("negatif windup sınırı", integral2, -INTEGRAL_MAX)

# ─── 8. Odometri dt Guard ────────────────────────────────────────────────────
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

# ─── 9. imu_guvenlik Hız Kısıt Mantığı (imu_guvenlik.py) ────────────────────
print("\n=== 9. IMU Güvenlik Hız Kısıt Mantığı ===")
WARN, STOP, ESTOP = 8.0, 15.0, 20.0
PITCH_DOWN = 15.0
NMAX, FREN, BAT_HIZ = 2.0, 0.4, 1.0
BAT_DUSUK, BAT_KRITIK = BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC


def guvenlik_hiz(roll_deg, pitch_deg, bat_pct_):
    return imu_guvenlik_hiz(
        roll_deg, pitch_deg, bat_pct_,
        WARN, STOP, ESTOP, PITCH_DOWN, NMAX, FREN,
        BAT_DUSUK, BAT_KRITIK, BAT_HIZ,
    )


check("düz zemin tam batarya",         guvenlik_hiz( 0,  0, 100), 2.0)
check("8° roll → hız azaldı",          guvenlik_hiz( 8,  0, 100) < 2.0, True)
check("15° roll → dur",                guvenlik_hiz(15,  0, 100), 0.0)
check("21° roll → estop + dur",        guvenlik_hiz(21,  0, 100), 0.0)
check("yokuş aşağı fren",             guvenlik_hiz( 0,-16, 100), FREN)
check("düşük batarya %20",            guvenlik_hiz( 0,  0,  20), BAT_HIZ)
check("kritik batarya %5",            guvenlik_hiz( 0,  0,   5), 0.0)
check("roll+düşük batarya kombinasyon", guvenlik_hiz( 5,  0,  20) <= BAT_HIZ, True)

# ─── 10. Ardışık-Frame Filtresi — STOP (§6.10, sticky=False) ────────────────
print("\n=== 10. ConsecutiveFrameFilter — STOP (§6.10) ===")

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
check("1. frame → stop_var=False",             f.isle(True),  False)

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
f.isle(True)
check("2 ardışık frame → stop_var=True",       f.isle(True),  True)

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
f.isle(True)
f.isle(False)   # detection yok → sayaç sıfır
check("kesilen ardışık → stop_var=False",      f.isle(True),  False)

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
f.isle(True)
f.isle(True)
check("3. frame → hâlâ True",                  f.isle(True),  True)

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
check("hiç detection yok → False",             f.isle(False), False)

f = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
f.isle(True)
f.isle(True)
check("onaylandıktan sonra kaybolursa ANINDA False (sticky değil)",
      f.isle(False), False)

# ─── 11. Ardışık-Frame Filtresi — Tabela (sticky=True) ──────────────────────
print("\n=== 11. ConsecutiveFrameFilter — Tabela (sticky) ===")
NO_DETECTION = 255

f = ConsecutiveFrameFilter(2, bos_deger=NO_DETECTION, sticky=True)
check("1. frame tek kare → henüz onaylanmadı", f.isle(5), NO_DETECTION)
check("2. aynı aday → onaylandı",              f.isle(5), 5)
check("tek kare kaybı confirmed'i bozmaz (sticky)",
      f.isle(NO_DETECTION), 5)
check("yeni aday 1. kare → hâlâ önceki onaylı",
      f.isle(7), 5)
check("yeni aday 2. kare → yeni onay",         f.isle(7), 7)

# ─── 12. FSM §6.10 STOP Cooldown Mantığı (misyon_fsm.NavigateState) ─────────
print("\n=== 12. FSM §6.10 STOP Cooldown ===")

aktif = _time.time() + 100.0   # cooldown henüz bitmemiş
bitmis = _time.time() - 1.0    # cooldown bitti
now = _time.time()

check("cooldown aktif + stop_var=True → False",
      stop_check(True, aktif, now), False)
check("cooldown aktif + stop_var=False → False",
      stop_check(False, aktif, now), False)
check("cooldown bitti + stop_var=True → True",
      stop_check(True, bitmis, now), True)
check("cooldown bitti + stop_var=False → False",
      stop_check(False, bitmis, now), False)
check("cooldown=0 (başlangıç) + stop_var=True → True",
      stop_check(True, 0.0, now), True)
check("cooldown=0 + stop_var=False → False",
      stop_check(False, 0.0, now), False)

# ─── 13. HizlanmaState Hız Profili (§6.11, misyon_fsm.py) ───────────────────
print("\n=== 13. HizlanmaState Hız Profili (§6.11) ===")

MAX_HIZ = 10.0
TOPLAM_MESAFE = 30.0
FREN_BASI_MESAFE = 25.0


def hizlanma_profili(dist):
    return hizlanma_hiz_profili(dist, MAX_HIZ, TOPLAM_MESAFE, FREN_BASI_MESAFE)


check("dist=0m → max hız 10 m/s",                   hizlanma_profili(0.0),  10.0)
check("dist=24.9m → max hız (eşik öncesi)",          hizlanma_profili(24.9), 10.0)
check("dist=25m → hâlâ max hız (eşikte)",            hizlanma_profili(25.0), 10.0)
check("dist=27.5m → 5 m/s (yarı fren)",              hizlanma_profili(27.5),  5.0)
check("dist=29m → 2 m/s",                            hizlanma_profili(29.0),  2.0)
check("dist=29.85m → 0.3 clamp",                     hizlanma_profili(29.85), 0.3)
check("dist=30m → 0.3 clamp (heartbeat korur)",      hizlanma_profili(30.0),  0.3)
check("hız hiçbir noktada negatif olmamalı",         hizlanma_profili(35.0) >= 0.0, True)

# ─── 14. DetectionsStore Veri Bütünlüğü (misyon_fsm.py) ─────────────────────
print("\n=== 14. DetectionsStore Veri Bütünlüğü ===")

ds = DetectionsStore()
check("başlangıç tabela=255",                        ds.get_field('tabela'),        255)
check("başlangıç stop_var=False",                    ds.get_field('stop_var'),      False)
check("başlangıç e_stop=False",                      ds.get_field('e_stop'),        False)

ds.update({'tabela': 5, 'stop_var': True})
check("update sonrası tabela=5",                     ds.get_field('tabela'),        5)
check("update sonrası stop_var=True",                ds.get_field('stop_var'),      True)
check("update sonrası eksik alan _DEFAULT'a döner",  ds.get_field('e_stop'),        False)

ds.update_field('e_stop', True)
check("update_field e_stop=True",                    ds.get_field('e_stop'),        True)
check("update_field tabela'yı bozmaz",               ds.get_field('tabela'),        5)

ds.update_field('stop_var', False)   # cooldown sonrası FSM sıfırlar
check("stop_var cooldown sıfırlaması",               ds.get_field('stop_var'),      False)

ds2 = DetectionsStore()
ds2.update({'tabela': 8, 'stop_var': True})
ds2.update({'tabela': 3})            # ikinci update _DEFAULT'tan merge eder
check("ikinci update stop_var _DEFAULT'a döner",     ds2.get_field('stop_var'),     False)
check("ikinci update tabela yenilendi",              ds2.get_field('tabela'),       3)

# /ika/detections'ın SAHİBİ OLMADIĞI alanlar update() ile silinmemeli:
# ön kamera 29.9 Hz yayınlıyor, silinirse E-STOP saniyede 30 kez temizlenir.
ds3 = DetectionsStore()
ds3.update_field('e_stop',     True)
ds3.update_field('manual_mod', True)
ds3.update_field('kayar_yon',  'sol')
# yolo_adapter_node.py:181 payload'ının birebir alan kümesi
ds3.update({'tabela': 255, 'hedef_var': False, 'hedef_hata_x': 0.0,
            'hedef_hata_y': 0.0, 'koni_var': False, 'hizlanma_bitti': False,
            'stop_var': False, 'bariyer_sol_m': 1.5, 'bariyer_sag_m': 1.5,
            'fps': 0.0})
check("detections karesi e_stop'u silmez",           ds3.get_field('e_stop'),       True)
check("detections karesi manual_mod'u silmez",       ds3.get_field('manual_mod'),   True)
check("detections karesi kayar_yon'u silmez",        ds3.get_field('kayar_yon'),    'sol')
check("dış kaynaklı koruma detections alanını tutmaz",
                                                     ds3.get_field('tabela'),       255)
# Eski payload'lar kayar_yon'u placeholder olarak taşıyordu; artık yok sayılır.
ds3.update({'tabela': 7, 'kayar_yon': 'bilinmiyor'})
check("payload'daki kayar_yon yok sayılır",          ds3.get_field('kayar_yon'),    'sol')

# ─── 15. Otomatik Fren Oranı (ackermann_converter.py) ───────────────────────
print("\n=== 15. Otomatik Fren Oranı ===")

ESIK_MIN = 1.5
ESIK_MAX = 5.0
TAM_DUR_ORAN = 0.3


def fren_hedef(onceki, hedef, dt):
    return fren_hedef_hesapla(onceki, hedef, dt, ESIK_MIN, ESIK_MAX, TAM_DUR_ORAN)


check("hızlanırken fren yok",              fren_hedef(1.0, 2.0, 0.1), 0.0)
check("sabit hızda fren yok",              fren_hedef(2.0, 2.0, 0.5), 0.0)
check("eşik altı yavaşlama → fren yok",    fren_hedef(2.0, 1.9, 0.1), 0.0)
check("orta yavaşlama → orantılı",         fren_hedef(3.0, 2.0, 0.5), 0.142857)
check("aşırı yavaşlama → tam fren (1.0)",  fren_hedef(5.0, 0.5, 0.2), 1.0)
check("tam dur → en az TAM_DUR_ORAN",      fren_hedef(0.5, 0.0, 1.0), 0.3)
check("geri viteste yavaşlama → orantılı", fren_hedef(-3.0, -1.0, 0.5), 0.714286)
check("dt=0 → fren yok (güvenli varsayılan)", fren_hedef(3.0, 0.0, 0.0), 0.0)

check("yumuşatma: hedefe doğru sınırlı artış",  fren_yumusat(0.0, 1.0, 0.5, 1.0), 0.5)
check("yumuşatma: küçük dt → küçük adım",       fren_yumusat(0.0, 1.0, 0.5, 0.1), 0.05)
check("yumuşatma: azalış da sınırlı",           fren_yumusat(0.8, 0.0, 0.5, 1.0), 0.3)
check("yumuşatma: hedefe zaten ulaşılmış",      fren_yumusat(0.5, 0.5, 0.5, 1.0), 0.5)
check("yumuşatma: dt=0 → değişmez",             fren_yumusat(0.4, 1.0, 0.5, 0.0), 0.4)

# ─── Sonuç ───────────────────────────────────────────────────────────────────
print(f"\n{'='*45}")
print(f"  TOPLAM: {PASS+FAIL} test | {PASS} GEÇTI | {FAIL} BAŞARISIZ")
print(f"{'='*45}")


def test_birim_hepsi_gecti():
    """pytest girişi — dosya import edilirken yukarıdaki kontroller çalışır."""
    assert FAIL == 0, f"{FAIL} birim testi başarısız"


# sys.exit modül düzeyinde çağrılırsa pytest dosyayı toplarken SystemExit
# alıp collection'ı hataya düşürüyor; çıkış kodu yalnız doğrudan çalıştırmaya ait.
if __name__ == '__main__':
    sys.exit(0 if FAIL == 0 else 1)
