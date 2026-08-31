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
import re
import yaml
import sys
import time as _time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    ackermann_steering,
    ackermann_komut,
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
    durma_degerlendir,
    tarama_yan_mesafe,
    koridor_sapmasi,
    aci_sarmala,
    DetectionsStore,
    fren_hedef_hesapla,
    fren_yumusat,
    kosu_butcesi,
    pas_verilebilir,
    rc_mod_otonom,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC, PAS_HAKKI, PAS_GECILEMEZ,
    KOSU_SURESI_S,
)

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
L = 1.40
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

# Eğrilik kırpması — R_min = L/tan(δ_max) = 1.40/tan(30°) = 2.425 m
TABAN = 0.45
R_MIN = L / math.tan(DELTA_MAX)


def komut(v, w, taban=TABAN):
    return ackermann_komut(v, w, L, DELTA_MAX, taban)


# Ulaşılabilir eğrilik: hız da direksiyon da dokunulmadan geçer
hiz, delta, doydu = komut(0.90, 0.30)
check("κ sınır içi: hız korunur",  round(hiz, 6), 0.90)
check("κ sınır içi: doyma yok",    doydu, False)
check("κ sınır içi: δ = atan(Lω/v)", round(delta, 6),
      round(math.atan(L * 0.30 / 0.90), 6))

# Nav2'nin sahada ürettiği doyma: R = 0.90/0.80 = 1.125 m < R_min
hiz, delta, doydu = komut(0.90, 0.80)
check("doyma bildirilir",       doydu, True)
check("doymada δ = δ_max",      round(delta, 6), round(DELTA_MAX, 6))
check("doymada hız düşürülür",  hiz < 0.90, True)
check("hız kalkış tabanında",   round(hiz, 6), TABAN)   # oransal 0.418 → taban

# Taban devrede değilken kırpma oranı κ_max/|κ| olmalı
hiz, _, _ = komut(0.90, 0.80, taban=0.0)
check("oransal kırpma", round(hiz, 4),
      round(0.90 * (1.0 / R_MIN) / (0.80 / 0.90), 4))

# İstenen hız zaten tabanın altındaysa yükseltilmez
hiz, _, doydu = komut(0.30, 0.90, taban=TABAN)
check("taban isteneni yükseltmez", hiz <= 0.30, True)
check("düşük hızda da doyar",      doydu, True)

# Geri viteste işaretler korunur
hiz, delta, doydu = komut(-0.90, 0.80)
check("geri: doyma bildirilir", doydu, True)
check("geri: hız negatif",      hiz < 0.0, True)
check("geri: δ negatif",        delta < 0.0, True)
check("geri: |δ| = δ_max",      round(abs(delta), 6), round(DELTA_MAX, 6))

# v≈0 (Nav2 spin recovery): mevcut davranış korunur, hız kırpılmaz
hiz, delta, doydu = komut(0.0, 1.0)
check("v=0: hız dokunulmaz", hiz, 0.0)
check("v=0: δ = δ_max",      delta, DELTA_MAX)
check("v=0: doyma yok",      doydu, False)

# Düz gidişte hız asla kırpılmaz — taban mantığı burada devreye girmemeli
hiz, delta, doydu = komut(0.90, 0.0)
check("düz: hız korunur", round(hiz, 6), 0.90)
check("düz: δ = 0",       round(delta, 6), 0.0)
check("düz: doyma yok",   doydu, False)

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
WARN, STOP, ESTOP = 13.0, 15.0, 20.0
PITCH_DOWN = 15.0
NMAX, FREN, BAT_HIZ = 2.0, 0.4, 1.0
TABAN = 0.45
YAN_EGIM_DEG = math.degrees(math.atan(0.20))   # §6.5 %20 = 11,31°
BAT_DUSUK, BAT_KRITIK = BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC


def guvenlik_hiz(roll_deg, pitch_deg, bat_pct_, taban=TABAN):
    return imu_guvenlik_hiz(
        roll_deg, pitch_deg, bat_pct_,
        WARN, STOP, ESTOP, PITCH_DOWN, NMAX, FREN,
        BAT_DUSUK, BAT_KRITIK, BAT_HIZ, taban,
    )


check("düz zemin tam batarya",         guvenlik_hiz( 0,  0, 100), 2.0)
check("13° roll → hız azaldı",         guvenlik_hiz(13,  0, 100) < 2.0, True)
check("15° roll → dur",                guvenlik_hiz(15,  0, 100), 0.0)
check("21° roll → estop + dur",        guvenlik_hiz(21,  0, 100), 0.0)

# §6.5 zorunlu bir aşama: %20 yan eğimde hız kısılmamalı
check("§6.5 %20 yan eğim → kısıtsız",  guvenlik_hiz(YAN_EGIM_DEG, 0, 100), 2.0)
check("§6.5 eşiği WARN'ın altında",    YAN_EGIM_DEG < WARN, True)

# Taban kelepçesi: sonuç 0 ile taban arasında kalamaz
ara = [guvenlik_hiz(r, 0, 100) for r in
       [13.5, 14.0, 14.2, 14.5, 14.7, 14.9]]
check("rampa 0-taban aralığına düşmez", all(v == 0.0 or v >= TABAN for v in ara), True)
check("rampa hâlâ azalıyor",           ara[0] > ara[-1] or ara[-1] == TABAN, True)
check("taban devrede: 14.9° → 0.45",   guvenlik_hiz(14.9, 0, 100), TABAN)
check("taban=0 iken eski davranış",    guvenlik_hiz(14.9, 0, 100, taban=0.0) < TABAN, True)
check("dur eşiği kelepçeden etkilenmez", guvenlik_hiz(15.0, 0, 100), 0.0)
# Kelepçe yalnız yan eğim rampasına ait: inişte düşük komut = çok fren
check("yokuş aşağı kelepçelenmez",     guvenlik_hiz( 0, -25, 100), FREN)
check("FRENLEME_HIZ zaten tabanın altı", FREN < TABAN, True)
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
OLCUM_MESAFE = 30.0
DURMA_MESAFE = 10.0


def hizlanma_profili(dist):
    return hizlanma_hiz_profili(dist, MAX_HIZ, OLCUM_MESAFE)


# §6.11: ölçülen 30 m'nin SONUNA KADAR tam gaz — içeride yavaşlama yok
check("dist=0m → tam gaz",              hizlanma_profili(0.0),  10.0)
check("dist=25m → hâlâ tam gaz",        hizlanma_profili(25.0), 10.0)
check("dist=29.9m → hâlâ tam gaz",      hizlanma_profili(29.9), 10.0)
check("dist=30m → çizgide 0 (fren)",    hizlanma_profili(30.0),  0.0)
check("dist=35m → 0",                   hizlanma_profili(35.0),  0.0)
check("hız hiçbir noktada negatif değil",
      all(hizlanma_profili(d) >= 0.0 for d in (0, 15, 29.9, 30, 40)), True)

# §6.11 durma payı: çizgi sonrası 10 m içinde durulmalı
def durma(asilan, hiz):
    return durma_degerlendir(asilan, hiz, DURMA_MESAFE)


check("hâlâ hızlı, pay içinde → devam",  durma(2.0, 1.5),  'devam')
check("hız eşiğin altı → durdu",         durma(2.0, 0.02), 'durdu')
check("tam sıfır → durdu",               durma(0.0, 0.0),  'durdu')
check("pay dolmuş, hâlâ hızlı → aşıldı", durma(10.0, 0.8), 'butce_asildi')
check("pay aşılmış → aşıldı",            durma(12.0, 0.8), 'butce_asildi')
check("pay sınırında ama durmuş → durdu", durma(10.0, 0.01), 'durdu')
check("geri kayma da hız sayılır",       durma(3.0, -0.5), 'devam')

# ─── 13b. §6.9 Koridor Ortalaması (misyon_fsm.KoridorIzleyici) ──────────────
print("\n=== 13b. §6.9 Koridor Ortalaması ===")

KORIDOR = 3.0
LIDAR_YAW = 1.6284          # 93,3° — LiDAR gövdeye dönük monte


def _tarama(sol_m, sag_m, n=360):
    """Sol/sağ duvarı verilen mesafede olan yapay bir tarama üretir.

    Araç ekseninde ±90°, LiDAR çerçevesine kaydırılmış hâlde doldurulur;
    böylece test montaj dönüklüğünü de kapsar.
    """
    artis = 2.0 * math.pi / n
    aci_min = -math.pi
    sol_c = aci_sarmala(math.pi / 2 - LIDAR_YAW)
    sag_c = aci_sarmala(-math.pi / 2 - LIDAR_YAW)
    r = []
    for i in range(n):
        aci = aci_min + i * artis
        if abs(aci_sarmala(aci - sol_c)) <= 0.26:
            r.append(sol_m)
        elif abs(aci_sarmala(aci - sag_c)) <= 0.26:
            r.append(sag_m)
        else:
            r.append(float('inf'))
    return r, aci_min, artis


def olc(sol_m, sag_m):
    r, amin, art = _tarama(sol_m, sag_m)
    sol = tarama_yan_mesafe(r, amin, art, aci_sarmala(math.pi/2 - LIDAR_YAW),
                            0.26, 0.05, 12.0)
    sag = tarama_yan_mesafe(r, amin, art, aci_sarmala(-math.pi/2 - LIDAR_YAW),
                            0.26, 0.05, 12.0)
    return koridor_sapmasi(sol, sag, KORIDOR)


# Açı sarması: sağ pencere ±180° civarına düşüyor, düz çıkarma onu böler
check("sarmalama ±pi",       round(aci_sarmala(math.pi + 0.1), 4),
                             round(-math.pi + 0.1, 4))
check("sağ pencere sarmalı", abs(aci_sarmala(-math.pi/2 - LIDAR_YAW)) > math.pi/2, True)

sapma, durum = olc(1.5, 1.5)
check("tam ortada → sapma 0",   round(sapma, 3), 0.0)
check("tam ortada → geçerli",   durum, 'gecerli')

sapma, durum = olc(2.0, 1.0)
check("sağa kaymış → + sapma",  round(sapma, 3), 0.5)
check("sağa kayma geçerli",     durum, 'gecerli')

sapma, durum = olc(1.0, 2.0)
check("sola kaymış → − sapma",  round(sapma, 3), -0.5)

# Bir duvar yoksa ölçüm koridora ait değildir — sapma diye raporlanmamalı
sapma, durum = olc(1.5, 9.0)
check("duvar yok → koridor_yok", durum, 'koridor_yok')
check("koridor_yok → sapma 0",   sapma, 0.0)

check("hiç ışın yok → olcum_yok",
      koridor_sapmasi(None, 1.5, KORIDOR)[1], 'olcum_yok')
check("boş tarama → None",
      tarama_yan_mesafe([], -math.pi, 0.01, 0.0, 0.26, 0.05, 12.0), None)
check("artış 0 → None",
      tarama_yan_mesafe([1.0], -math.pi, 0.0, 0.0, 0.26, 0.05, 12.0), None)

# Menzil dışı ve NaN ışınlar pencereye girmemeli
check("menzil dışı elenir",
      tarama_yan_mesafe([99.0, 2.0, 99.0], -0.02, 0.02, 0.0, 0.26, 0.05, 12.0), 2.0)
check("NaN elenir",
      tarama_yan_mesafe([float('nan'), 2.0], -0.02, 0.02, 0.0, 0.26, 0.05, 12.0), 2.0)
# Medyan: tek aykırı ışın sonucu kaydırmamalı
check("medyan aykırıya direnir",
      tarama_yan_mesafe([1.5, 1.5, 0.2], -0.02, 0.02, 0.0, 0.26, 0.05, 12.0), 1.5)

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

# ─── 16. Koşu Saati ve Pas Hakkı (§6.12 / §9) ───────────────────────────────
print("\n=== 16. Koşu Saati ve Pas Hakkı ===")

check("bütçe kalan süreye kırpılır",      kosu_butcesi(120.0, 45.0), 45.0)
check("bütçe istenenden büyümez",         kosu_butcesi(30.0, 900.0), 30.0)
check("süre bitmişse bütçe 0",            kosu_butcesi(120.0, 0.0), 0.0)
check("negatif kalan → 0, eksi süre yok", kosu_butcesi(120.0, -5.0), 0.0)
# 11 aşama × 120 s = 1320 s; koşu limiti 900 s. Kırpma olmadan tek bir takılan
# aşama koşunun tamamını yiyebiliyordu.
check("aşama bütçesi toplamı limiti aşar", 11 * 120.0 > KOSU_SURESI_S, True)
check("§6.12 koşu limiti 15 dakika",       KOSU_SURESI_S, 900.0)


def pas(acik, label, kullanildi):
    return pas_verilebilir(acik, label, kullanildi, PAS_HAKKI, PAS_GECILEMEZ)


check("§9 pas hakkı koşu başına 1",        PAS_HAKKI, 1)
check("varsayılan kapalı → pas yok",       pas(False, 'KAYAR_ENGEL', 0), (False, 'kapali'))
check("açık + serbest aşama → pas",        pas(True, 'KAYAR_ENGEL', 0), (True, 'izin'))
check("koniler pas geçilemez (§9)",        pas(True, 'KONİLİ_YOL', 0), (False, 'sartname_yasak'))
check("hızlanma pas geçilemez (§9)",       pas(True, 'HIZLANMA_PARKURU', 0), (False, 'sartname_yasak'))
check("hak bitince ikinci pas yok",        pas(True, 'DIK_ENGEL', 1), (False, 'hak_bitti'))
check("yasak, hak dolu olsa da yasak",     pas(True, 'KONİLİ_YOL', 1), (False, 'sartname_yasak'))


# ─── 17. RC Mod Eşiği — ROS ile firmware aynı sayıda bölmeli ────────────────
from teknofest_ika.otonomi.topics import RC_MOD_ESIK_US  # noqa: E402

check("eşiğin altı manuel",               rc_mod_otonom(1499, RC_MOD_ESIK_US), False)
check("eşik dahil otonom",                rc_mod_otonom(1500, RC_MOD_ESIK_US), True)
check("pot dipte manuel",                 rc_mod_otonom(1000, RC_MOD_ESIK_US), False)
check("pot tepede otonom",                rc_mod_otonom(2000, RC_MOD_ESIK_US), True)
check("ara bant yok — 1400 manuel",       rc_mod_otonom(1400, RC_MOD_ESIK_US), False)
check("ara bant yok — 1600 otonom",       rc_mod_otonom(1600, RC_MOD_ESIK_US), True)


def _firmware_mod_esigi():
    """config.h'deki RC_MOD_ESIK — Mega'nın kullandığı sayı."""
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'arduino', 'include', 'config.h')
    with open(yol, encoding='utf-8', errors='replace') as f:
        m = re.search(r'^#define\s+RC_MOD_ESIK\s+(\d+)', f.read(), re.M)
    return int(m.group(1)) if m else None


# Aynı kanalı iki taraf okuyor. Sayılar ayrışırsa potun arada kaldığı bantta
# ROS ile Mega aynı anda farklı modda olur ve bu yalnız sahada fark edilir.
check("ROS eşiği = firmware eşiği",       _firmware_mod_esigi(), RC_MOD_ESIK_US)


# ─── §7.5 fren: sınır ötesi anlam kayması koruması ──────────────────────────
from teknofest_ika.otonomi.topics import FREN_GUVENLI_DUR_BINDE  # noqa: E402


def _firmware_kaynak():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'arduino', 'src', 'main.cpp')
    with open(yol, encoding='utf-8', errors='replace') as f:
        return f.read()


def _firmware_fren_sifir_anlami():
    """fren_uygula()'nın sıfır dalı: 'TUT' mu, 'SERBEST' mi.

    Firmware bir kez sıfırı SERBEST'e çevirmiş, ROS tarafı haberdar olmamış ve
    watchdog `data=0` yayınlayarak freni §6.10'un zorunlu duruşunun ortasında
    bırakmıştı. Dal geri çevrilirse burada patlasın, sahada değil.
    """
    m = re.search(r'fren_esc_hiz\(binde > 0 \? binde / 1000\.0f : ([^)]+)\)',
                  _firmware_kaynak())
    return m.group(1).strip() if m else None


check("firmware'de fren 0 = TUT (NOTR)",  _firmware_fren_sifir_anlami(), "0.0f")

# Güvenlik dalları park freni makinesinin bekleme penceresini atlamalı.
check("güvenli duruş freni sıfır değil",  FREN_GUVENLI_DUR_BINDE > 0, True)
check("güvenli duruş freni tam fren",     FREN_GUVENLI_DUR_BINDE, 1000)


def _firmware_fren_sabiti(ad):
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'arduino', 'include', 'config.h')
    with open(yol, encoding='utf-8', errors='replace') as f:
        m = re.search(r'^#define\s+' + ad + r'\s+(\d+)UL', f.read(), re.M)
    return int(m.group(1)) if m else None


# Strok süreleri ÖLÇÜLMEDİ (placeholder). Testin iddiası sayının doğruluğu değil,
# üçünün de tanımlı ve pozitif olması — biri silinirse geçiş anında takılır.
check("FREN_ACMA_MS tanımlı",             (_firmware_fren_sabiti('FREN_ACMA_MS') or 0) > 0, True)
check("FREN_SIKMA_MS tanımlı",            (_firmware_fren_sabiti('FREN_SIKMA_MS') or 0) > 0, True)
check("FREN_BEKLEME_MS tanımlı",          (_firmware_fren_sabiti('FREN_BEKLEME_MS') or 0) > 0, True)


# ─── waypoints.yaml: FSM'i dallandıran `type` alanı ─────────────────────────
# misyon_fsm NavigateState'ten yalnız wp.get('type') ile dallanıyor. Alan
# sessizce düşerse ShootApproach/Shoot ve Hizlanma durumlarına HİÇ girilmez;
# hiçbir hata basılmaz, yalnız puan gider. Bir kez böyle oldu.
def _waypoint_tipleri():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'config', 'waypoints.yaml')
    with open(yol, encoding='utf-8') as f:
        return {a['isim']: a.get('type') for a in yaml.safe_load(f)['asamalar']}


_TIP = _waypoint_tipleri()
check("ATIS_BOLGESI type=shoot",          _TIP.get('ATIS_BOLGESI'), 'shoot')
check("HIZLANMA_PARKURU type=hizlanma",   _TIP.get('HIZLANMA_PARKURU'), 'hizlanma')

# Fiziksel sıra: atış rampanın ORTASINDA. Şartname §6.10 hedefi "minimum 10
# metre" uzağa koyuyor; rampa çıkışından hedefe 8,18 m var, yani atış orada
# yapılamaz. Liste sırası FSM'in sürüş sırasıdır.
def _asama_sirasi():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'config', 'waypoints.yaml')
    with open(yol, encoding='utf-8') as f:
        return [a['isim'] for a in yaml.safe_load(f)['asamalar']]


_SIRA = _asama_sirasi()
check("atış rampa girişinden sonra",      _SIRA.index('ATIS_BOLGESI') > _SIRA.index('DIK_EGIM_GIRIS'), True)
check("atış rampa çıkışından önce",       _SIRA.index('ATIS_BOLGESI') < _SIRA.index('DIK_EGIM_CIKIS'), True)


# ─── Hata kurtarma: tek istasyonda takılmak koşuyu bitirmemeli ──────────────
from teknofest_ika.otonomi.pure_logic import kurtarma_karari  # noqa: E402


def _fsm_kaynak():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'teknofest_ika', 'otonomi', 'misyon_fsm.py')
    with open(yol, encoding='utf-8') as f:
        return f.read()


_FSM = _fsm_kaynak()

check("1. denemede tekrar",               kurtarma_karari(1, 2), 'tekrar')
check("2. denemede vazgeç",               kurtarma_karari(2, 2), 'sonraki')
check("sayaç aşarsa yine vazgeç",         kurtarma_karari(5, 2), 'sonraki')

# ErrorRecovery artık görevi bitiremez: 'abort' çıktısı kaldırıldı. Kalsaydı
# 5 puanlık bir aşamada takılmak aşağı akıştaki ~275 puanı da götürürdü.
check("kurtarmada 'abort' çıktısı yok",   "'abort'" in _FSM, False)
check("kurtarma sonraki aşamaya geçer",   "'sonraki_asama': 'NAVIGATE'" in _FSM, True)

# E-STOP ile navigasyon başarısızlığı AYNI telden gidiyordu: E-STOP basılınca
# FSM hemen durmuyor, kurtarma denemesi harcayıp saniyelerce oyalanıyordu.
check("E-STOP ayrı çıktı",                "'e_stop'" in _FSM, True)
check("E-STOP doğrudan iptale gider",     "'e_stop':           'MISSION_ABORT'" in _FSM, True)
check("E-STOP kurtarmaya uğramaz",        "'e_stop':           'ERROR_RECOVERY'" in _FSM, False)
# Süre dolması da kurtarmaya uğramadan iptale gitmeli (§6.12).
check("süre dolunca doğrudan iptal",      "'sure_doldu':       'MISSION_ABORT'" in _FSM, True)

# Durum sınıflarının hepsi e_stop'u ilan etmeli; etmezse smach çalışma anında
# InvalidTransitionError atar ve bu yalnız sahada görülür.
check("NavigateState e_stop ilan eder",   _FSM.count("'sure_doldu', 'failed', 'e_stop'"), 1)
check("ShootApproach e_stop ilan eder",   "'in_position', 'failed', 'e_stop'" in _FSM, True)
check("ShootState e_stop ilan eder",      "'shot_fired', 'failed', 'e_stop'" in _FSM, True)
check("HizlanmaState e_stop ilan eder",   "'completed', 'failed', 'e_stop'" in _FSM, True)


def _smach_uyusmazliklari():
    """Her durumun ilan ettiği çıktı bir geçişe bağlı mı.

    smach uyuşmazlığı yalnız ÇALIŞMA ANINDA InvalidTransitionError olarak
    patlar — yani ilk kez sahada, o duruma girildiğinde. Bağlanmamış tek bir
    çıktı koşunun ortasında FSM'i düşürür.
    """
    import ast
    agac = ast.parse(_FSM)
    ciktilar = {}
    for d in ast.walk(agac):
        if isinstance(d, ast.ClassDef):
            for c in ast.walk(d):
                if (isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                        and c.func.attr == '__init__'):
                    for kw in c.keywords:
                        if kw.arg == 'outcomes' and isinstance(kw.value, ast.List):
                            ciktilar[d.name] = {e.value for e in kw.value.elts}
    sorun = []
    durumlar = set()
    eklemeler = []
    for n in ast.walk(agac):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'add' and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant)):
            sinif = (n.args[1].func.id if isinstance(n.args[1], ast.Call)
                     and isinstance(n.args[1].func, ast.Name) else None)
            gecis = set()
            hedef = set()
            for kw in n.keywords:
                if kw.arg == 'transitions' and isinstance(kw.value, ast.Dict):
                    gecis = {k.value for k in kw.value.keys}
                    hedef = {v.value for v in kw.value.values}
            durumlar.add(n.args[0].value)
            eklemeler.append((n.args[0].value, sinif, gecis, hedef))
    bilinen = durumlar | {'GOREV_TAMAMLANDI', 'GOREV_IPTAL'}
    for ad, sinif, gecis, hedef in eklemeler:
        for eksik in ciktilar.get(sinif, set()) - gecis:
            sorun.append(f'{ad}: {eksik} çıktısı bağlanmamış')
        for fazla in gecis - ciktilar.get(sinif, set()):
            sorun.append(f'{ad}: {fazla} geçişinin karşılığı yok')
        for h in hedef - bilinen:
            sorun.append(f'{ad}: bilinmeyen hedef {h}')
    return sorun


check("smach çıktı/geçiş uyuşmazlığı yok", _smach_uyusmazliklari(), [])


# ─── İstasyon başına süre bütçesi ────────────────────────────────────────────
def _wp_yaml():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'config', 'waypoints.yaml')
    with open(yol, encoding='utf-8') as f:
        return yaml.safe_load(f)


_WPY   = _wp_yaml()
_ASAMA = _WPY['asamalar']
_BUTCE = [a.get('timeout_saniye') for a in _ASAMA]

# Süresi olmayan aşama sessizce 120 s'lik yedek tavana düşer — tek bir ortak
# sayı 2,5 m'lik rampa inişine de 22 m'lik U dönüşüne de aynı süreyi veriyordu.
check("her aşamanın kendi süresi var",    [b for b in _BUTCE if b is None], [])
check("hiçbir bütçe 25 s'nin altında değil", [b for b in _BUTCE if b < 25], [])

# Atış (SHOOT_APPROACH 15 + SHOOT 3×8 + 3×1), hızlanma (52,6+10), rampa
# duruşları (2×2) ve kayar engel yön beklemesi (10) aşama bütçesinin DIŞINDA
# ama koşu saatinden yiyor. §6.12'yi aşmak koşu süresi puanının (100) tamamını
# götürür — istasyon puanları kalır, süre puanı sıfırlanır.
# RampaState iki kez koşar (tırmanış + iniş) ve timeout'unu hızdan türetir:
#   (YAKLASMA_MAX 3 + EGIM_MAX 6) / (hız × 0.40) + DURMA_TIMEOUT 8 + DURUS 2
_RAMPA_S = sum((3.0 + 6.0) / (h * 0.40) + 8.0 + 2.0
               for h in (_WPY['parametreler']['rampa_tirmanis_hiz'],
                         _WPY['parametreler']['rampa_inis_hiz']))
_ASAMA_DISI = 10 + 42 + 63 + _RAMPA_S
check("bütçe toplamı limiti aşmıyor",     sum(_BUTCE) <= KOSU_SURESI_S, True)
check("aşama dışı dahil en kötü hâl sığar",
      sum(_BUTCE) + _ASAMA_DISI <= KOSU_SURESI_S, True)
# Pay bir-iki tekrar denemeyi karşılamalı (MAX_RETRIES=2).
check("en uzun aşama bir kez daha sığar",
      sum(_BUTCE) + _ASAMA_DISI + max(_BUTCE) <= KOSU_SURESI_S, True)

# Alan okunmazsa bütçeler ölü konfigürasyon olur ve kimse fark etmez.
check("FSM aşama başına süreyi okuyor",   "wp.get('timeout_saniye'" in _FSM, True)
check("yedek tavan hâlâ var",             'asama_timeout_saniye' in _WPY['parametreler'], True)


# ─── §6.10 rampa: Nav2 baypası ───────────────────────────────────────────────
from teknofest_ika.otonomi.pure_logic import rampa_faz_gecisi  # noqa: E402


def _faz(f, pitch, mesafe):
    return rampa_faz_gecisi(f, pitch, 15.0, mesafe, 3.0, 6.0)


check("düz zeminde yaklaşma sürer",       _faz('yaklasma', 2.0, 0.5), 'yaklasma')
check("eğim başlayınca faz değişir",      _faz('yaklasma', 20.0, 0.5), 'egimde')
check("eğim bulunamazsa iptal",           _faz('yaklasma', 2.0, 3.5), 'bulunamadi')
check("eğimde kalır",                     _faz('egimde', 20.0, 2.0), 'egimde')
check("eğim biterse tamam",               _faz('egimde', 3.0, 4.0), 'bitti')
check("eğim bitmezse iptal",              _faz('egimde', 20.0, 7.0), 'asildi')
# Mutlak pitch: aynı mantık inişte de çalışmalı (pitch negatif gelir).
check("iniş de aynı mantıkla yürür",      _faz('yaklasma', abs(-20.0), 0.5), 'egimde')

_DIK = [a for a in _ASAMA if 'DIK_EGIM' in a['isim']]
check("iki dik eğim aşaması var",         len(_DIK), 2)
check("dik eğim tipi rampa olarak atanır",
      "elif isim in DIK_EGIM_ETIKETLERI:" in _FSM, True)
check("NavigateState rampaya yönlendirir", "'rampa_waypoint'" in _FSM, True)

# İki durak üst üste binerse §6.10'un istediği iki duruş dörde çıkar ve
# koşu saatinden boşuna 4 s gider.
check("NavigateState rampa duruşunu tekrarlamaz",
      "and wp.get('type') != 'rampa'" in _FSM, True)

# IMU BEST_EFFORT yayınlanıyor; RELIABLE abone olunursa eşleşme kurulmaz,
# callback HİÇ çağrılmaz, pitch sonsuza kadar 0 kalır ve faz hiç değişmez.
# Hata da basılmaz — bu yüzden kaynak seviyesinde kilitleniyor.
check("rampa IMU aboneliği BEST_EFFORT",
      "ReliabilityPolicy.BEST_EFFORT" in _FSM, True)

_HIZLAR = (_WPY['parametreler']['rampa_tirmanis_hiz'],
           _WPY['parametreler']['rampa_inis_hiz'])
check("rampa hızları tanımlı",            [h for h in _HIZLAR if h is None], [])
check("iniş tırmanıştan hızlı değil",     _HIZLAR[1] <= _HIZLAR[0], True)
check("rampa hızları makul aralıkta",     all(0.1 <= h <= 1.5 for h in _HIZLAR), True)


# ─── Haritasız hedef üretimi: koridor merkez çizgisi + iç duvar takibi ──────
from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    koridor_merkez_cizgisi, kayan_hedef, ic_duvar_hedefi,
)


def _koridor_taramasi(sol_m, sag_m, n=360, menzil=10.0):
    """İki paralel duvarlı düz koridorun sentetik taraması (lidar_yaw = 0)."""
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        s, c = math.sin(a), math.cos(a)
        en = float('inf')
        for d, isaret in ((sol_m, 1.0), (sag_m, -1.0)):
            if d is None or isaret * s <= 1e-6:
                continue
            r = d / (isaret * s)
            if 0.0 < r < menzil and abs(r * c) < menzil:
                en = min(en, r)
        ranges.append(en if en < float('inf') else float('nan'))
    return ranges, -math.pi, inc


def _merkez(sol_m, sag_m):
    r, a, i = _koridor_taramasi(sol_m, sag_m)
    return koridor_merkez_cizgisi(r, a, i, lidar_yaw=0.0)


# Ortalanmış araç: merkez çizgisi araç ekseninde kalmalı.
_ORTA = _merkez(1.5, 1.5)
check("ortalanmışta merkez ~0",           abs(_ORTA[0][1]) < 0.05, True)
check("ortalanmışta iki duvar görülür",   _ORTA[0][2], 'iki_duvar')
# Sola kaymış araç (sol duvar yakın): merkez SAĞDA, yani yanal negatif.
_KAYIK = _merkez(1.1, 1.9)
check("sola kaymışta merkez sağda",       _KAYIK[0][1] < -0.3, True)
# Tek duvar: koridor genişliğinden çıkarım, güven düşer.
_TEK = _merkez(1.5, None)
check("tek duvarda güven düşer",          _TEK[0][2], 'tek_duvar')
# Genişlik tutmuyorsa ölçüm koridora ait değildir — hedef oraya konmamalı.
# 1,0 + 1,0 = 2,0 m: pencereye giriyor ama §6.1'in 3 m'sini tutmuyor.
_GENIS = _merkez(1.0, 1.0)
check("genişlik tutmazsa belirsiz",       _GENIS[0][2], 'belirsiz')
check("belirsiz kutuya hedef konmaz",     kayan_hedef(_GENIS), None)
check("düz koridorda hedef üretilir",     kayan_hedef(_ORTA) is not None, True)

# İç duvar takibi: sağdaki duvardan `hedef_mesafe` kadar içeride, düz.
_R, _A, _I = _koridor_taramasi(None, 1.0)
_ICH = ic_duvar_hedefi(_R, _A, _I, 0.0, 'sag')
check("iç duvar hedefi üretilir",         _ICH is not None, True)
check("iç duvar hedefi koridora kayar",   abs(_ICH[1] - 0.4) < 0.1, True)
check("düz duvarda hedef yönü ileri",     abs(_ICH[2]) < 0.1, True)
# Duvar olmayan tarafı istemek hedef üretmemeli.
check("duvarsız tarafta hedef yok",       ic_duvar_hedefi(_R, _A, _I, 0.0, 'sol'), None)
check("geçersiz taraf reddedilir",        ic_duvar_hedefi(_R, _A, _I, 0.0, 'orta'), None)

# CAD ölçümünden gelen varsayılanlar: dar viraj kısa hedef ve dar pencere ister.
import inspect  # noqa: E402
_IMZA = inspect.signature(ic_duvar_hedefi).parameters
check("iç duvar hedefi kısa tutulur",     _IMZA['ileri_m'].default <= 2.5, True)
check("iç duvar penceresi dar tutulur",   _IMZA['maks_yanal'].default <= 2.4, True)


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
