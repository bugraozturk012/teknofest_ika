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
import glob
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    ackermann_steering,
    ackermann_komut,
    paket_olustur,
    paket_dogrula,
    paket_v0_i,
    paket_v1_i,
    paket_v0_u,
    paket_v1_u,
    paket_int32,
    gaz_binde_us,
    kip_us,
    rc_dizisi,
    surum_uyumlu,
    yaw_kovaryansi,
    ayar_ham,
    calib_stat_coz,
    ayar_deger,
    ayar_gonderilecek,
    AYAR_DIREKSIYON,
    enkoder_sessiz,
    kip_modu,
    bms_okuma_gecerli,
    bms_dip_olu,
    kesme_estop,
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
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC, PAS_HAKKI, PAS_GECILEMEZ,
    KOSU_SURESI_S, BEKLENEN_PROTOKOL_SURUMU,
    KART_KIP_MANUEL, KART_KIP_BOS, KART_KIP_OTONOM,
    SERIAL_ODOM, SERIAL_BAUD_KART, SERIAL_BAUD_TARET,
    KART_HIZ_TAVAN, KART_HIZ_TABAN, KART_HIZ_OLU_BOLGE,
    DRM_KESME, HATA_GAZ_YOK,
    BMS_YAS_ESIK_S, BMS_HUCRE_DIP_MV, BMS_HUCRE_UYARI_MV, BMS_HUCRE_SAYISI,
    TABELA_GORUS_ZAMAN_ASIMI_S,
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
L = 1.44
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

# Eğrilik kırpması — R_min = L/tan(δ_max) = 1.44/tan(30°) = 2.494 m
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

# ─── 2. Sürüş Kartı Telemetri Alanlarının Çözülmesi ─────────────────────────
print("\n=== 2. Telemetri alanları ===")

# Kartın gönderdiği paketleri aynı kurallarla üretip geri okuyoruz: alan
# genişliği, işaret ve bayt sırası burada kilitlenir.
_p = paket_olustur(0x31, -1234, 0)
check("v0 işaretli okunur",        paket_v0_i(_p), -1234)
check("v1 işaretli okunur",        paket_v1_i(paket_olustur(0x32, 0, -900)), -900)
check("v0 işaretsiz okunur",       paket_v0_u(paket_olustur(0x35, 0x80, 0)), 0x80)
check("işaretsiz alan 32767'yi aşar",
      paket_v1_u(paket_olustur(0x35, 0, -1)), 65535)

# 0x30'un iki alanı tek int32'dir: üst alan yüksek 16 bit.
check("int32 birleşimi",  paket_int32(paket_olustur(0x30, 1, 0)),      65536)
check("int32 negatif",    paket_int32(paket_olustur(0x30, -1, -1)),       -1)
check("int32 alt alan",   paket_int32(paket_olustur(0x30, 0, 25)),        25)

# ─── 3. RC Uyumluluk Dizisi ─────────────────────────────────────────────────
print("\n=== 3. RC uyumluluk dizisi ===")

check("gaz binde 0 → nötr",     gaz_binde_us(0),      1500.0)
check("gaz binde +1000 → tam",  gaz_binde_us(1000),   2000.0)
check("gaz binde -1000 → tam geri", gaz_binde_us(-1000), 1000.0)
check("gaz binde taşarsa kırpılır", gaz_binde_us(5000), 2000.0)

check("otonom kip üst uç", kip_us(KART_KIP_OTONOM), 2000.0)
check("manuel kip alt uç", kip_us(KART_KIP_MANUEL), 1000.0)
# Kullanılmayan orta kip güvenli tarafa yazılmalı: ham CH9 orta konumda mod
# eşiğinin üstüne düşüyor ve eşikleyen taraf onu otonom okurdu.
check("kullanılmayan kip manuel sayılır", kip_us(KART_KIP_BOS), 1000.0)
# CH9 üç konumlu ve orta konumu 1500 µs; ham değeri eşikleyen her mantık o
# konumu otonom okurdu. kip_us onu güvenli uca yazarak bu tuzağı kapatıyor.
_CH9_ORTA_US = 1500.0
check("kip alanı orta konumu otonom okumaz",
      kip_us(KART_KIP_BOS) >= _CH9_ORTA_US, False)
check("ham eşikleme aynı konumu otonom okurdu",
      _CH9_ORTA_US >= _CH9_ORTA_US, True)

_d = rc_dizisi(gaz_binde=500, ham_ch1=1620.0, kip=KART_KIP_OTONOM,
               taret_aktif=True, ham_ch9=1750.0)
check("dizi uzunluğu",        len(_d),  7)
check("dizi[0] gaz",          _d[0], 1750.0)
check("dizi[1] ham CH1",      _d[1], 1620.0)
check("dizi[2] kipten türer", _d[2], 2000.0)
check("dizi[3] lazer kapalı", _d[3], 1000.0)
check("dizi[4] tilt nötr",    _d[4], 1500.0)
check("dizi[5] taret aktif",  _d[5], 2000.0)
check("dizi[6] ham CH9",      _d[6], 1750.0)
# Ham CH9 mod alanına sızarsa üç konumlu anahtarın ortası otonom okunur.
check("ham CH9 mod alanına sızmaz",
      rc_dizisi(0, 1500.0, KART_KIP_MANUEL, False, 1900.0)[2], 1000.0)

# ─── 4. Sürüm Karşılaştırması ───────────────────────────────────────────────
print("\n=== 4. Protokol sürümü ===")

check("aynı protokol uyumlu",   surum_uyumlu(BEKLENEN_PROTOKOL_SURUMU,
                                             BEKLENEN_PROTOKOL_SURUMU), True)
check("farklı protokol uyumsuz", surum_uyumlu(BEKLENEN_PROTOKOL_SURUMU + 1,
                                              BEKLENEN_PROTOKOL_SURUMU), False)

# Eşik kartınkiyle aynı olmalı: kart sys < 3'te HATA_BNO_KALIB basıyor, yani
# sys == 2'de yaw'ı güvenilir saymak EKF'e kartın reddettiği ölçümü tam
# ağırlıkla vermek olur. Arka aks tek parça, yönün ikinci kaynağı yok.
check("sys 3 → yaw güvenilir",  yaw_kovaryansi(3), 0.02)
check("sys 2 → yaw şüpheli",    yaw_kovaryansi(2), 0.30)
check("sys 1 → yaw şüpheli",    yaw_kovaryansi(1), 0.30)
check("sys 0 → yaw şüpheli",    yaw_kovaryansi(0), 0.30)
check("kalibre yaw, kalibresizden dar", yaw_kovaryansi(3) < yaw_kovaryansi(2), True)

# ─── Kart ayar paketi (0x09) ────────────────────────────────────────────────
print("\n=== 5e. Kart ayarları ===")
# Ölçekler kart sözleşmesinden; iki tarafın ayrı çarpan tutması sayının
# sessizce on kat yanlış girilmesinin en kolay yolu. Örnekler sözleşmeden.
check("çevre 1842,5 mm → 18425", ayar_ham(1, 1842.5)[0], 18425)
check("dişli oranı 3,25 → 3250", ayar_ham(2, 3.25)[0],   3250)
check("direksiyon 12,4 → 12400", ayar_ham(3, 12.4)[0],   12400)
check("darbe/tur 6,0 → 60",      ayar_ham(4, 6.0)[0],    60)
check("işaret -1 ham gider",     ayar_ham(5, -1.0)[0],   -1)
# int16 taşması sessizce sarmamalı: 3,2 m üstü çevre kartta anlamsız bir
# sayıya dönüşür ve odometri bir daha hiç doğru olmaz.
check("aralık dışı gönderilmez", ayar_ham(1, 4000.0)[0], None)
check("aralık dışı sebebi",      ayar_ham(1, 4000.0)[1], 'aralik_disi')
check("bilinmeyen kimlik gönderilmez", ayar_ham(9, 1.0)[0], None)
# Geri çevrim: kartın döndürdüğü ham sayı aynı ölçekten okunmalı.
check("geri çevrim tutuyor", ayar_deger(1, 18425), 1842.5)
check("bilinmeyen kimlik çözülmez", ayar_deger(9, 1), None)
# 🔑 SIFIR = ÖLÇÜLMEDİ. Ölçülmemiş bir sayıyı göndermek, kartın bilerek
# sustuğu alana uydurma değer yazmaktır — ve hiçbiri girilmeden köprünün
# bugünkü davranışı değişmemeli.
check("sıfır ayar gönderilmez",  ayar_gonderilecek(0.0),   False)
check("ölçülmüş ayar gönderilir", ayar_gonderilecek(1842.5), True)
# Kartın kabul kuralları bizde de uygulanır: 1-4 için sıfır ve negatif
# reddediliyor (0 yazmak "ölçüm yok" demek ve hız alanını sessizce sıfırlar),
# kimlik 5 yalnız ±1. Erken elemek, karşılaştırma alarmını beklemekten iyi.
check("negatif çevre gönderilmez",  ayar_ham(1, -1842.5)[0], None)
check("negatif çevre sebebi",       ayar_ham(1, -1842.5)[1], 'pozitif_olmali')
check("sıfır çevre gönderilmez",    ayar_ham(1, 0.0)[0],     None)
check("işaret +1 geçerli",          ayar_ham(5, 1.0)[0],     1)
check("işaret 0 geçersiz",          ayar_ham(5, 0.0)[0],     None)
check("işaret 2 geçersiz",          ayar_ham(5, 2.0)[0],     None)
check("işaret sebebi",              ayar_ham(5, 2.0)[1],     'isaret_gecersiz')

# ─── 5. Binary Paket CRC (seri_kopru.py protokolü) ──────────────────────────
print("\n=== 5. Binary Protokol CRC ===")

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

# ─── 5b. Sürüş Kartı Bağlantı Ayarı — üç yerde aynı olmalı ──────────────────
print("\n=== 5b. Bağlantı ayarı tutarlılığı ===")

# Sahada koşan yol scripts/lydia_startup.sh; launch dosyaları kullanılmıyor.
# Port ya da baud yalnız bir yerde değiştirilirse köprü ya eski cihaza bağlanır
# ya da hiç açılamaz, ve bunun tek belirtisi sessiz bir köprüdür.
_KOK = os.path.dirname(os.path.abspath(__file__))

check("port sabiti", SERIAL_ODOM, "/dev/f767")
check("kart baud sabiti", SERIAL_BAUD_KART, 921600)
check("taret baud'u ayrı", SERIAL_BAUD_TARET, 115200)

with open(os.path.join(_KOK, 'scripts/lydia_startup.sh'), encoding='utf-8') as f:
    _BETIK = f.read()
_m_port = re.search(r'-p\s+port:=(\S+)', _BETIK)
_m_baud = re.search(r'-p\s+baud:=(\S+)', _BETIK)
check("boot betiği portu geçiyor", _m_port is not None, True)
check("boot betiği baud'u geçiyor", _m_baud is not None, True)
# Port artık doğrudan yazılmıyor, $SERI_PORT değişkeninden geliyor: elektrik
# ekibi ayrı bir komut hattı çekerse betiği düzenlemeden geçebilsin diye.
# Denetim bu yüzden iki kademeli — dolaylılık korunmalı VE varsayılan doğru
# olmalı. Yalnız birine bakmak, değişkenin yanlış bir cihaza kurulmasını
# görmezdi.
if _m_port:
    check("boot betiği portu değişkenden alıyor", _m_port.group(1), '"$SERI_PORT"')
_m_seri = re.search(r':\s*"\$\{SERI_PORT:=([^}]+)\}"', _BETIK)
check("SERI_PORT varsayılanı tanımlı", _m_seri is not None, True)
if _m_seri:
    check("SERI_PORT varsayılanı sabitle aynı", _m_seri.group(1), SERIAL_ODOM)
if _m_baud:
    check("boot betiği baud'u sabitle aynı", int(_m_baud.group(1)), SERIAL_BAUD_KART)

with open(os.path.join(_KOK, 'launch/gercek_arac.launch.py'), encoding='utf-8') as f:
    _LAUNCH = f.read()
# Launch dosyasında başka düğümlerin de port parametresi var (LiDAR gibi);
# arama seri_kopru bloğuyla sınırlanır.
_sk_bas   = _LAUNCH.index('seri_kopru = Node(')
_sk_blok  = _LAUNCH[_sk_bas:_sk_bas + 900]
_l_port = re.search(r"'port':\s*'([^']+)'", _sk_blok)
_l_baud = re.search(r"'baud':\s*(\d+)", _sk_blok)
check("launch portu sabitle aynı", _l_port.group(1) if _l_port else None, SERIAL_ODOM)
check("launch baud'u sabitle aynı", int(_l_baud.group(1)) if _l_baud else None, SERIAL_BAUD_KART)

# Eski Mega bloğu köprüde kalmamalı: 0x10-0x23 aralığını bekleyen bir çözücü
# yeni kartın hiçbir paketiyle örtüşmez ve sessizce hepsini atar.
with open(os.path.join(_KOK, 'teknofest_ika/gomulu/seri_kopru.py'),
          encoding='utf-8') as f:
    _SK_KAYNAK = f.read()


def _govde(kaynak: str, imza: str) -> str:
    """Bir metodun gövdesi — arama fonksiyon sınırında durmalı."""
    bas = kaynak.find(imza)
    if bas < 0:
        return ''
    son = kaynak.find('\n    def ', bas + 1)
    return kaynak[bas:son if son > 0 else len(kaynak)]


for _kod in ('0x10', '0x11', '0x12', '0x13', '0x20', '0x21', '0x22', '0x23'):
    check(f"köprü {_kod} beklemiyor", _kod in _SK_KAYNAK, False)
for _kod in ('0x30', '0x31', '0x32', '0x33', '0x34', '0x35',
             '0x36', '0x37', '0x38', '0x39', '0x3A', '0x3B', '0x3C'):
    check(f"köprü {_kod} tanıyor", _kod in _SK_KAYNAK, True)

# Eşik köprüde gömülü kalmamalı: kartınkiyle aynı tutulan tek yer saf
# fonksiyon, kopyalanan bir sabit sessizce ayrışır.
check("köprü yaw eşiğini saf fonksiyondan alıyor",
      'yaw_kovaryansi(' in _SK_KAYNAK, True)
check("köprüde gömülü kalibrasyon eşiği yok",
      'sys_kalib >= 2' in _SK_KAYNAK, False)

# Yapı numarası KARŞILAŞTIRILMAZ ama DEĞİŞİMİ izlenir: kartta haber verilmemiş
# bir sabit değişikliğinin (hız tavanı, zaman aşımı, ölçek) tek görünür izi bu.
check("köprü yapı numarasını takip ediyor", 'self._yapi' in _SK_KAYNAK, True)

# Heartbeat penceresi kartta 700 ms. 400 ms'lik gönderim tek paketlik pay
# bırakıyordu: bir heartbeat düşünce 800 ms > 700 ms olup kart Jetson'ı ölü
# sayıyor ve gaz kesiliyordu. Sürüş komutu aktığı sürece görünmez, ama komut
# akışının bilerek kesildiği anlarda (atış duraklatması) pay gerçekten dardı.
_HB = re.search(r'create_timer\(([\d.]+),\s*self\._hb_gonder\)', _SK_KAYNAK)
check("heartbeat zamanlayıcısı bulundu", _HB is not None, True)
check("heartbeat penceresinin en az üçte biri hızında",
      float(_HB.group(1)) <= 0.7 / 3.0 if _HB else False, True)

# Jetson'ı besleyen paket (0x3D) traksiyon paketinden AYRI konuda: kimyaları
# ve eşikleri farklı, biri bitince ötekinden işaret gelmiyor.
check("köprü 0x3D tanıyor", 'PKT_F7_BATARYA' in _SK_KAYNAK, True)
check("Jetson paketi ayrı konuda", 'BATTERY_JETSON_TOPIC' in _SK_KAYNAK, True)
# Sağlık kararı paket geriliminden değil EN DÜŞÜK HÜCREDEN verilir.
_BAT = _govde(_SK_KAYNAK, '    def _batarya_isle')
check("Jetson paketi sağlığı hücre dibinden", 'dip_mv <' in _BAT, True)
# Kimya teyit edilmedi; belirsizlikte yüksek (LiPo) eşik seçilir — erken
# uyarının bedeli bir log satırı, geç uyarınınki paket.
from teknofest_ika.otonomi.topics import (   # noqa: E402
    BMS_JETSON_UYARI_MV, BMS_HUCRE_UYARI_MV,
)
check("Jetson eşiği traksiyon eşiğinden yüksek",
      BMS_JETSON_UYARI_MV > BMS_HUCRE_UYARI_MV, True)
check("yapı değişimi uyarı basıyor",
      'FIRMWARE DEĞİŞTİ' in _SK_KAYNAK, True)

# ATIŞ KİLİDİ — PKT_J_DUR ile durdurulamaz. Kart o paketi jetson_dur olarak
# mandallıyor ve otonom kipte aynı bayrak lazer isteğini düşürüp tareti
# merkeze döndürüyor: ateş komutu kendi ateşini iptal eder ve kilitli bir
# döngü kurulur (istek → DUR → taret ölür → onay gelmez → istek sürer).
# Kilit /ackermann_cmd yolunda durmalı. Aramayı fonksiyona hapsetmezsek
# `if self._lazer_aktif:` ifadesi zaman aşımı dalında da geçtiği için,
# buradaki dal tümüyle silinse bile denetim geçerdi.
_CMD_CB  = _govde(_SK_KAYNAK, '    def _cmd_cb')
_L_PARCA = _CMD_CB.split('if self._lazer_aktif:')
_LAZER_DALI = _L_PARCA[1].split('return')[0] if len(_L_PARCA) > 1 else ''
check("sürüş komutunda atış kilidi var", len(_L_PARCA) > 1, True)
check("atış durdurması ayrı yolda", '_atis_duraklat()' in _LAZER_DALI, True)
check("atış durdurması DUR kullanmıyor", 'PKT_J_DUR' in _LAZER_DALI, False)
# Komut akışı atış sırasında bilerek kesiliyor; zaman aşımı dalı da DUR'a
# düşerse taret aynı şekilde ölür.
_ZA_DALI = _govde(_SK_KAYNAK, '    def _guvenlik_kontrol')
check("zaman aşımı dalı atışı gözetiyor",
      'if self._lazer_aktif:' in _ZA_DALI, True)
# E-STOP gerçek acil durum: taretin sönmesi ve merkeze dönmesi İSTENEN
# sonuçtur, DUR orada doğru pakettir. Kol kendi içinde denetlenir — dalın
# herhangi bir yerinde DUR aramak, atış kolundaki DUR'u sayıp geçerdi.
_ZA_ESTOP = _ZA_DALI.split('if self._e_stop_aktif:')
check("zaman aşımı dalında E-STOP kolu var", len(_ZA_ESTOP) > 1, True)
check("E-STOP acil durum paketini basıyor",
      'PKT_J_DUR' in _ZA_ESTOP[1].split('return')[0] if len(_ZA_ESTOP) > 1 else False,
      True)
# Fren atış boyunca /fren_komut sahibinde tutulur; köprüden basılan bir fren
# 20 Hz yayının altında 50 ms içinde üzerine yazılırdı.
with open(os.path.join(_KOK, 'teknofest_ika/otonomi/ackermann_converter.py'),
          encoding='utf-8') as f:
    _ACK = f.read()
check("fren sahibi atışı biliyor", 'SHOOT_CMD_TOPIC' in _ACK, True)

# Gösterge ucu ileri hızın kaynağı olacak; sustuğunda 0x31 sıfıra düşer, EKF
# konumu ilerlemez ve Nav2 hedefi "ilerleme yok" diye iptal eder. Bayrağın
# sessizce yutulması, otonom koşunun neden yürümediğini görünmez kılardı.
check("gösterge sessizliği raporlanıyor",
      'HATA_GOST_SESSIZ' in _SK_KAYNAK, True)

# Kalıcılık kartta DEĞİL köprüde: kart ayarları flash'a yazmıyor, resette
# hepsi sıfırlanıyor ve 0x31 sessizce 0 basmaya dönüyor.
check("köprü ayar paketi gönderiyor", 'PKT_J_AYAR' in _SK_KAYNAK, True)
check("köprü 0x3E geri bildirimini okuyor", 'PKT_F7_AYAR' in _SK_KAYNAK, True)
_CAL = _govde(_SK_KAYNAK, '    def _calisma_isle')
check("kart ilk görüldüğünde ayarlar gidiyor",
      "_ayarlari_gonder('kart ilk görüldü')" in _CAL, True)
check("kart resetinde ayarlar yeniden gidiyor",
      "_ayarlari_gonder('kart resetlendi')" in _CAL, True)
# Reset sonrası eski karşılaştırma tabanı da düşmeli, yoksa kart temiz
# değerlerle açılırken "uyuşmuyor" alarmı basılır.
check("resette gönderilen kaydı temizleniyor",
      '_ayar_gonderilen.clear()' in _CAL, True)
# Kart ayarları RAM'de: panoda görünmezlerse "kart neye göre çalışıyor"
# sorusunun cevabı hiçbir yerde yok. Turnike yayını 5 saniyede tam tur atıyor.
with open(os.path.join(_KOK, 'scripts/web_dashboard.py'), encoding='utf-8') as f:
    _PANO_AYAR = f.read()
check("pano ayar turnikesini dinliyor", 'KART_AYAR_TOPIC' in _PANO_AYAR, True)
check("pano beş ayarı da gösteriyor",
      all(a in _PANO_AYAR for a in
          ('ayar_cevre', 'ayar_disli', 'ayar_direksiyon', 'ayar_darbe',
           'ayar_isaret')), True)
# Ölçek panoda ikinci kez tutulmamalı — sayı burada on kat yanlış görünürdü.
check("pano ölçeği saf fonksiyondan alıyor", 'ayar_deger(' in _PANO_AYAR, True)
# Enkoder sessizliğinde olduğu gibi: yalnız uyarı, karşı davranış YOK. Kaynağı
# henüz bağlanmamış araçta bir kilit, sürüşü kendi kendine durdururdu.
_GOST_BAS  = _SK_KAYNAK.find('if yeni & HATA_GOST_SESSIZ:')
_GOST_DALI = (_SK_KAYNAK[_GOST_BAS:_SK_KAYNAK.index('if yeni & HATA_GAZ_YOK:', _GOST_BAS)]
              if _GOST_BAS >= 0 else '')
check("gösterge bayrağına eylem bağlı değil",
      _GOST_BAS >= 0 and '_paket_gonder' not in _GOST_DALI, True)
check("atışta tam fren basılıyor",
      'if atis_aktif:' in _ACK and 'FREN_GUVENLI_DUR_BINDE' in
      _ACK[_ACK.index('if atis_aktif:'):_ACK.index('if atis_aktif:') + 500], True)

# Kip 1 kumandadan verilen yumuşak E-STOP. Panoda boş bir işaretle göstermek
# operatörü kilitli araçta arıza aramaya yollar.
with open(os.path.join(_KOK, 'scripts/web_dashboard.py'), encoding='utf-8') as f:
    _PANO = f.read()
_kip_bas    = _PANO.find('_KIP_ADI')
_kip_satiri = _PANO[_kip_bas:_kip_bas + 200] if _kip_bas >= 0 else ''
check("pano kip 1'i boş göstermiyor", "KART_KIP_BOS: '—'" in _kip_satiri, False)
check("pano kip 1'i kilitli gösteriyor", 'BOŞ/DUR' in _kip_satiri, True)

# MANUEL LAZER TETİĞİ — geri konmamalı. Sürüş kartı ham CH3'ü göndermiyor;
# /rc_input'un aux alanı sabit bir değer taşıyor, yani o alanı eşikleyen bir
# tetik hiçbir koşulda ateşlenemez. Sessizce çalışmayan bir güvenlik yolu,
# hiç olmayandan kötüdür: operatör anahtarı çevirir ve neden ateş etmediğini
# arar. Lazer yetkisi /shoot_command'ın sahiplerinde.
with open(os.path.join(_KOK, 'teknofest_ika/otonomi/mod_yoneticisi.py'),
          encoding='utf-8') as f:
    _MOD = f.read()
check("mod yöneticisi aux alanını okumuyor", 'msg.data[3]' in _MOD, False)
check("mod yöneticisi lazer yayınlamıyor", 'SHOOT_CMD_TOPIC' in _MOD, False)

# DİNGİL ARASI beş ayrı yerde yaşıyor ve Ackermann kinematiğinin tek girdisi.
# Tek kaynağa indirmek ayrı bir iş; o zamana kadar denetim beşinin AYNI sayıyı
# taşıdığını doğruluyor. Bir yerde güncellenip ötekilerde unutulması, sahada
# yalnız "araç virajı geniş/dar alıyor" diye görünür ve hiçbir logda çıkmaz.
_L_BEKLENEN = '1.44'
for _dosya, _anahtar in (
        ('config/ekf.yaml',                             'Dingil arası'),
        ('config/nav2_params.yaml',                     'Dingil arası'),
        ('teknofest_ika/otonomi/ackermann_converter.py', "declare_parameter('wheelbase'"),
        ('launch/gercek_arac.launch.py',                "'wheelbase':"),
):
    with open(os.path.join(_KOK, _dosya), encoding='utf-8') as f:
        _SATIRLAR = [l for l in f if _anahtar in l]
    check(f"{_dosya}: dingil arası {_L_BEKLENEN}",
          bool(_SATIRLAR) and any(_L_BEKLENEN in l for l in _SATIRLAR), True)

# ── ARAÇ ÖLÇÜLERİ — URDF'in DÜŞEY geometrisi ve iz genişliği ──────────────
# Gövde kutusunun eni/boyu/yüksekliği aşağıda ayrıca sınanıyor; burada
# sayılar metin olarak değil URDF'ten ÇÖZÜLÜP aritmetikle doğrulanıyor.
# Bir sayı elle değiştirilip ötekiler unutulursa kutu havada kalır ya da
# tekerlek yere gömülür — ikisi de yalnız rviz'de gözle görülür.
def _kaynak_ham(yol):
    with open(os.path.join(_KOK, yol), encoding='utf-8') as f:
        return f.read()


import xml.etree.ElementTree as _ET25
_U25_ROOT = _ET25.parse(os.path.join(_KOK, 'urdf/arac.urdf')).getroot()


def _u25_xyz(e):
    return [float(v) for v in e.get('xyz').split()]


# Araçtan ölçülen (11 Eylül): en 1,17 · toplam yükseklik 0,78 ·
# zemin boşluğu 0,365 (şasi altı) · iz genişliği 1,00 · dingil arası 1,44
_U25_TOPLAM_Y, _U25_BOSLUK, _U25_IZ, _U25_L = 0.78, 0.365, 1.00, 1.44

_u25_bj = [j for j in _U25_ROOT.iter('joint') if j.get('name') == 'base_joint'][0]
_U25_BASE_Z = _u25_xyz(_u25_bj.find('origin'))[2]
_u25_govde = [l for l in _U25_ROOT.iter('link') if l.get('name') == 'base_link'][0]
_u25_vis = _u25_govde.find('visual')
_U25_KUTU = [float(v) for v in _u25_vis.find('geometry').find('box').get('size').split()]
_U25_KUTU_Z = _u25_xyz(_u25_vis.find('origin'))[2]

# 🔑 Kutu ZEMİNE DEĞMİYOR: şasi altı 0,365 m yukarıda. Eskiden kutu zemine
# oturuyordu ve yüksekliği toplam yükseklik sanılıyordu.
check("kutu yüksekliği = toplam − zemin boşluğu",
      round(_U25_KUTU[2], 4), round(_U25_TOPLAM_Y - _U25_BOSLUK, 4))
check("kutu altı = zemin boşluğu",
      round(_U25_BASE_Z + _U25_KUTU_Z - _U25_KUTU[2] / 2.0, 4), _U25_BOSLUK)
check("kutu üstü = toplam yükseklik",
      round(_U25_BASE_Z + _U25_KUTU_Z + _U25_KUTU[2] / 2.0, 4), _U25_TOPLAM_Y)

# Tekerlek merkezleri: |x| = L/2, |y| = iz/2. Dördü de sınanıyor — iz
# genişliği ÖLÇÜLENE kadar depoda 0,50/0,67/0,900 diye üç değer dolaşıyordu.
_u25_tekerler = {j.get('name'): _u25_xyz(j.find('origin'))
                 for j in _U25_ROOT.iter('joint')
                 if j.get('name', '').split('_')[0] in ('sag', 'sol')}
check("dört teker joint'i bulundu", len(_u25_tekerler), 4)
for _u25_ad, _u25_o in sorted(_u25_tekerler.items()):
    check(f"{_u25_ad}: |x| = L/2",  round(abs(_u25_o[0]), 4), round(_U25_L / 2.0, 4))
    check(f"{_u25_ad}: |y| = iz/2", round(abs(_u25_o[1]), 4), round(_U25_IZ / 2.0, 4))
check("ön tekerler +x",
      all(o[0] > 0 for a, o in _u25_tekerler.items() if '_on_' in a), True)
check("arka tekerler -x",
      all(o[0] < 0 for a, o in _u25_tekerler.items() if '_arka_' in a), True)

# ── LiDAR yüksekliği ZEMİNDEN ölçülür, TF base_link'e göre yayınlanır ─────
# 🔴 Ölçülen sayı doğrudan TF'e konursa tarama düzlemi base_link'in yüksekliği
# kadar (0,28 m) fazla yükseğe yerleşir. İki yer de aynı ZEMİN yüksekliğini
# vermeli: urdf lidar_joint (base_link göreli) ve açılış betiği (zeminden
# ölçüp farkı düşüyor).
_U25_LIDAR_ZEMIN = 0.60          # mezürle ölçüldü
_u25_lj = [j for j in _U25_ROOT.iter('joint') if j.get('name') == 'lidar_joint'][0]
_u25_lidar_z = _u25_xyz(_u25_lj.find('origin'))[2]
check("urdf lidar_joint parent base_link",
      _u25_lj.find('parent').get('link'), 'base_link')
check("urdf: base_link + lidar_joint = zemin yüksekliği",
      round(_U25_BASE_Z + _u25_lidar_z, 4), _U25_LIDAR_ZEMIN)

_u25_st = _kaynak_ham('scripts/lydia_startup.sh')
check("betikte LIDAR_Z_M zeminden", ': "${LIDAR_Z_M:=0.60}"' in _u25_st, True)
check("betik base_link farkını düşüyor",
      '$LIDAR_Z_M - $BASE_LINK_Z_M' in _u25_st, True)
check("betikteki base_link yüksekliği urdf ile aynı",
      ': "${BASE_LINK_Z_M:=0.2989}"' in _u25_st, True)
# base_link yüksekliği = tekerlek yarıçapı + teker joint ofseti (0,10).
# Yarıçap ölçümden: yuvarlanma çevresi 1250 mm → r = 1,250/2π = 0,1989.
check("urdf base_joint = yarıçap + 0.10", _U25_BASE_Z, 0.2989)
_u25_sil = [l for l in _U25_ROOT.iter('link')
            if l.get('name', '').endswith('teker')]
check("dört teker link'i", len(_u25_sil), 4)
for _u25_l in _u25_sil:
    _u25_r = float(_u25_l.find('visual').find('geometry').find('cylinder').get('radius'))
    check(f"{_u25_l.get('name')} yarıçapı çevreden", _u25_r, 0.1989)
# Ham sayı doğrudan TF'e verilmemeli.
check("TF'e ham LIDAR_Z_M verilmiyor",
      'static_transform_publisher 0 0 "$LIDAR_Z_M"' in _u25_st, False)



# ─── 5c. Kart Hız Kısıtları ve Enkoder Sessizliği ───────────────────────────
print("\n=== 5c. Kart kısıtları ===")

# Kart komutu reddetmiyor, kırpıyor. Bizim tarafta daha yüksek bir tavan
# tutmak komutu gerçekleşmeyecek bir değere kilitler ve gönderdiğimizle
# 0x38'de okuduğumuz arasında kalıcı fark üretir.
check("kart hız tavanı",    KART_HIZ_TAVAN,     1.50)
check("kart hız tabanı",    KART_HIZ_TABAN,     0.20)
check("kart ölü bölgesi",   KART_HIZ_OLU_BOLGE, 0.01)

with open(os.path.join(_KOK, 'teknofest_ika/gomulu/seri_kopru.py'),
          encoding='utf-8') as f:
    _SK2 = f.read()
check("köprü tavanı kart tavanına bağlı",
      'MAX_HIZ_MS     = KART_HIZ_TAVAN' in _SK2, True)
check("köprü sabit bir tavan yazmıyor",
      re.search(r'MAX_HIZ_MS\s*=\s*[0-9]', _SK2) is None, True)

with open(os.path.join(_KOK, 'teknofest_ika/otonomi/ackermann_converter.py'),
          encoding='utf-8') as f:
    _AC2 = f.read()
check("ackermann tavanı kart tavanına bağlı",
      "declare_parameter('max_speed', KART_HIZ_TAVAN)" in _AC2, True)

with open(os.path.join(_KOK, 'teknofest_ika/otonomi/misyon_fsm.py'),
          encoding='utf-8') as f:
    _FSM2 = f.read()
check("hızlanma tavanı kart tavanına bağlı",
      'MAX_HIZ          = KART_HIZ_TAVAN' in _FSM2, True)

# Enkoder sessizliği: kartın kendi bayrağı ölü olduğu için denetim bizde.
check("hareket var, sayım sabit → sessiz",
      enkoder_sessiz(hiz_mms=400, sayim_sabit_s=2.0), True)
check("hareket var, sayım yeni değişti → sessiz değil",
      enkoder_sessiz(hiz_mms=400, sayim_sabit_s=0.2), False)
check("araç duruyor, sayım sabit → sessiz değil",
      enkoder_sessiz(hiz_mms=0, sayim_sabit_s=10.0), False)
check("geri giderken de yakalar",
      enkoder_sessiz(hiz_mms=-400, sayim_sabit_s=2.0), True)
# Kartın kalkış tabanı 0,20 m/s; gerçek bir sürüş komutu her zaman eşiğin
# üstünde kalır, yani eşik taban altındaki gürültüye takılmamalı.
check("eşik kart tabanının altında",
      enkoder_sessiz(hiz_mms=int(KART_HIZ_TABAN * 1000), sayim_sabit_s=2.0), True)

# Kart IMU paketlerini yalnız BNO takılıyken basıyor; çip yokken susması
# arıza değil. Sabit listede tutmak kalıcı sahte alarm demek.
with open(os.path.join(_KOK, 'teknofest_ika/otonomi/watchdog.py'),
          encoding='utf-8') as f:
    _WD = f.read()
_temel = _WD[_WD.index('TEMEL_TOPICLER = {'):_WD.index('BATARYA_TOPICLERI')]
check("IMU koşulsuz izlenmiyor",  'IMU_TOPIC' in _temel, False)
check("batarya koşulsuz izlenmiyor", 'BATTERY_TOPIC' in _temel, False)
check("IMU için ayrı anahtar var", "declare_parameter('imu_izle'" in _WD, True)

# ─── 5d. Kip Otoritesi ve Kesme Kaynağı ─────────────────────────────────────
print("\n=== 5d. Kip ve kesme ===")

# Kip kararı kartta; Jetson eşiklemiyor.
check("kart otonom → FULL_AUTO", kip_modu(KART_KIP_OTONOM, False), 2)
check("kart manuel → MANUAL",    kip_modu(KART_KIP_MANUEL, False), 0)
check("kullanılmayan kip manuel", kip_modu(KART_KIP_BOS,   False), 0)
# Kart susarken otonom kalmak, elle sürülen bir araca komut basmak olur.
check("bayat kip manuele düşer", kip_modu(KART_KIP_OTONOM, True), 0)

# Kumandadan kesme (SwA): alıcı verici kapalıyken de yayın sürdürdüğü için
# çerçeve sessizliği sinyal kaybını yakalamıyor, kesme biti yakalıyor.
check("kesme biti → E-STOP",       kesme_estop(DRM_KESME, True), True)
check("kesme yokken E-STOP yok",   kesme_estop(0x00, True), False)
check("başka bit E-STOP üretmez",  kesme_estop(0x20, True), False)
# Veri gelmeden E-STOP basmak kaynağı açılışta kalıcı kilitler.
check("veri gelmeden E-STOP yok",  kesme_estop(DRM_KESME, False), False)

with open(os.path.join(_KOK, 'teknofest_ika/otonomi/mod_yoneticisi.py'),
          encoding='utf-8') as f:
    _MY = f.read()
check("mod kaynağı /kart/kip",     'KART_KIP_TOPIC' in _MY, True)
check("ham kanal eşiklenmiyor",    'rc_mod_otonom' in _MY, False)
check("kesme kaynağı DRM_KESME",   'DRM_KESME' in _MY, True)
# Eski kural /rc_input sessizliğini E-STOP'a çeviriyordu ve açılışta
# tetiklenip sistemi kalıcı olarak kilitliyordu.
check("RC sessizliği E-STOP üretmiyor",
      '_rc_estop_pub' in _MY or 'rc_kopuk' in _MY, False)

# Gaz arızası kilidi mandallı: kurtarılamaz durum, geçici hata değil.
with open(os.path.join(_KOK, 'teknofest_ika/otonomi/misyon_fsm.py'),
          encoding='utf-8') as f:
    _FSM3 = f.read()
check("FSM gaz arızasını izliyor", 'HATA_GAZ_YOK' in _FSM3, True)
check("durdurma kontrolü ortak",   'def durdurma_gerekli' in _FSM3, True)
# Alanı okuyan tek yer yardımcının kendisi olmalı: durum kontrolleri dağınık
# kalırsa gaz arızası yalnız bazı yerlerde durdurur.
check("e_stop alanını yalnız yardımcı okuyor",
      _FSM3.count("get_field('e_stop', False)"), 1)
check("kontroller yardımcıdan geçiyor",
      _FSM3.count('durdurma_gerekli(self.det_store)') >= 10, True)

# E-STOP hattı Jetson'a bağlı değil; boştaki pini okumak gürültüyle
# rastgele E-STOP üretir.
with open(os.path.join(_KOK, 'teknofest_ika/otonomi/e_stop_node.py'),
          encoding='utf-8') as f:
    _ES = f.read()
check("GPIO varsayılanı kapalı",
      "declare_parameter('gpio_mod',   False)" in _ES, True)
with open(os.path.join(_KOK, 'launch/gercek_arac.launch.py'), encoding='utf-8') as f:
    _LC = f.read()
check("launch GPIO'yu açmıyor", "'gpio_mod':   True" in _LC, False)

# ─── 6. Batarya Yüzdesi (4S LiPo) ───────────────────────────────────────────
print("\n=== 6. Batarya Yüzde (4S LiPo) ===")
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

# ── sticky'nin ÖMRÜ ────────────────────────────────────────────────────────
# 🔴 Sınırsız sticky, onaylanan değeri KOŞU SONUNA KADAR tutuyordu: aday boşa
# düşünce sayaç sıfırlanıyor, eşik hiç dolmuyor ve confirmed asla boşa
# dönemiyor. Sonucu, bir kez görülen tabelanın arazi profilinin (hız +
# inflation) bir daha bırakılmamasıydı — araç 0,65 yerine 0,45-0,50 ile
# koşuyu bitirir. 11 aşamanın yalnız birinin bitiş tabelası var, yani
# "sonraki tabelaya kadar sürsün" kuralı diğer onunda bölümü hiç bitirmiyor.
_TO = TABELA_GORUS_ZAMAN_ASIMI_S
g = ConsecutiveFrameFilter(2, bos_deger=NO_DETECTION, sticky=True,
                           sticky_zaman_asimi_s=_TO)
check("zaman aşımlı: 2 karede onay", [g.isle(5, 0.0), g.isle(5, 0.03)][-1], 5)
# Süre SON GERÇEK TESPİTTEN sayılıyor, onaydan değil.
check("eşiğin hemen altında hâlâ onaylı", g.isle(NO_DETECTION, 0.03 + _TO - 0.01), 5)
check("eşikte boşa dönüyor",              g.isle(NO_DETECTION, 0.03 + _TO), NO_DETECTION)
check("boşta kalmaya devam",              g.isle(NO_DETECTION, 0.03 + _TO + 5.0), NO_DETECTION)
# Aynı tabela geri gelirse normal onay süreci işliyor.
check("geri gelen tabela 1. kare", g.isle(5, 10.0), NO_DETECTION)
check("geri gelen tabela 2. kare", g.isle(5, 10.03), 5)

# Eşiğin ALTINDAKİ görüş kaybı profili DÜŞÜRMEMELİ — yoksa bölüm ortasında
# tabelayı bir saniye görememek aracı yanlış profile atar.
h = ConsecutiveFrameFilter(2, bos_deger=NO_DETECTION, sticky=True,
                           sticky_zaman_asimi_s=_TO)
h.isle(5, 0.0); h.isle(5, 0.03)
check("kısa görüş kaybı (eşiğin yarısı) profili düşürmüyor",
      h.isle(NO_DETECTION, 0.03 + _TO / 2.0), 5)

# `simdi` verilmezse zaman aşımı HİÇ işlemez: STOP filtresi gibi zamansız
# çağıranların davranışı değişmemeli.
k = ConsecutiveFrameFilter(2, bos_deger=NO_DETECTION, sticky=True,
                           sticky_zaman_asimi_s=_TO)
k.isle(5); k.isle(5)
check("simdi yoksa sınırsız sticky (eski davranış)",
      [k.isle(NO_DETECTION) for _ in range(200)][-1], 5)

# sticky=False olan STOP filtresi zaman aşımından etkilenmemeli.
m = ConsecutiveFrameFilter(2, bos_deger=False, sticky=False)
check("STOP filtresi: aday düşünce ANINDA boş",
      [m.isle(True, 0.0), m.isle(True, 0.03), m.isle(False, 0.06)][-1], False)

# Zaman aşımı değeri: kamera hızı değişse de kural değişmesin diye SÜRE
# cinsinden; kullanıcı kararı 3 saniye.
check("tabela görüş zaman aşımı 3 s", TABELA_GORUS_ZAMAN_ASIMI_S, 3.0)

# Zincirin öteki ucu: terrain_adapter boş değeri 'normal'e eşlemek zorunda,
# yoksa zaman aşımı dolar ama profil düşmez.
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'teknofest_ika/otonomi/terrain_adapter.py'),
          encoding='utf-8') as _f:
    _TA = _f.read()
check("terrain_adapter boş tespiti normal'e eşliyor",
      "NO_DETECTION: 'normal'" in _TA, True)

# Mantığı test etmek yetmiyor: DÜĞÜMÜN onu gerçekten kullandığı da
# denetlenmeli. İki bağlantı noktası var ve ikisi de sessizce koparılabilir —
# zaman argümanını düşürmek ya da sabiti None yapmak, testleri yeşil bırakıp
# zaman aşımını tamamen devre dışı bırakıyordu (mutasyonla bulundu).
# Denetim AST üzerinden: metin arama biçim değişince yanılır.
import ast as _ast_ya
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'teknofest_ika/gorsel/yolo_adapter_node.py'),
          encoding='utf-8') as _f:
    _YA = _ast_ya.parse(_f.read())

_isle_cagri = [
    n for n in _ast_ya.walk(_YA)
    if isinstance(n, _ast_ya.Call)
    and isinstance(n.func, _ast_ya.Attribute) and n.func.attr == 'isle'
    and isinstance(n.func.value, _ast_ya.Attribute)
    and n.func.value.attr == '_tabela_filter'
]
check("tabela filtresi tek yerden çağrılıyor", len(_isle_cagri), 1)
check("çağrıya ZAMAN da geçiriliyor",
      len(_isle_cagri[0].args) if _isle_cagri else 0, 2)

_kur = [
    n for n in _ast_ya.walk(_YA)
    if isinstance(n, _ast_ya.Call)
    and isinstance(n.func, _ast_ya.Name) and n.func.id == 'ConsecutiveFrameFilter'
    and any(k.arg == 'sticky_zaman_asimi_s' for k in n.keywords)
]
check("zaman aşımlı tek filtre kuruluyor", len(_kur), 1)
if _kur:
    _deger = [k.value for k in _kur[0].keywords if k.arg == 'sticky_zaman_asimi_s'][0]
    check("zaman aşımı None'a sabitlenmemiş",
          isinstance(_deger, _ast_ya.Constant) and _deger.value is None, False)
    check("zaman aşımı sabitten geliyor",
          isinstance(_deger, _ast_ya.Name)
          and _deger.id == 'TABELA_GORUS_ZAMAN_ASIMI_S', True)

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


# ─── §7.5 fren: sınır ötesi anlam kayması koruması ──────────────────────────
from teknofest_ika.otonomi.topics import FREN_GUVENLI_DUR_BINDE  # noqa: E402


# Kart fren değerini oranla alıyor ve SIFIR "freni bırak" demek: fren
# kaynakları arasında büyük olan seçildiği için sıfır hiçbir katkı yapmıyor.
# Güvenlik dalları bu yüzden sıfır yayınlayamaz — §6.10'un zorunlu duruşu
# %45 eğimde geçiyor ve orada fren isteğini kesmek aracı kaydırır.
check("güvenli duruş freni sıfır değil",  FREN_GUVENLI_DUR_BINDE > 0, True)
check("güvenli duruş freni tam fren",     FREN_GUVENLI_DUR_BINDE, 1000)

# Köprü fren komutunu kartın kabul ettiği aralığa kırpmalı: 0x08 işaretsiz ve
# negatif değeri kart sessizce sıfıra çeviriyor, yani kırpmayı görmeden
# göndermek "fren istedim ama bırakıldı" durumunu üretir.
with open(os.path.join(_KOK, 'teknofest_ika/gomulu/seri_kopru.py'),
          encoding='utf-8') as f:
    _SK3 = f.read()
check("fren komutu 0-1000'e kırpılıyor",
      'max(0, min(MAX_FREN_BINDE' in _SK3, True)
check("fren üst sınırı binde 1000",  'MAX_FREN_BINDE = 1000' in _SK3, True)


# ─── Aşama → durum eşlemesi: YÜKLEYİCİNİN ÇIKTISI ──────────────────────────
# 🔑 Bu test bir kez TERS yazılmıştı ve iki yönde birden yanılttı: waypoints.yaml'daki
# `type:` alanına bakıyordu, oysa o alan HİÇ OKUNMUYOR — load_waypoints eşlemeyi
# `isim`den türetiyor. Sonucu:
#   · dekoratif alanı silmek testi düşürüyordu       → sahte alarm
#   · gerçek türetmeyi bozmak testten geçiyordu      → sahte güvence
# İkincisi tam olarak korunmak istenen olaydı: ATIS_BOLGESI `shoot`u kaybederse
# atış durumuna hiç girilmez, hata basılmaz, 50 puan sessizce gider.
# O yüzden artık YÜKLEYİCİ ÇALIŞTIRILIYOR ve ürettiği tip okunuyor.
_WP_YOLU = os.path.join(_KOK, 'config', 'waypoints.yaml')
from teknofest_ika.otonomi.misyon_fsm import load_waypoints  # noqa: E402

_WP  = load_waypoints(_WP_YOLU)[0]
_TIP = {w['label']: w['type'] for w in _WP}

# ─── 12c. Düz başlangıç — LiDAR'sız ilk bacak ───────────────────────────────
print("\n=== 12c. Düz başlangıç (IMU + enkoder) ===")

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    duz_git_omega, duz_baslangic_tuketimi, duz_bacak_iptal,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    DUZ_BASLANGIC_MESAFE_M, DUZ_BASLANGIC_HIZ_MS, DUZ_BASLANGIC_KP,
    DUZ_DIREKSIYON_PAYI, DUZ_IMU_BAYATLAMA_S, PLANLAYICI_DONUS_YARICAPI_M,
)

_KL = DUZ_DIREKSIYON_PAYI / PLANLAYICI_DONUS_YARICAPI_M

# Sapma yoksa düzeltme de yok: küçük bir kalıcı ω düz şeritte 20 m boyunca
# birikir ve aracı duvara götürür.
check("sapma yok → ω sıfır", duz_git_omega(0.0, 0.0, 0.5, 1.0, _KL), 0.0)
# İŞARET: sola sapınca (yaw +) sağa düzeltilmeli (ω −). Ters olursa düzeltme
# hatayı büyütür ve araç ilk metrelerde şeritten çıkar.
check("sola sapma → sağa düzeltme",
      duz_git_omega(math.radians(2), 0.0, 0.5, 1.0, _KL) < 0.0, True)
check("sağa sapma → sola düzeltme",
      duz_git_omega(math.radians(-2), 0.0, 0.5, 1.0, _KL) > 0.0, True)
# Orantılılık: 2° hatası 1°'nin iki katı düzeltme ister (kelepçenin altında).
check("düzeltme hataya oranlı",
      round(duz_git_omega(math.radians(2), 0.0, 0.5, 1.0, _KL)
            / duz_git_omega(math.radians(1), 0.0, 0.5, 1.0, _KL), 3), 2.0)
# Kelepçe eğrilik cinsinden: büyük hatada ω = v × κ_limit'te durur.
check("büyük sapma kelepçede durur",
      round(duz_git_omega(math.radians(45), 0.0, 0.5, 1.0, _KL), 6),
      round(-0.5 * _KL, 6))
# Kelepçe HIZA göre ölçeklenir. Sabit bir ω tavanı yavaşken daha büyük bir
# direksiyon açısına karşılık gelir ve araç yılankavi sürer.
check("kelepçe hızla ölçekleniyor",
      round(duz_git_omega(math.radians(45), 0.0, 1.0, 1.0, _KL)
            / duz_git_omega(math.radians(45), 0.0, 0.5, 1.0, _KL), 3), 2.0)
# Duruyorken direksiyon kırmak aracı döndürmez, yalnız kalkışta baş açısını
# referanstan uzağa atar.
check("v≈0 → ω sıfır", duz_git_omega(math.radians(45), 0.0, 0.0, 1.0, _KL), 0.0)
# Açı sarması: -179° başlık ile +179° referans arasındaki fark 2°, 358° değil.
# Araç referanstan 2° SOLA dönmüş sayılır, düzeltme de 2° sapmayla aynı olmalı.
# Sarma olmazsa fark 358° okunur, düzeltme ters işaretli ve kelepçede çıkar.
check("açı sarması kısa yoldan",
      round(duz_git_omega(math.radians(-179), math.radians(179), 0.5, 1.0, _KL), 6),
      round(duz_git_omega(math.radians(2), 0.0, 0.5, 1.0, _KL), 6))
# Kelepçenin karşılığı teker açısı: tam kilit (30°) düz giden araçta salınım
# üretir, pay bunu çeyreğe indiriyor.
check("kelepçe ≈ 8° teker açısı",
      round(math.degrees(math.atan(_KL * 1.44)), 1), 8.2)

# ── Çizelge tüketimi ────────────────────────────────────────────────────────
# Kayan modda aşamalar KAT EDİLEN YOLA bakıyor: düz bacağı çizelgeye eklemek
# rampayı, atışı ve hızlanmayı bacak boyu kadar ileri kaydırır. Bu yüzden
# düşülüyor ve bu fonksiyon nereden devam edileceğini söylüyor.
_MES = [float(w.get('mesafe_m', 0.0)) for w in _WP]
check("tüketim yok → baştan, tam mesafe", duz_baslangic_tuketimi(_MES, 0.0),
      (0, _MES[0]))
check("ilk aşamanın yarısı", duz_baslangic_tuketimi(_MES, _MES[0] / 2.0),
      (0, _MES[0] / 2.0))
# Tam sınırda: ilk aşama bitti, ikincisi tam mesafesiyle başlar. Sınırı
# yanlış tarafa koymak bitmiş bir aşamayı 0 m mesafeyle sürdürür ve o aşama
# anında biter — parkurda bir istasyon sessizce atlanır.
_ITB = duz_baslangic_tuketimi(_MES, _MES[0])
check("ilk aşama tam bitti", (_ITB[0], round(_ITB[1], 6)),
      (1, round(_MES[1], 6)))
# 20 m gerçek çizelgede: aşama 1 ve 2 tamamen, 3'ün içinde kalan.
_I20, _K20 = duz_baslangic_tuketimi(_MES, 20.0)
check("20 m → 3. aşamadan devam", _I20, 2)
check("20 m → kalan doğru", round(_K20, 3),
      round(sum(_MES[:3]) - 20.0, 3))
check("20 m'nin bittiği aşama YAN_EGIM", _WP[_I20]['label'], 'YAN_EGIM')
# Düşülen + kalan = çizelgenin o noktasına kadarki toplam. Muhasebe bozulursa
# koşu ya erken ya geç biter ve sapma her aşamada birikir.
check("muhasebe kapanıyor", round(20.0 + _K20, 3), round(sum(_MES[:3]), 3))
# Çizelgeden uzun bacak: parkur bitti demek, sessizce son aşamayı sürmek değil.
check("çizelgeden uzun → aşama kalmadı",
      duz_baslangic_tuketimi(_MES, sum(_MES) + 5.0), (len(_MES), 0.0))
check("tam toplam → aşama kalmadı",
      duz_baslangic_tuketimi(_MES, sum(_MES)), (len(_MES), 0.0))
# Mesafesi olmayan aşama çizelgede yer tutmaz: "kalanı" da yoktur.
check("mesafesiz aşama atlanıyor", duz_baslangic_tuketimi([0.0, 5.0], 0.0),
      (1, 5.0))
# Negatif mesafe çizelgeyi GERİ SARMAMALI. Eksi bir sayı toplamı düşürürse
# sonraki aşama olduğundan yakın görünür: bacak onu yemiş sayılır ve parkurda
# bir istasyon sessizce atlanır. yaml'a elle girilen bir eksi işareti bunu
# tek başına yapar.
check("negatif mesafe geri sarmıyor",
      duz_baslangic_tuketimi([-5.0, 5.0], 0.0), (1, 5.0))
check("negatif mesafe araya girse de toplam korunur",
      duz_baslangic_tuketimi([10.0, -3.0, 5.0], 10.0), (2, 5.0))

# Varsayılan bacak düz şeridin İÇİNDE kalmalı: istasyon 3'e kümülatif mesafe
# kadar yol düz, sonrası viraj. Bacak oradan uzun olursa araç virajı LiDAR'sız
# ve baş açısını sabit tutarak girer, yani duvara sürer.
_DUZ_SERIT_M = sum(_MES[:3])
check("varsayılan bacak düz şeridin içinde",
      DUZ_BASLANGIC_MESAFE_M <= _DUZ_SERIT_M, True)
# İlk üç istasyon gerçekten düz mü — bacağın dayanağı bu. CAD'de üçü de aynı
# y'de ve aynı yönde; biri kayarsa sabit baş açısı şeridi takip etmeyi bırakır.
with open(_WP_YOLU, encoding='utf-8') as _f:
    _WP_HAM = yaml.safe_load(_f)
_CAD3 = [a.get('waypoint_cad', {}) for a in _WP_HAM['asamalar'][:3]]
check("ilk üç istasyon aynı y'de",
      all(abs(float(c.get('y', 99))) < 0.01 for c in _CAD3), True)
check("ilk üç istasyon aynı yönde",
      all(abs(abs(float(c.get('yaw', 0))) - math.pi) < 0.01 for c in _CAD3), True)
# İlk viraj bacağın DIŞINDA kalmalı.
check("4. istasyon virajda (bacağın dışında)",
      abs(float(_WP_HAM['asamalar'][3]['waypoint_cad']['y'])) > 1.0, True)

# ── İptal kapıları ──────────────────────────────────────────────────────────
# Bacağın iki girdisi var ve ikisi de susabiliyor. Kapılar eskiden execute()
# içindeydi ve kaldırılmaları hiçbir testi düşürmüyordu — mutasyon bunu
# yakaladı, karar buraya taşındı.
_KAPI = dict(odom_bayatlama_s=1.0, imu_bayatlama_s=0.5, ilerleme_s=4.0,
             ilerleme_m=0.20, sapma_sinir=math.radians(6.0))

check("sağlıklı durumda sürmeye devam",
      duz_bacak_iptal(0.1, 0.1, True, 5.0, 10.0, 0.0, **_KAPI), None)
# /odom bayatsa yol ölçülemiyor: bacak mesafeyi bilmeden sürerdi ve aşama
# çizelgesinden ne düşüleceği de bilinmezdi.
check("odom bayat → failed",
      duz_bacak_iptal(1.5, 0.1, True, 5.0, 10.0, 0.0, **_KAPI),
      ('odom_bayat', 'failed'))
# IMU bu bacağın TEK geri beslemesi. Tolere etmek "düz gittiğini varsayarak
# 20 m sür" demek; 1° sapma 0,35 m, payı 0,915 m.
check("IMU bayat → failed",
      duz_bacak_iptal(0.1, 0.9, True, 5.0, 10.0, 0.0, **_KAPI),
      ('imu_bayat', 'failed'))
check("yaw hiç gelmedi → failed",
      duz_bacak_iptal(0.1, 0.1, False, 5.0, 10.0, 0.0, **_KAPI),
      ('imu_bayat', 'failed'))
# Sınırda tolere edilir, aşınca değil: eşiği yanlış tarafa koymak 50 Hz'lik
# bir akışta her örnekte sahte iptal üretir.
check("IMU tam eşikte tolere",
      duz_bacak_iptal(0.1, 0.5, True, 5.0, 10.0, 0.0, **_KAPI), None)
check("odom tam eşikte tolere",
      duz_bacak_iptal(1.0, 0.1, True, 5.0, 10.0, 0.0, **_KAPI), None)
# Komut verildi ama araç kımıldamıyor.
check("ilerleme yok → failed",
      duz_bacak_iptal(0.1, 0.1, True, 0.05, 5.0, 0.0, **_KAPI),
      ('ilerleme_yok', 'failed'))
# Kalkış sürtünmesi ve kartın kalkış darbesi ilk saniyelerde yolu 0'da tutuyor;
# kapı o pencerede bakmamalı yoksa her koşu kalkışta iptal olur.
check("kalkış penceresinde ilerleme aranmıyor",
      duz_bacak_iptal(0.1, 0.1, True, 0.0, 2.0, 0.0, **_KAPI), None)
# Sapma sınırı aşılınca bacak BIRAKILIR, kurtarmaya gidilmez: araç sağlam,
# yalnız bu bacak işini yapamadı. failed demek kurtarma denemesi harcamaktır.
check("sapma aşıldı → bırak (kurtarma değil)",
      duz_bacak_iptal(0.1, 0.1, True, 5.0, 10.0, math.radians(7), **_KAPI),
      ('sapma_asildi', 'birak'))
check("sapma işaretten bağımsız",
      duz_bacak_iptal(0.1, 0.1, True, 5.0, 10.0, math.radians(-7), **_KAPI),
      ('sapma_asildi', 'birak'))
check("sapma sınır içinde tolere",
      duz_bacak_iptal(0.1, 0.1, True, 5.0, 10.0, math.radians(5.9), **_KAPI), None)
# Sensör kapıları sapma kapısından ÖNCE: bayat yaw ile hesaplanan sapma
# gerçeği değil son bilinen değeri ölçer, yani araç dönerken "sapma yok" der.
check("bayat sensör sapmadan önce gelir",
      duz_bacak_iptal(1.5, 0.1, True, 5.0, 10.0, math.radians(7), **_KAPI),
      ('odom_bayat', 'failed'))
check("bayat IMU sapmadan önce gelir",
      duz_bacak_iptal(0.1, 0.9, True, 5.0, 10.0, math.radians(7), **_KAPI),
      ('imu_bayat', 'failed'))
# Donmuş odometride yol hep 0'dır; onu "ilerleme yok" saymak yanlış teşhistir.
check("bayat odom ilerleme yokluğundan önce gelir",
      duz_bacak_iptal(1.5, 0.1, True, 0.0, 10.0, 0.0, **_KAPI),
      ('odom_bayat', 'failed'))

# Sebep kodlarının tamamının sahada okunacak bir metni olmalı: eşlemede
# olmayan bir kod log satırını KeyError ile düşürür ve bacak orada patlar.
_IPTAL_KODLARI = {'odom_bayat', 'imu_bayat', 'ilerleme_yok', 'sapma_asildi'}
with open(os.path.join(_KOK, 'teknofest_ika', 'otonomi', 'misyon_fsm.py'),
          encoding='utf-8') as _f:
    _FSM_DUZ = _f.read()
_M_METIN = re.search(r'_IPTAL_METNI = \{(.*?)\n    \}', _FSM_DUZ, flags=re.S)
check("iptal metni eşlemesi bulundu", _M_METIN is not None, True)
check("her sebebin metni var",
      set(re.findall(r"'(\w+)':", _M_METIN.group(1))) if _M_METIN else set(),
      _IPTAL_KODLARI)

# Hız sulu yola göre seçiliyor ve kartın etkili tabanının üstünde olmalı:
# altındaki komutlar kartta tabana YÜKSELTİLİYOR, yani yazılan sayı sahada
# geçerli olmaz.
check("düz bacak hızı kart tabanının üstünde", DUZ_BASLANGIC_HIZ_MS >= 0.20, True)
check("düz bacak hızı kart tavanının altında",
      DUZ_BASLANGIC_HIZ_MS <= KART_HIZ_TAVAN, True)
# IMU 50 Hz; bayatlama eşiği birkaç örneğe pay bırakmalı ama bacağın açık
# döngüye düşmesine izin verecek kadar uzun olmamalı.
check("IMU bayatlama eşiği makul",
      0.1 <= DUZ_IMU_BAYATLAMA_S <= 1.0, True)
check("kazanç pozitif", DUZ_BASLANGIC_KP > 0.0, True)
# Pay tam kilidi kullanmamalı: 1,0 düz giderken direksiyonu sonuna dayandırır.
check("direksiyon payı tam kilidin altında",
      0.0 < DUZ_DIREKSIYON_PAYI < 1.0, True)



# Nav2'yi baypas eden üç yol. Eşleme koparsa aşama sıradan bir navigasyon
# hedefine dönüşür ve bunu söyleyen hiçbir log yoktur.
check("ATIS_BOLGESI shoot durumuna gidiyor",       _TIP.get('ATIS_BOLGESI'), 'shoot')
check("HIZLANMA_PARKURU hizlanma durumuna gidiyor", _TIP.get('HIZLANMA_PARKURU'), 'hizlanma')
# Rampa baypası hiç test edilmemişti; ben de bu yüzden "RampaState ölü kod"
# sonucuna vardım. §6.10'da 2B tarama eğimi DUVAR okuduğu için baypas şart.
check("DIK_EGIM_GIRIS rampa durumuna gidiyor",     _TIP.get('DIK_EGIM_GIRIS'), 'rampa')
check("DIK_EGIM_CIKIS rampa durumuna gidiyor",     _TIP.get('DIK_EGIM_CIKIS'), 'rampa')

# Ters yön de kilitli: fazladan bir aşamanın baypasa düşmesi, o istasyonda
# Nav2'nin engel kaçınmasını sessizce kapatırdı.
check("yalnız dört aşama Nav2'yi baypas ediyor",
      sorted(a for a, t in _TIP.items() if t != 'nav'),
      ['ATIS_BOLGESI', 'DIK_EGIM_CIKIS', 'DIK_EGIM_GIRIS', 'HIZLANMA_PARKURU'])

# Üretilen her tipin FSM'de bir dalı olmalı. Yükleyiciye yeni bir tip eklenip
# dallandırma unutulursa aşama sessizce sıradan navigasyona düşer.
import ast as _ASTM  # noqa: E402

with open(os.path.join(_KOK, 'teknofest_ika', 'otonomi', 'misyon_fsm.py'),
          encoding='utf-8') as _f:
    _FSM_AGACI = _ASTM.parse(_f.read())
_DALLAR = set()
for _n in _ASTM.walk(_FSM_AGACI):
    if not isinstance(_n, _ASTM.Compare) or len(_n.comparators) != 1:
        continue
    _sol = _n.left
    if not (isinstance(_sol, _ASTM.Call)
            and getattr(_sol.func, 'attr', '') == 'get'
            and _sol.args and getattr(_sol.args[0], 'value', None) == 'type'):
        continue
    _sag = _n.comparators[0]
    if isinstance(_sag, _ASTM.Constant) and isinstance(_sag.value, str):
        _DALLAR.add(_sag.value)
check("FSM'de tip dalları bulundu", len(_DALLAR) >= 3, True)
check("üretilen her özel tipin FSM'de dalı var",
      sorted({t for t in _TIP.values() if t != 'nav'} - _DALLAR), [])

# Fiziksel sıra: atış rampanın ORTASINDA. Şartname §6.10 hedefi "minimum 10
# metre" uzağa koyuyor; rampa çıkışından hedefe 8,18 m var, yani atış orada
# yapılamaz. Liste sırası FSM'in sürüş sırasıdır.
_SIRA = [w['label'] for w in _WP]
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

    def _sabitler(dugum):
        """Geçiş hedefindeki dizge sabitleri.

        Hedef koşullu olabilir (`'A' if kapi else 'B'`): iki dal da sahada
        gerçekleşen bir hedeftir ve ikisi de bilinen bir duruma bağlanmalı.
        Yalnız `Constant` okumak koşulun bir dalını denetimsiz bırakırdı.
        """
        if isinstance(dugum, ast.Constant):
            return {dugum.value}
        if isinstance(dugum, ast.IfExp):
            return _sabitler(dugum.body) | _sabitler(dugum.orelse)
        return set()

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
                    hedef = set()
                    for v in kw.value.values:
                        hedef |= _sabitler(v)
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


# ─── Düz başlangıcın FSM'e bağlanması ───────────────────────────────────────
print("\n=== Düz başlangıç FSM'e bağlı mı ===")


def _sinif_dugumu(ad):
    import ast
    for d in ast.walk(ast.parse(_FSM)):
        if isinstance(d, ast.ClassDef) and d.name == ad:
            return d
    return None


_DUZ_SINIF = _sinif_dugumu('DuzBaslangicState')
check("DuzBaslangicState var", _DUZ_SINIF is not None, True)


def _abone_konulari(sinif):
    """Sınıfın create_subscription ile abone olduğu konu adları."""
    import ast
    adlar = set()
    for n in ast.walk(sinif):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'create_subscription' and len(n.args) >= 2):
            hedef = n.args[1]
            if isinstance(hedef, ast.Name):
                adlar.add(hedef.id)
            elif isinstance(hedef, ast.Constant):
                adlar.add(hedef.value)
    return adlar


if _DUZ_SINIF is not None:
    _DUZ_KONU = _abone_konulari(_DUZ_SINIF)
    # Bacağın varlık sebebi taramaya BAKMAMAK: tarama gelmiyorken ya da açısı
    # kalibre değilken Nav2 ve kayan hedef ikisi de çalışmıyor. Buraya bir
    # tarama aboneliği sızarsa bacak sessizce aynı bağımlılığı edinir.
    check("düz bacak taramaya abone DEĞİL",
          sorted(k for k in _DUZ_KONU if 'SCAN' in k or 'scan' in k), [])
    # İki girdisi zorunlu: yön IMU'dan, yol enkoder odometrisinden.
    check("düz bacak IMU'ya abone", 'IMU_TOPIC' in _DUZ_KONU, True)
    check("düz bacak odometriye abone", 'ODOM_TOPIC' in _DUZ_KONU, True)
    # Yol ölçümü KART odometrisinden gelmeli. /odometry/filtered EKF çıkışı ve
    # EKF yaw'ı aynı IMU'dan alıyor; ikisini aynı döngüde kullanmak düzeltmeyi
    # kendi etkisiyle besler.
    check("düz bacak EKF çıkışını kullanmıyor",
          'EKF_ODOM_TOPIC' in _DUZ_KONU, False)
    # Saf kontrolcü çağrılıyor mu — ω'yı state içinde yeniden hesaplamak
    # testsiz bir ikinci kopya demek.
    _DUZ_KAYNAK = _FSM[_FSM.index('class DuzBaslangicState'):]
    _DUZ_KAYNAK = _DUZ_KAYNAK[:_DUZ_KAYNAK.index('\nclass ')]
    check("ω saf fonksiyondan geliyor", 'duz_git_omega(' in _DUZ_KAYNAK, True)
    check("çizelge saf fonksiyonla düşülüyor",
          'duz_baslangic_tuketimi(' in _DUZ_KAYNAK, True)
    # Bacak yarıda kesilse de kat edilen yol çizelgeden düşülmeli: düşülmezse
    # normal sistem aşamaları o kadar geç bitirir ve sapma parkur boyu birikir.
    check("çıkışta çizelge her hâlde düşülüyor",
          _DUZ_KAYNAK.count('_cizelgeyi_dus(userdata'), 1)


def _durum_eklemeleri():
    """ad → (sınıf, geçiş hedefleri) — durum makinesine eklenen her durum."""
    import ast
    sonuc = {}
    for n in ast.walk(ast.parse(_FSM)):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'add' and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant)):
            sinif = (n.args[1].func.id if isinstance(n.args[1], ast.Call)
                     and isinstance(n.args[1].func, ast.Name) else None)
            hedef = {}
            for kw in n.keywords:
                if kw.arg == 'transitions' and isinstance(kw.value, ast.Dict):
                    for k, v in zip(kw.value.keys, kw.value.values):
                        hedef[k.value] = _sabit_kume(v)
            sonuc[n.args[0].value] = (sinif, hedef)
    return sonuc


def _sabit_kume(dugum):
    import ast
    if isinstance(dugum, ast.Constant):
        return {dugum.value}
    if isinstance(dugum, ast.IfExp):
        return _sabit_kume(dugum.body) | _sabit_kume(dugum.orelse)
    return set()


_EKLEME = _durum_eklemeleri()
check("DUZ_BASLANGIC durum makinesinde", 'DUZ_BASLANGIC' in _EKLEME, True)
check("DUZ_BASLANGIC doğru sınıfla eklenmiş",
      _EKLEME.get('DUZ_BASLANGIC', (None, {}))[0], 'DuzBaslangicState')
# IDLE'ın hedefi KOŞULLU: kapı açıkken düz bacak, kapalıyken eski akış. İki
# dalın ikisi de denetlenmeli — birini kaybetmek ya bacağı hiç çalıştırmamak
# ya da kapatılamaz hâle getirmek olurdu.
check("IDLE düz bacağa da doğrudan NAVIGATE'e de çıkabiliyor",
      _EKLEME.get('IDLE', (None, {}))[1].get('started'),
      {'DUZ_BASLANGIC', 'NAVIGATE'})
# Bacak bittiğinde normal sisteme dönülmeli; hata kurtarmaya, E-STOP iptale.
_DUZ_GECIS = _EKLEME.get('DUZ_BASLANGIC', (None, {}))[1]
check("düz bacak tamamlanınca NAVIGATE", _DUZ_GECIS.get('completed'), {'NAVIGATE'})
check("düz bacak hatada kurtarmaya", _DUZ_GECIS.get('failed'), {'ERROR_RECOVERY'})
check("düz bacak E-STOP'ta iptale", _DUZ_GECIS.get('e_stop'), {'MISSION_ABORT'})

# Parametre açılış betiğinden geçmeli: betik launch dosyası kullanmıyor, yani
# yalnız düğüm varsayılanında değiştirilen bir değer sahaya hiç ulaşmaz.
with open(os.path.join(_KOK, 'scripts', 'lydia_startup.sh'),
          encoding='utf-8') as _f:
    _BOOT_DUZ = _f.read()
_DUZ_VARSAYILAN = dict(re.findall(r'^:\s*"\$\{([A-Z0-9_]+):=([^}]*)\}"',
                                  _BOOT_DUZ, flags=re.M))
# Argümanlar bir DEĞİŞKENDE birikiyor, çağrı satırında literal durmuyorlar:
# dizge aramak `$_FSM_ARG`'ı görür ve içinde ne olduğunu söylemez. Bu yüzden
# satırlar çıkarılıp KOŞTURULUYOR ve düğüme gerçekte ne gittiği ölçülüyor.
_M_FSM = re.search(r'(\s*_FSM_ARG="-p duz_baslangic_m.*?2>&1 &\n)',
                   _BOOT_DUZ, flags=re.S)
check("misyon_fsm çağrı bloğu bulundu", _M_FSM is not None, True)

if _M_FSM:
    import subprocess as _sp
    import tempfile as _tf
    import textwrap as _tw

    def _fsm_argumanlari(mesafe, hiz, mod):
        with _tf.TemporaryDirectory() as _t:
            betik = (
                f'LOG={_t}\nDUZ_BASLANGIC_M={mesafe}\n'
                f'DUZ_BASLANGIC_HIZ={hiz}\nHEDEFLEME_MODU={mod}\n'
                f'IZ={_t}/iz\n'
                # Çağrılan satır çıktısını log dosyasına yönlendiriyor, yani
                # shim'in stdout'u yutulur; iz ayrı dosyaya yazılıyor.
                'ros2() { echo "$*" >> "$IZ"; }\n'
                + _tw.dedent(_M_FSM.group(1)) + 'wait\n')
            _sp.run(['bash', '-c', betik], capture_output=True, text=True)
            try:
                with open(os.path.join(_t, 'iz'), encoding='utf-8') as f:
                    return f.read()
            except FileNotFoundError:
                return ''

    _ARG = _fsm_argumanlari('20.0', '0.50', '')
    check("duz_baslangic_m düğüme geçiriliyor",
          '-p duz_baslangic_m:=20.0' in _ARG, True)
    check("duz_baslangic_hiz düğüme geçiriliyor",
          '-p duz_baslangic_hiz:=0.50' in _ARG, True)
    # Boş HEDEFLEME_MODU parametre olarak GEÇMEMELİ: boş dizgeye ayarlamak
    # waypoints.yaml'daki değerin geçerli olmasını engeller ve mod sessizce
    # geçersiz olur.
    check("boş hedefleme modu geçirilmiyor",
          'hedefleme_modu' in _ARG, False)
    _ARG_K = _fsm_argumanlari('20.0', '0.50', 'kayan')
    check("verilen hedefleme modu geçiriliyor",
          '-p hedefleme_modu:=kayan' in _ARG_K, True)
    check("mod verilince düz bacak yine geçiyor",
          '-p duz_baslangic_m:=20.0' in _ARG_K, True)
    # Kapatma yolu: 0 geçirilebilmeli, yoksa sahada bacağı kapatmanın yolu
    # betiği düzenlemekten geçer.
    check("bacak kapatılabiliyor",
          '-p duz_baslangic_m:=0' in _fsm_argumanlari('0', '0.50', ''), True)
check("misyon_fsm duz_baslangic_m tanımlıyor",
      "declare_parameter('duz_baslangic_m'" in _FSM, True)
check("misyon_fsm duz_baslangic_hiz tanımlıyor",
      "declare_parameter('duz_baslangic_hiz'" in _FSM, True)
# Betikteki varsayılan sabitle aynı olmalı; ayrışırlarsa sahada hangisinin
# geçerli olduğu çağrı sırasına kalır.
check("betik varsayılanı sabitle aynı",
      float(_DUZ_VARSAYILAN.get('DUZ_BASLANGIC_M', -1)), DUZ_BASLANGIC_MESAFE_M)
check("betik hız varsayılanı sabitle aynı",
      float(_DUZ_VARSAYILAN.get('DUZ_BASLANGIC_HIZ', -1)), DUZ_BASLANGIC_HIZ_MS)


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


# ─── Hedefleme modu: harita hedefi ↔ kayan hedef ────────────────────────────
from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    hedefleme_modu_sec, kayan_hedef_karari, quat_yaw, arac_hedefini_odoma_tasi,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    KAYAN_HEDEF_FRAME, KAYAN_HEDEF_PERIYOT_S, NAV2_GLOBAL_FRAME,
)

# Waypoint'ler doluyken 'oto' harita yolunu seçer, boşken kayan hedefe geçer.
# Bu ikinci hal bugünkü gerçek durum: waypoint alanları 11/11 (0,0).
check("oto + dolu waypoint → harita",     hedefleme_modu_sec('oto', True),  ('harita', 'oto'))
check("oto + boş waypoint → kayan",       hedefleme_modu_sec('oto', False), ('kayan', 'oto'))
check("açık istek harita",                hedefleme_modu_sec('harita', True), ('harita', 'istendi'))
check("açık istek kayan",                 hedefleme_modu_sec('kayan', False), ('kayan', 'istendi'))
# Açıkça 'harita' istenmiş ama waypoint'ler boşsa isteğe UYULUR — ancak
# gerekçe bunu söyler, çağıran yüksek sesle uyarabilsin diye. Sessizce kayan
# moda kaymak, aracın neden başka türlü sürdüğünü kimseye açıklamazdı.
check("harita ama waypointler boş",       hedefleme_modu_sec('harita', False),
      ('harita', 'harita_ama_waypointler_bos'))
# Yazım hatası çökertmemeli (koşu tamamen engellenirdi) ama sessizce de
# geçmemeli: 'oto' gibi davranıp gerekçede bildirir.
check("geçersiz mod oto gibi davranır",   hedefleme_modu_sec('kyan', False)[0], 'kayan')
check("geçersiz mod gerekçede görünür",   hedefleme_modu_sec('kyan', False)[1].startswith('gecersiz'), True)

# Aşama bitişi: kat edilen YOL ölçütü. Kuş uçuşu ölçüt U dönüşünde aşamayı
# yolun yarısında bitirirdi (22 m yol ↔ 11,3 m kuş uçuşu).
check("mesafe dolmadan devam",            kayan_hedef_karari(5.0, 22.0, 0, 8), 'devam')
check("mesafe dolunca tamam",             kayan_hedef_karari(22.0, 22.0, 0, 8), 'tamam')
check("mesafe aşılınca da tamam",         kayan_hedef_karari(23.5, 22.0, 0, 8), 'tamam')
# Varış ölçütü önce bakılır: istasyon sonunda koridor açılıp hedef üretilemese
# bile aşama bitmiştir, kurtarmaya düşmek boşuna deneme harcardı.
check("varış hedefsizlikten önce gelir",  kayan_hedef_karari(22.0, 22.0, 8, 8), 'tamam')
check("ardışık hedefsizlik başarısız",    kayan_hedef_karari(5.0, 22.0, 8, 8), 'hedef_yok')
check("sınırın altında hâlâ devam",       kayan_hedef_karari(5.0, 22.0, 7, 8), 'devam')
# mesafe_m yoksa (0.0) aşama asla 'tamam' demez — bütçesi dolana kadar sürer.
# misyon_fsm bu yüzden kayan modda mesafesiz aşamaları başlangıçta uyarıyor.
check("mesafesiz aşama bitmez",           kayan_hedef_karari(99.0, 0.0, 0, 8), 'devam')


# ─── waypoints.yaml: kayan hedef alanları ───────────────────────────────────
# Kayan modda aşamanın nerede bittiğini YALNIZ mesafe_m söyler. Alan düşerse
# aşama timeout'a kadar sürer ve koşu saatinden yer — hata basılmaz.
_ASAMALAR = _ASAMA
check("her aşamada mesafe_m var",         all(float(a.get('mesafe_m', 0)) > 0 for a in _ASAMALAR), True)
# CAD'den ölçülen yol uzunlukları; toplamı 111,9 m. Kuş uçuşu 83 m'lik
# parkurda U dönüşleri yolu 1,5-2 katına çıkarıyor.
check("mesafe toplamı CAD ile uyumlu",    round(sum(float(a['mesafe_m']) for a in _ASAMALAR), 1), 111.9)
# U dönüşleri: 3→4 sola dönerken iç duvar sağda, 6→7 sağa dönerken solda.
# Yanlış yan, aracı virajın DIŞ duvarına sürer.
_VIRAJLAR = {a['isim']: a.get('viraj') for a in _ASAMALAR if a.get('viraj')}
check("U dönüşü olan iki aşama",          len(_VIRAJLAR), 2)
check("DIK_ENGEL iç duvarı sağda",        _VIRAJLAR.get('DIK_ENGEL'), 'sag')
check("ENGEBELİ_ARAZİ iç duvarı solda",   _VIRAJLAR.get('ENGEBELİ_ARAZİ'), 'sol')
# viraj değeri ic_duvar_hedefi'nin kabul ettiği iki dizeden biri olmalı;
# başka bir şey yazılırsa fonksiyon hedef üretmez ve viraj hedefsiz kalır.
check("viraj değerleri geçerli",          set(_VIRAJLAR.values()) <= {'sol', 'sag'}, True)

# ─── §6.10 eğim ortası duruşu + yokuş kalkışı ───────────────────────────────
# Duruş noktası rampanın DİBİ DEĞİL, eğimin ORTASI (şartname: "rampa üzerindeki
# işaretli yerlerde"). Cezası aşamanın tüm puanı, o yüzden sayı CAD geometrisine
# kilitleniyor: "Dik egim" 8,602 × 1,850 m, %45 eğimde yamaç boyu 4,51 m.
from teknofest_ika.otonomi.pure_logic import (          # noqa: E402
    rampa_ara_durus_gerekli, yokus_kalkis_freni,
)
from teknofest_ika.otonomi.topics import (              # noqa: E402
    YOKUS_TUTMA_FREN_BINDE, YOKUS_TORK_SURESI_S,
    YOKUS_FREN_BIRAKMA_BINDE_PER_S, FREN_GUVENLI_DUR_BINDE,
)

_RAMPA_X, _RAMPA_H = 8.602, 1.850          # CAD sınır kutusu
_YAMAC_KOSU = _RAMPA_H / 0.45              # §6.10 %45 eğim
_YAMAC_BOYU = math.hypot(_YAMAC_KOSU, _RAMPA_H)
check("CAD yamaç koşusu 4,11 m",          round(_YAMAC_KOSU, 2), 4.11)
check("CAD yamaç boyu 4,51 m",            round(_YAMAC_BOYU, 2), 4.51)
# Tepe platosu araçtan (1,90 m) kısa: araç zirveyi köprüler ve pitch ~0 okur.
# Atış noktasının "düz zemin" gibi davranmasının sebebi bu.
check("tepe platosu araçtan kısa",        (_RAMPA_X - 2 * _YAMAC_KOSU) < 1.90, True)

_DURUSLU = [a for a in _ASAMALAR if float(a.get('durus_mesafe_m', 0) or 0) > 0]
check("iki rampa aşamasında duruş var",   len(_DURUSLU), 2)
check("duruşlu aşamalar DIK_EGIM",        sorted(a['isim'] for a in _DURUSLU),
      ['DIK_EGIM_CIKIS', 'DIK_EGIM_GIRIS'])
# Duruş yamacın yarısına yakın olmalı; yarıdan büyükse araç ortayı geçer,
# çok küçükse eğime daha yeni girmişken durur.
for _a in _DURUSLU:
    _d = float(_a['durus_mesafe_m'])
    check(f"{_a['isim']} duruşu yamaç ortasına yakın",
          0.6 * (_YAMAC_BOYU / 2) < _d < _YAMAC_BOYU / 2, True)
    check(f"{_a['isim']} duruşu eğim içinde", _d < _YAMAC_BOYU, True)

# 🔑 ASIL REGRESYON: yükleyici anahtarları BEYAZ LİSTELİYOR. durus_mesafe_m
# listeye eklenmezse RampaState wp.get() ile hep 0.0 okur ve duruş SESSİZCE
# hiç tetiklenmez — aynı sınıf hata `type` alanında bir kez yaşandı.
_FSM_KAYNAK = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'teknofest_ika', 'otonomi', 'misyon_fsm.py'),
                   encoding='utf-8').read()
_YUKLEYICI = _FSM_KAYNAK[_FSM_KAYNAK.index('waypoints.append({'):]
_YUKLEYICI = _YUKLEYICI[:_YUKLEYICI.index('})')]
check("yükleyici durus_mesafe_m'i geçiriyor", "'durus_mesafe_m'" in _YUKLEYICI, True)
check("yükleyici mesafe_m'i geçiriyor",       "'mesafe_m'" in _YUKLEYICI, True)

# Ara duruş kararı
check("ortada duruş tetiklenir",          rampa_ara_durus_gerekli('egimde', 2.10, 2.05, False), True)
check("eşiğin altında tetiklenmez",       rampa_ara_durus_gerekli('egimde', 1.90, 2.05, False), False)
check("bir kez tetiklenir",               rampa_ara_durus_gerekli('egimde', 3.00, 2.05, True), False)
check("yaklaşmada tetiklenmez",           rampa_ara_durus_gerekli('yaklasma', 9.0, 2.05, False), False)
check("0 mesafe duruşu kapatır",          rampa_ara_durus_gerekli('egimde', 9.0, 0.0, False), False)

# Yokuş kalkışı: ÖNCE gaz (fren tam basılı), SONRA fren rampası. Sıra tersine
# dönerse fren bırakıldığı an tork yoktur, araç geri kaçar ve sürücü ters dönen
# rotoru sürmeyi reddeder — kaçınılan arıza tam olarak budur.
_TB, _TS = YOKUS_TUTMA_FREN_BINDE, YOKUS_TORK_SURESI_S
check("tork penceresinde fren TAM",       yokus_kalkis_freni(0.0, _TB, _TS, 2000.0), (_TB, False))
check("tork penceresi sonuna kadar tam",  yokus_kalkis_freni(_TS * 0.99, _TB, _TS, 2000.0)[0], _TB)
check("tork sonrası fren düşüyor",        yokus_kalkis_freni(_TS + 0.2, _TB, _TS, 2000.0)[0] < _TB, True)
check("fren sıfırlanınca biter",          yokus_kalkis_freni(_TS + 1.0, _TB, _TS, 2000.0), (0, True))
check("fren monoton azalıyor",
      yokus_kalkis_freni(_TS + 0.10, _TB, _TS, 2000.0)[0] >
      yokus_kalkis_freni(_TS + 0.20, _TB, _TS, 2000.0)[0], True)
# Tutma freni tam güç olmalı: eğimde yarım fren aracı tutmaz.
check("tutma freni tam güç",              _TB, FREN_GUVENLI_DUR_BINDE)
check("bırakma hızı pozitif",             YOKUS_FREN_BIRAKMA_BINDE_PER_S > 0, True)
# Tork penceresi motoru frene karşı zorluyor: uzun tutmak akım/ısı demek.
check("tork penceresi kısa",              0.0 < _TS <= 1.0, True)

# ─── Yokuş kalkışı override'ı BAYATLARSA bırakılmalı ────────────────────────
# RampaState süreç olarak ölürse `finally` çalışmaz. Bayrak tek sefer basılıp
# güvenilirse ackermann_converter sonsuza kadar override'da kalır: otomatik
# fren tamamen ölür ve fren son değerinde donar. Koruma iki parçalı ve İKİSİ
# BİRDEN gerekli — bayrağın nabız gibi basılması + karşı tarafta bayatlık
# kontrolü. Biri düşerse diğeri anlamsız, hatta zararlı olur.
from teknofest_ika.otonomi.topics import YOKUS_BAYATLAMA_S   # noqa: E402

_AC_KAYNAK = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'teknofest_ika', 'otonomi',
                               'ackermann_converter.py'), encoding='utf-8').read()
# Sabitin import edilmiş olması yetmez — KULLANILDIĞI da görülmeli. İlk
# sürümde bu kontrol yalnız dizgenin varlığına bakıyordu ve mutasyon testinde
# yakalanmadı: blok silinse bile import satırı dizgeyi hayatta tutuyordu.
_AC_GOVDE = _AC_KAYNAK[_AC_KAYNAK.index('def _cmd_vel_callback'):]
check("bayatlık kontrolü callback İÇİNDE",
      'YOKUS_BAYATLAMA_S' in _AC_GOVDE, True)
check("bayatlık karşılaştırması yapılıyor",
      re.search(r'>\s*YOKUS_BAYATLAMA_S', _AC_GOVDE) is not None, True)
check("bayrak zamanı callback'te okunuyor", '_yokus_zaman' in _AC_GOVDE, True)
check("bayrak zamanı callback'te yazılıyor",
      '_yokus_zaman = self.get_clock().now()' in _AC_KAYNAK, True)

# Nabız periyodu: RampaState.CMD_HZ. Bayatlama ondan BÜYÜK olmalı, yoksa
# override daha kalkış sürerken bayat sayılır ve fren ortada bırakılır.
_CMD_HZ = float(re.search(r'class RampaState.*?CMD_HZ\s*=\s*([\d.]+)',
                          _FSM_KAYNAK, re.S).group(1))
check("RampaState nabız frekansı okundu",  _CMD_HZ > 0, True)
check("bayatlama nabız periyodundan büyük",
      YOKUS_BAYATLAMA_S > (1.0 / _CMD_HZ), True)
check("bayatlama en az 3 nabız payı bırakıyor",
      YOKUS_BAYATLAMA_S >= 3.0 / _CMD_HZ, True)

# 🔑 ASIL REGRESYON: bayrak DÖNGÜNÜN İÇİNDE basılmalı. Döngü dışına alınırsa
# tek mesaj gider, bayatlık koruması kalkışın ortasında override'ı bırakır ve
# fren tam basılıyken gaz veren pencere çöker.
_YK = _FSM_KAYNAK[_FSM_KAYNAK.index('def _yokus_kalkis'):]
_YK = _YK[:_YK.index('\n    def ', 10)]
_DONGU = _YK[_YK.index('while rclpy.ok():'):]
check("yokuş bayrağı döngü İÇİNDE basılıyor",
      '_yokus_aktif_pub.publish' in _DONGU, True)
check("yokuş bayrağı döngü öncesi basılmıyor",
      '_yokus_aktif_pub.publish' in _YK[:_YK.index('while rclpy.ok():')], False)
# Bayrak her hâlükârda düşürülmeli — finally dalı da yerinde dursun.
check("kalkışta finally ile bayrak düşürülüyor",
      'finally:' in _YK and _YK.count('_yokus_aktif_pub.publish') >= 2, True)

# waypoints.yaml'daki varsayılan mod da geçerli bir değer olmalı.
_MOD = _WPY['parametreler'].get('hedefleme_modu')
check("varsayılan mod geçerli",           _MOD in ('harita', 'kayan', 'oto'), True)
# `harita` yalnız CAD→map dönüşümü ölçülmüşken meşru. Dönüşüm boşken bu modu
# istemek, koşuyu parkuru hiç sürmeden bitirmenin sessiz yoludur; kural
# kendi kendini bakar, dönüşüm doldurulduğu gün harita yeniden serbest kalır.
_DONUSUM = _WPY.get('parkur_cad', {}).get('donusum', {})
_DONUSUM_OLCULDU = all(_DONUSUM.get(k) is not None for k in ('dx', 'dy', 'dyaw'))
check("dönüşüm ölçülmeden harita modu seçilmez",
      _MOD == 'harita' and not _DONUSUM_OLCULDU, False)


# ─── misyon_fsm: kayan hedef bağlantısı ─────────────────────────────────────
# Kayan sürüş 'e_stop' döndürebiliyor; NavigateState bunu doğrudan iletmezse
# E-STOP kurtarmaya düşer ve araç saniyelerce oyalanır (bir kez böyle oldu).
check("kayan e_stop doğrudan iletiliyor", "if result == 'e_stop':" in _fsm_kaynak(), True)
# Aşama sonunda etkin hedef iptal edilmezse Nav2 son kayan hedefe (2-8 m
# ileri) sürmeye devam eder ve araç istasyonu geçer.
check("sürüş sonunda hedef iptal edilir", 'self.nav.iptal()' in _fsm_kaynak(), True)
# Kayan sürüş pozu EKF'ten okumalı, ham /odom'dan DEĞİL. Arka aks tek parça
# olduğu için seri_kopru tek enkoderle koşuyor ve d_theta = 0 bırakıyor: ham
# /odom'un yaw'ı hep sıfırdır. Hedefi o yaw ile odom'a taşımak, araç ilk
# dönüşten sonra hedefleri sabit bir yöne koyardı — ve bu hiçbir hata
# basmadan, yalnız sahada görülürdü.
def _kayan_odom_kaynagi():
    """KayanHedefSurucusu'nun Odometry aboneliğindeki topic sabitinin adı."""
    import ast
    agac = ast.parse(_fsm_kaynak())
    for d in ast.walk(agac):
        if isinstance(d, ast.ClassDef) and d.name == 'KayanHedefSurucusu':
            for c in ast.walk(d):
                if (isinstance(c, ast.Call)
                        and getattr(c.func, 'attr', '') == 'create_subscription'
                        and getattr(c.args[0], 'id', '') == 'Odometry'):
                    return getattr(c.args[1], 'id', None)
    return None


check("kayan sürüş pozu EKF'ten",         _kayan_odom_kaynagi(), 'EKF_ODOM_TOPIC')

# Hedef gönderilmeden önce odom'a taşınmalı; taşıma atlanırsa araç
# çerçevesindeki sayı odom sayısı sanılır ve hedef parkurun dışına düşer.
check("hedef odoma taşınarak gönderilir", 'arac_hedefini_odoma_tasi(hedef, poz)' in _fsm_kaynak(), True)
# Hedef odom çerçevesinde gönderilir. 'map' olsaydı SLAM düzeltmesi hedefi
# sıçratırdı; araç çerçevesi ('base_link') olsaydı Nav2 hedefi her yeniden
# planlamada o anki poza göre çözer, hedef araçla birlikte kayar ve asla
# varılmaz — ikisi de kayan hedefin anlamını bozar.
check("kayan hedef odom çerçevesinde",    KAYAN_HEDEF_FRAME, 'odom')

# Araç çerçevesindeki hedefin odom'a taşınması. Araç (5,5)'te +y'ye bakarken
# 2 m ilerisi (5,7) olmalı — eksen karışırsa hedef 90° yanlış yere düşer.
_TASINAN = arac_hedefini_odoma_tasi((2.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2))
check("hedef odoma taşınır — x",          round(_TASINAN[0], 3), 5.0)
check("hedef odoma taşınır — y",          round(_TASINAN[1], 3), 7.0)
check("hedef yönü poza eklenir",          round(_TASINAN[2], 4), round(math.pi / 2, 4))
# Araç orijinde ve hizalıysa dönüşüm kimlik olmalı.
check("orijinde dönüşüm kimlik",          arac_hedefini_odoma_tasi((3.0, 1.0, 0.5), (0.0, 0.0, 0.0)),
      (3.0, 1.0, 0.5))
check("quat_yaw 90 derece",               round(quat_yaw(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)), 4),
      round(math.pi / 2, 4))
check("quat_yaw birim kuaterniyon",       quat_yaw(0, 0, 0, 1), 0.0)
# Yeniden hedefleme periyodu Nav2'nin planlama süresinden kısa olmamalı:
# her hedef öncekini preempt ediyor, çok sık gönderilirse planlayıcı hiçbir
# planı bitiremez.
check("yeniden hedefleme çok sık değil",  KAYAN_HEDEF_PERIYOT_S >= 1.0, True)


# ─── LiDAR montaj açısı: taramayı indeksleyen her düğüm dönüşü uygulamalı ───
from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    tarama_acisi_arac, arac_acisi_tarama, aci_pencerede, tarama_kirpma_penceresi,
)
from teknofest_ika.otonomi.topics import LIDAR_MONTAJ_YAW_RAD  # noqa: E402

_Y = LIDAR_MONTAJ_YAW_RAD


def _derece(r):
    return round(math.degrees(r), 1)


# Sabit urdf lidar_joint ile aynı sayı olmak zorunda: TF bir açıyı, tarama
# indeksleme başka bir açıyı kullanırsa engeller iki ayrı yere düşer.
def _urdf_lidar_yaw():
    """lidar_joint'in yaw'ı. XML AYRIŞTIRILARAK okunuyor: metin deseni,
    joint'in içine bir yorum eklendiğinde sessizce None dönüyordu."""
    import xml.etree.ElementTree as _ET
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'arac.urdf')
    for j in _ET.parse(yol).getroot().iter('joint'):
        if j.get('name') == 'lidar_joint':
            return float(j.find('origin').get('rpy').split()[2])
    return None


check("montaj açısı urdf ile aynı",       _urdf_lidar_yaw(), LIDAR_MONTAJ_YAW_RAD)
check("ileri araç açısı → tarama",        _derece(arac_acisi_tarama(0.0, _Y)), -93.3)
check("sağ araç açısı → tarama",          _derece(arac_acisi_tarama(-math.pi / 2, _Y)), 176.7)
check("dönüşüm gidiş-dönüş",              _derece(tarama_acisi_arac(arac_acisi_tarama(-1.0, _Y), _Y)),
      _derece(-1.0))
# Sarmalı pencere: montaj dönüşünden sonra pencere ±180°'yi aşabiliyor; düz
# karşılaştırma o pencereyi ikiye böler ve arada kalan huzmeleri sessizce atar.
check("sarmalı pencere içi",              aci_pencerede(math.radians(179), math.radians(170), math.radians(-170)), True)
check("sarmalı pencere dışı",             aci_pencerede(0.0, math.radians(170), math.radians(-170)), False)
check("düz pencere içi",                  aci_pencerede(0.0, math.radians(-135), math.radians(135)), True)

# ASIL REGRESYON: ±135° kırpma penceresi ARAÇ çerçevesinde uygulanmalı.
# Tarama açısına doğrudan uygulandığında aracın sağ yanı (−131,7°…−41,7°)
# komple inf oluyordu; bunu /scan/filtered'ı okuyan herkes miras alıyordu
# (Nav2'nin iki costmap'i, kayar engel, koni füzyonu, §6.9 koridor ölçümü).
def _kirpma_hayatta_kalir(arac_deg, alt_deg=-135.0, ust_deg=135.0, yaw=_Y):
    """Verilen araç yönündeki huzme kırpmadan geçiyor mu.

    preprocessing_node ile AYNI yolu izler: pencere bir kez tarama çerçevesine
    taşınır (tarama_kirpma_penceresi), sonra huzmenin tarama açısı o pencereye
    sokulur. `yaw=0` verilirse düzeltmenin olmadığı eski davranış çıkar.
    """
    alt, ust = tarama_kirpma_penceresi(math.radians(alt_deg), math.radians(ust_deg), yaw)
    tarama   = arac_acisi_tarama(math.radians(arac_deg), _Y)   # huzme gerçekte nerede
    if alt > ust:                       # pencere ±180°'yi aşıyor
        return tarama >= alt or tarama <= ust
    return alt <= tarama <= ust


check("sağ yan kırpmadan geçer",          _kirpma_hayatta_kalir(-90), True)
check("sağ-ön kırpmadan geçer",           _kirpma_hayatta_kalir(-45), True)
check("sol yan kırpmadan geçer",          _kirpma_hayatta_kalir(90), True)
check("ileri kırpmadan geçer",            _kirpma_hayatta_kalir(0), True)
# Pencere hâlâ ARKAYI kesiyor — kırpmanın var olma sebebi bu, tamamen açmak
# düzeltme değil, kuralın iptali olurdu.
check("arka hâlâ kesiliyor",              _kirpma_hayatta_kalir(180), False)
# Düzeltme geri alınırsa (montaj açısı uygulanmazsa) sağ yan yeniden körleşir —
# hatanın kendisi burada kilitleniyor, yalnız doğru davranış değil.
check("düzeltmesiz sağ yan körleşir",     _kirpma_hayatta_kalir(-90, yaw=0.0), False)
check("düzeltmesiz arka açılır",          _kirpma_hayatta_kalir(180, yaw=0.0), True)


def _kaynak(yol):
    tam = os.path.join(os.path.dirname(os.path.abspath(__file__)), yol)
    with open(tam, encoding='utf-8') as f:
        return f.read()


# Taramayı dizi olarak indeksleyen dört düğüm de dönüşü uygulamak zorunda.
# TF yalnız costmap'e YERLEŞTİRMEYİ düzeltir; dizi indeksini düzeltmez.
for _ad, _yol in [
    ('preprocessing', 'teknofest_ika/gorsel/preprocessing_node.py'),
    ('kayar engel kalman', 'teknofest_ika/gorsel/kayar_engel_kalman.py'),
    ('kayar engel costmap', 'teknofest_ika/gorsel/kayar_engel_costmap.py'),
    ('koni füzyonu', 'teknofest_ika/gorsel/cone_fusion_node.py'),
]:
    check(f"{_ad} montaj açısını uygular", 'LIDAR_MONTAJ_YAW_RAD' in _kaynak(_yol), True)

check("preprocessing pencereyi çevirir",
      'tarama_kirpma_penceresi(' in _kaynak('teknofest_ika/gorsel/preprocessing_node.py'), True)

# Açı sabiti tek yerde: elle yazılmış 1.6284 kalmamalı (iki kopya ayrışırsa
# hangisinin geçerli olduğu düğüm sırasına kalır).
check("misyon_fsm sabiti elle yazmaz",    '1.6284' in _fsm_kaynak(), False)


# ─── Otonom komut bayatlığı: mux son Twist'i tekrarlamamalı ─────────────────
# mod_yoneticisi FULL_AUTO'da /cmd_vel'i 20 Hz ile mux'a geçiriyor. Zaman
# damgası tutulmazsa yayıncı sıfırdan farklı bir komutla susunca araç o hızda
# gitmeye devam eder — ve aşağı akıştaki watchdog'lar bunu YAKALAYAMAZ:
# ackermann_converter /mux/cmd_vel'i dinliyor, mux taze mesaj ürettiği için
# timeout'u hiç dolmuyor.
from teknofest_ika.otonomi.topics import NAV2_CMD_BAYATLAMA_S  # noqa: E402

_MOD_KAYNAK = _kaynak('teknofest_ika/otonomi/mod_yoneticisi.py')
check("mux komut zamanı tutuluyor",
      'self._nav2_son   = time.monotonic()' in _MOD_KAYNAK, True)
check("mux bayat komutu sıfırlar",        'nav2_gecmis > NAV2_CMD_BAYATLAMA_S' in _MOD_KAYNAK, True)
check("bayatlık sınırı ackermann ile aynı", NAV2_CMD_BAYATLAMA_S, 0.5)


# ─── Watchdog: kapalı alt sistem "arıza" sayılmamalı ────────────────────────
# Sabit listede /scan_lidar, EKF ve yolo_adapter vardı; boot betiğinde
# üçünün de yayıncısı yok (ham tarama /scan'e gidiyor, diğer ikisi Nav2
# bloğunda). Sonuç her açılışta üç KALICI sahte arıza — sahte alarm,
# kontrolün kendisini değersizleştirdiği için gerçek arızayı kaçırmakla
# aynı sonucu verir.
# watchdog.py doğrudan import EDİLEMEZ (vision_msgs gibi ROS paketlerine
# bağlı), sözlükler kaynaktan AST ile okunuyor.
def _watchdog_sozlugu(ad):
    import ast as _ast
    agac = _ast.parse(_kaynak('teknofest_ika/otonomi/watchdog.py'))
    for d in agac.body:
        if (isinstance(d, _ast.Assign) and d.targets
                and getattr(d.targets[0], 'id', '') == ad):
            return [getattr(k, 'id', getattr(k, 'value', None)) for k in d.value.keys]
    return None


_TEMEL = _watchdog_sozlugu('TEMEL_TOPICLER')
_NAV2  = _watchdog_sozlugu('NAV2_TOPICLERI')

check("EKF koşullu izleniyor",            'EKF_ODOM_TOPIC' in _NAV2 and 'EKF_ODOM_TOPIC' not in _TEMEL, True)
check("yolo_adapter koşullu izleniyor",   'DETECTIONS_TOPIC' in _NAV2 and 'DETECTIONS_TOPIC' not in _TEMEL, True)
check("ham tarama sabit değil",           'SCAN_LIDAR_TOPIC' not in _TEMEL, True)
check("E-STOP her zaman izleniyor",       'E_STOP_TOPIC' in _TEMEL, True)
check("ham tarama parametreli",           "declare_parameter('ham_tarama_topic'" in _kaynak('teknofest_ika/otonomi/watchdog.py'), True)
# Boot betiği sahada koşan yol: watchdog'a doğru ham tarama adını ve Nav2
# durumunu geçirmeli, yoksa düzeltme betikte geçersiz kalır.
_BOOT = _kaynak('scripts/lydia_startup.sh')
check("boot betiği ham taramayı geçirir", 'ham_tarama_topic:=/scan' in _BOOT, True)
check("boot betiği nav2 durumunu geçirir", 'nav2_aktif:=' in _BOOT, True)


# ─── RPP eğrilik kısması ölü kural olmamalı ─────────────────────────────────
# Kısma eşiği aracın çizebildiği en dar yaydan (R_min) BÜYÜK olmalı; küçükse
# koşul hiç sağlanmaz ve kural sessizce hiçbir şey yapmaz.
def _nav2_params():
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'config', 'nav2_params.yaml')
    with open(yol, encoding='utf-8') as f:
        return yaml.safe_load(f)


_FP = _nav2_params()['controller_server']['ros__parameters']['FollowPath']
_R_MIN = 1.44 / math.tan(0.5236)          # L / tan(δ_max) — ackermann_converter ile aynı
check("kısma eşiği R_min üstünde",        _FP['regulated_linear_scaling_min_radius'] > _R_MIN, True)
# Kısma sonrası hız kalkış sürtünmesi tabanının altına düşmemeli, yoksa araç
# viraj ortasında yerinden kalkamaz.
check("kısılan hız tabanın üstünde",
      round(0.65 * (_R_MIN / _FP['regulated_linear_scaling_min_radius']), 2) >= _FP['regulated_linear_scaling_min_speed'], True)


# ─── Nav2 davranış ağacı: kullanılan her düğüm KAYITLI olmalı ───────────────
# Bu kontrol, sahada Nav2'nin tamamını düşüren hatayı yakalamak için var.
# Stok ağaç kurtarma dalında <Spin> kullanıyor; Ackermann araç yerinde
# dönemediği için nav2_spin_action_bt_node bilerek yüklenmiyordu. Tanınmayan
# düğüm = ağaç ayrıştırılamaz ve bt_navigator aktive OLAMAZ:
#   [bt_navigator] Node not recognized: Spin
#   [lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
# (Jetson launch.log 2026-07-19 19:56 — enkoder takılı olsa bile Nav2 bu
# konfigürasyonla ayağa kalkamazdı.)
import xml.etree.ElementTree as _ET  # noqa: E402

# BehaviorTree.CPP çekirdeği — eklenti gerektirmez.
_BT_DAHILI = {
    'root', 'BehaviorTree', 'Sequence', 'SequenceStar', 'ReactiveSequence',
    'Fallback', 'ReactiveFallback', 'Inverter', 'ForceSuccess', 'ForceFailure',
    'Repeat', 'RetryUntilSuccessful', 'Parallel', 'BlackboardCheckInt',
}

# Düğüm adı → nav2_params.yaml plugin_lib_names girdisi. Eşleme düzensiz
# (BackUp → back_up), o yüzden tahmin edilmiyor, elle yazılıyor. Ağaca yeni
# bir düğüm eklenirse burası da güncellenmeli — test o zaman "bilinmeyen
# düğüm" diye durur, sessizce geçmez.
_BT_EKLENTI = {
    'ComputePathToPose':        'nav2_compute_path_to_pose_action_bt_node',
    'ComputePathThroughPoses':  'nav2_compute_path_through_poses_action_bt_node',
    'FollowPath':               'nav2_follow_path_action_bt_node',
    'SmoothPath':               'nav2_smooth_path_action_bt_node',
    'ClearEntireCostmap':       'nav2_clear_costmap_service_bt_node',
    'GoalUpdated':              'nav2_goal_updated_condition_bt_node',
    'GoalReached':              'nav2_goal_reached_condition_bt_node',
    'IsStuck':                  'nav2_is_stuck_condition_bt_node',
    'BackUp':                   'nav2_back_up_action_bt_node',
    'DriveOnHeading':           'nav2_drive_on_heading_bt_node',
    'Wait':                     'nav2_wait_action_bt_node',
    'Spin':                     'nav2_spin_action_bt_node',
    'RateController':           'nav2_rate_controller_bt_node',
    'DistanceController':       'nav2_distance_controller_bt_node',
    'SpeedController':          'nav2_speed_controller_bt_node',
    'RecoveryNode':             'nav2_recovery_node_bt_node',
    'PipelineSequence':         'nav2_pipeline_sequence_bt_node',
    'RoundRobin':               'nav2_round_robin_node_bt_node',
    'RemovePassedGoals':        'nav2_remove_passed_goals_action_bt_node',
    'TruncatePath':             'nav2_truncate_path_action_bt_node',
}

_BT_PARAM  = _nav2_params()['bt_navigator']['ros__parameters']
_EKLENTILER = set(_BT_PARAM['plugin_lib_names'])
_KOK = os.path.dirname(os.path.abspath(__file__))


def _bt_dosyasi(anahtar):
    """nav2_params'taki yolu repo içindeki gerçek dosyaya çevirir."""
    yol = _BT_PARAM[anahtar]
    # Yol Jetson'ın çalışma alanına mutlak; repoda karşılığı config/bt altında.
    return os.path.join(_KOK, 'config', 'bt', os.path.basename(yol)), yol


def _bt_dugumleri(dosya):
    return {e.tag for e in _ET.parse(dosya).iter()}


for _anahtar in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
    _dosya, _yol = _bt_dosyasi(_anahtar)
    _ad = os.path.basename(_dosya)
    check(f"{_ad} repoda var",            os.path.exists(_dosya), True)
    # Yol MUTLAK olmalı: çıplak dosya adı verilirse bt_navigator onu çalışma
    # dizinine göre açmaya çalışır ve bulamaz.
    check(f"{_ad} yolu mutlak",           os.path.isabs(_yol), True)

    _dugumler = _bt_dugumleri(_dosya) if os.path.exists(_dosya) else set()
    # ASIL KONTROL: her düğümün ya dahili olması ya da eklentisinin YÜKLÜ olması.
    _kayitsiz = sorted(
        d for d in _dugumler
        if d not in _BT_DAHILI and _BT_EKLENTI.get(d) not in _EKLENTILER
    )
    check(f"{_ad} kayıtsız düğüm yok",    _kayitsiz, [])
    # Spin özel olarak yasak: eklentisi yüklense bile Ackermann araç yerinde
    # dönemez, komut verilirse titreyip timeout'a girer.
    check(f"{_ad} Spin içermiyor",        'Spin' in _dugumler, False)
    check(f"{_ad} BackUp içeriyor",       'BackUp' in _dugumler, True)

# Spin eklentisi listeye geri eklenmemeli — ağaç yüklenir ama araç yerinde
# dönmeye çalışır. Doğru çözüm ağaçtan çıkarmaktı, eklentiyi geri koymak değil.
check("spin eklentisi yüklenmiyor",       'nav2_spin_action_bt_node' in _EKLENTILER, False)


# ─── BackUp gerçekten aracı hareket ettirebilmeli ───────────────────────────
# Üç eşik birden aşılmazsa geri gitme komutu araca hiç ulaşmaz:
#   1. velocity_smoother deadband'i altındaki komut sıfırlanır
#   2. velocity_smoother min_velocity tavanı
#   3. aracın kalkış sürtünmesi — 0.45 m/s altında yerinden kalkmıyor
_KALKIS_ESIGI = 0.45          # nav2_params min_approach_linear_velocity ile aynı
_VS = _nav2_params()['velocity_smoother']['ros__parameters']
_BT_TO_POSE, _ = _bt_dosyasi('default_nav_to_pose_bt_xml')
_BACKUP = [e for e in _ET.parse(_BT_TO_POSE).iter() if e.tag == 'BackUp'][0]
_BACKUP_HIZ  = float(_BACKUP.get('backup_speed'))
_BACKUP_MESAFE = float(_BACKUP.get('backup_dist'))

check("geri tavanı kalkış eşiği üstünde", abs(_VS['min_velocity'][0]) > _KALKIS_ESIGI, True)
check("backup hızı kalkış eşiği üstünde", _BACKUP_HIZ >= _KALKIS_ESIGI, True)
check("backup hızı tavanı aşmıyor",       _BACKUP_HIZ <= abs(_VS['min_velocity'][0]), True)
check("backup hızı deadband üstünde",     _BACKUP_HIZ > _VS['deadband_velocity'][0], True)
# Mesafe: planlayıcıyı yeniden çözebilir bir poza taşımaya yetecek kadar,
# aracın arkası kör olduğu için (LiDAR gövdeye takılıyor, geri kamera yok)
# fazlası göze alınmadı.
check("backup mesafesi makul",            0.2 <= _BACKUP_MESAFE <= 1.0, True)


# ─── Geri kayma koruması kasıtlı geri gitmeyi engellememeli ─────────────────
# Rampada "burun yukarı + hız negatif" tablosunu iki farklı şey üretir:
# istenmeyen geri kayma ve Nav2'nin BackUp kurtarması. Koruma ikisini
# ayırt etmezse kurtarmaya karşı komut basar; kurtarmanın tek işi aracı
# planlanamaz pozdan çıkarmaktı, engellenirse araç orada kalır.
from teknofest_ika.otonomi.pure_logic import rollback_mudahale_gerekli  # noqa: E402

_PE, _HE, _GE = 10.0, 0.05, 0.05     # pitch eşiği, hız eşiği, geri komut eşiği


def _mudahale(pitch, hiz, komut, bayat=False):
    return rollback_mudahale_gerekli(pitch, hiz, komut, bayat, _PE, _HE, _GE)


# Düz zeminde koruma hiç devreye girmez — geri gitmek serbest.
check("düz zeminde müdahale yok",         _mudahale(0.0, -0.3, 0.0), False)
# Rampada ileri komut varken geri kayıyorsa: GERÇEK kayma, müdahale et.
check("rampada gerçek kayma",             _mudahale(15.0, -0.3, 0.5), True)
check("rampada komutsuz kayma",           _mudahale(15.0, -0.3, 0.0), True)
# Rampada geri komut varken geri gidiyorsa: KASITLI, karışma.
check("kasıtlı geri gitmeye karışmaz",    _mudahale(15.0, -0.3, -0.5), False)
# Komut bayatsa koruma AÇIK kalır: komut bilinmiyorken varsayım "istenmeyen
# kayma" olmalı. Ters varsayım korumayı, ona en çok ihtiyaç duyulan anda
# (komut yayını kesildiğinde) sessizce kapatırdı.
check("bayat komutta koruma açık",        _mudahale(15.0, -0.3, -0.5, bayat=True), True)
# Eşik sınırı: tam eşikte koruma devam eder, altında bırakır.
check("eşikte koruma sürer",              _mudahale(15.0, -0.3, -_GE), True)
check("eşiğin altında bırakır",           _mudahale(15.0, -0.3, -_GE - 0.01), False)
# Hız negatif değilse zaten risk yok — geri komut olsa bile müdahale edilmez.
check("ileri giderken müdahale yok",      _mudahale(15.0, 0.4, -0.5), False)

# ─── BMS köprüsü: bayat okuma yayınlanmamalı ────────────────────────────────
print("\n=== BMS köprüsü ===")

_TAM = {'yas': 0.9, 'bagli': True, 'v48': 52.63, 'bms_enaz': 3289, 'bms_soc': 63}

def _gec(d): return bms_okuma_gecerli(d, BMS_YAS_ESIK_S)[0]

check("taze ve dolu okuma geçerli", _gec(_TAM), True)

# 🔑 BLE koptuğunda alanlar silinmiyor, SON DEĞERDE donuyor. Ölçüt bu yüzden
# bağlantı bayrağı değil okumanın yaşı.
_bayat = dict(_TAM, yas=45.0)
check("bayat okuma reddedilir",     _gec(_bayat), False)
check("bayat okumanın sebebi",
      bms_okuma_gecerli(_bayat, BMS_YAS_ESIK_S)[1], 'bayat')
# Bağlantı o an kopuk görünse bile son okuma tazeyse sayı kullanılabilir —
# tersi de doğru: bağlı görünüp donmuş veri yayınlanmamalı.
check("kopuk ama taze → geçerli",   _gec(dict(_TAM, bagli=False)), True)
check("bağlı ama bayat → geçersiz", _gec(dict(_TAM, bagli=True, yas=99.0)), False)

# Servis ayakta ama henüz çerçeve çözmemişse ölçüm alanları hiç yok.
check("ölçüm alanı yoksa geçersiz",
      _gec({'yas': 0.4, 'bagli': True}), False)
check("yaş alanı yoksa geçersiz",   _gec({'v48': 52.0, 'bms_enaz': 3289}), False)
check("gövde sözlük değilse geçersiz", _gec(None), False)

# Kesme kararı hücre dibinden verilir: LiFePO4 eğrisi düz olduğu için paket
# %60 SOC gösterirken tek bir çökmüş hücre dibi görmüş olabilir.
check("dip eşiğin altında → ölü",   bms_dip_olu(2750, BMS_HUCRE_DIP_MV), True)
check("dip tam eşikte → ölü",       bms_dip_olu(BMS_HUCRE_DIP_MV, BMS_HUCRE_DIP_MV), True)
check("dip eşiğin üstünde → sağlam", bms_dip_olu(3289, BMS_HUCRE_DIP_MV), False)

check("bayatlık eşiği",  BMS_YAS_ESIK_S,      10.0)
check("hücre dibi [mV]", BMS_HUCRE_DIP_MV,    2800)
check("hücre uyarı [mV]", BMS_HUCRE_UYARI_MV, 3000)
check("paket hücre sayısı", BMS_HUCRE_SAYISI, 16)

# Düğüm sağlığı SOC'a değil hücre dibine bakmalı; SOC yalnız gösterge.
with open(os.path.join(_KOK, 'teknofest_ika/gomulu/bms_koprusu.py'),
          encoding='utf-8') as f:
    _BMS = f.read()
check("sağlık hücre dibinden",  'bms_dip_olu(' in _BMS, True)
check("sağlık SOC'a bakmıyor",
      "bms_soc" in _BMS.split('def _saglik')[1], False)
check("tazelik saf fonksiyonda", 'bms_okuma_gecerli(' in _BMS, True)

# Düğüm çalıştırılabilir ve sahada gerçekten başlatılıyor olmalı.
with open(os.path.join(_KOK, 'setup.py'), encoding='utf-8') as f:
    check("bms_koprusu setup.py'de", 'bms_koprusu' in f.read(), True)
check("bms_koprusu boot betiğinde", 'bms_koprusu' in _BOOT, True)


def _kosul_derinligi(betik: str, arama: str) -> int:
    """Bir satırın kaç `if` bloğunun içinde kaldığı."""
    derinlik = 0
    for satir in betik.splitlines():
        sadeleşmiş = satir.strip()
        if arama in sadeleşmiş:
            return derinlik
        if sadeleşmiş.startswith('if ') and sadeleşmiş.endswith('then'):
            derinlik += 1
        elif sadeleşmiş == 'fi':
            derinlik -= 1
    return -1


# Batarya izleme kipten bağımsız olmalı. Düğüm bir kez IMU güvenlik bloğunun
# içine düştüğünde o bayrak varsayılan kapalı olduğu için araçta hiç
# başlamıyordu: hata vermiyor, log basmıyor, yalnız pano boş kalıyor.
check("bms_koprusu koşulsuz başlıyor",
      _kosul_derinligi(_BOOT, 'ros2 run teknofest_ika bms_koprusu'), 0)
# Yayıncısı koşulsuz başladığına göre sessizliği gerçek arızadır.
check("watchdog bataryayı izliyor", 'batarya_izle:=true' in _BOOT, True)


# ─── Açılış betiği ↔ launch: ayarlar sahaya ULAŞIYOR MU ─────────────────────
print("\n=== Boot ↔ launch parametre ayrışması ===")

# Sahada koşan yol açılış betiği; launch dosyası kullanılmıyor. Bir ayar
# yalnız launch'ta durursa araca HİÇ ulaşmaz ve bu sessizdir. Bu depoda dört
# kez oldu: SLAM parametre dosyası, sürüş kartı portu, gpio_mod ve nişan
# kamerasının HSV kalibrasyonu.
with open(os.path.join(_KOK, 'launch/gercek_arac.launch.py'), encoding='utf-8') as f:
    _LNC = f.read()

_boot_paramsiz = re.findall(r'ros2 run teknofest_ika (\w+)\s*>', _BOOT)

def _launch_bloklari(kaynak):
    blok = {}
    for m in re.finditer(r"executable='(\w+)'(.*?)\n    \)", kaynak, re.S):
        blok[m.group(1)] = m.group(2)
    return blok

_LB = _launch_bloklari(_LNC)

# Açılış betiği bu düğümlere hiç parametre geçmiyor; launch'ta ayarlanan her
# değer kod varsayılanıyla aynı olmalı, yoksa sahada başka bir araç koşar.
for _dugum, _yasak in [
    # image_timeout_sec ölçüme dayalı: üç kamera aynı USB2 hattını
    # paylaştığında akış ~7-9 Hz'e düşüyor ve eşik kare aralığının altına
    # inerse her kare STALE sayılıp nişan hiç çalışmıyor.
    ('targeting_node',      ['hsv_lower', 'hsv_upper', 'hsv_lower2',
                             'hsv_upper2', 'hough_max_radius',
                             'image_timeout_sec']),
    ('ackermann_converter', ['max_speed']),
    ('taret_rc_koprusu',    ['port', 'baud']),
]:
    _blok = _LB.get(_dugum, '')
    for _p in _yasak:
        check(f"launch {_dugum}.{_p} sabitlemiyor",
              re.search(rf"'{_p}':", _blok) is not None, False)

# Kalibrasyon tek kaynakta: düğümün varsayılanı topics.py'den geliyor.
with open(os.path.join(_KOK, 'teknofest_ika/gorsel/targeting_node.py'),
          encoding='utf-8') as f:
    _TN = f.read()
check("nişan HSV varsayılanı sabitten geliyor",
      'NISAN_HSV_ALT' in _TN and 'NISAN_HSV_UST' in _TN, True)
check("nişan HSV literal yazılmıyor",
      '[0, 100, 100]' in _TN, False)
check("Hough yarıçapı sabitten geliyor",
      'NISAN_HOUGH_MAX_YARICAP' in _TN, True)

from teknofest_ika.otonomi.topics import (  # noqa: E402
    NISAN_HSV_ALT, NISAN_HSV_UST, NISAN_HSV_ALT2, NISAN_HSV_UST2,
    NISAN_HOUGH_MAX_YARICAP,
)
# 2026-07-19 kalibrasyonu. Üst ton sınırı 4: turuncu bant sahte tespit
# üretiyor. V tabanı 70: karanlık sahteleri eler, loş ışıkta halkayı elemez.
check("HSV alt sınır",        NISAN_HSV_ALT,  [0, 40, 70])
check("HSV üst sınır",        NISAN_HSV_UST,  [4, 255, 255])
check("HSV sarma alt sınırı", NISAN_HSV_ALT2, [150, 40, 70])
check("HSV sarma üst sınırı", NISAN_HSV_UST2, [179, 255, 255])
check("Hough azami yarıçap",  NISAN_HOUGH_MAX_YARICAP, 250)


# ─── SLAM: parametre dosyası gerçekten yükleniyor mu ────────────────────────
print("\n=== SLAM yapılandırması ===")

# Dosya geçilmezse slam_toolbox stok ayarlarıyla açılıyor ve yaml'ın tamamı
# sessizce ölü kalıyor — çözünürlük, çerçeveler, döngü kapama, hiçbiri
# uygulanmıyor. Bu depoda üçüncü kez görülen desen.
with open(os.path.join(_KOK, 'scripts/lydia_startup.sh'), encoding='utf-8') as f:
    _BOOT = f.read()
check("boot betiği SLAM parametre dosyası geçiyor",
      'slam_params_file:=' in _BOOT, True)
check("SLAM tarama konusu sahada ezilebiliyor",
      'SLAM_SCAN_TOPIC' in _BOOT, True)

with open(os.path.join(_KOK, 'config/mapper_params_online_sync.yaml'),
          encoding='utf-8') as f:
    _SLAM = yaml.safe_load(f)['slam_toolbox']['ros__parameters']

# SLAM ile Nav2 aynı taramayı görmeli: farklı görmeleri, planlayıcının
# gördüğü engelin haritada olmaması (ya da tersi) demek.
with open(os.path.join(_KOK, 'config/nav2_params.yaml'), encoding='utf-8') as f:
    _NV = f.read()
_nav2_scan = re.search(r'scan:\s*\n\s*topic:\s*(\S+)', _NV)
check("SLAM ve Nav2 aynı taramayı okuyor",
      _SLAM['scan_topic'], _nav2_scan.group(1) if _nav2_scan else None)

# Ham /scan aracın arkasındaki gövde dönüşlerini taşıyor; menzil eşiği onları
# elemiyor, eleyen tek şey filtrenin açı kırpması.
check("SLAM ham taramayı okumuyor", _SLAM['scan_topic'] == '/scan', False)

# Sıfır eşik "her taramayı düğüm yap" demek ve duran araçta haritalama için
# yazılmıştı; koşuda poz grafiği gereksiz büyür.
check("hareket eşiği sıfır değil (mesafe)",
      _SLAM['minimum_travel_distance'] > 0.0, True)
check("hareket eşiği sıfır değil (yön)",
      _SLAM['minimum_travel_heading'] > 0.0, True)


# ─── Gövde Ölçüsü — footprint, urdf ve analiz betiği aynı aracı anlatmalı ───
print("\n=== Gövde ölçüsü tutarlılığı ===")

# Araçtan ölçülen gerçek gövde. Planlayıcı bu dikdörtgeni kullanıyor; küçük
# yazmak koridorlarda ve dönüşlerde olmayan bir pay uydurur.
# Boy ve en araçtan ÖLÇÜLDÜ. Kutu yüksekliği TÜRETİLİR: toplam yükseklik
# (0,78) eksi zemin boşluğu (0,365) — gövde zemine değmiyor.
# ⚠️ Taşmanın öne/arkaya dağılımı ölçülmedi; footprint simetrik varsayıyor.
_ARAC_BOY, _ARAC_GEN = 1.83, 1.17
# Footprint eni ayrı bir sayı: iki ölçüm 5 cm ayrıştı, büyüğü alındı.
_FOOTPRINT_EN = 1.22
_ARAC_YUK = round(0.78 - 0.365, 3)

with open(os.path.join(_KOK, 'config/nav2_params.yaml'), encoding='utf-8') as f:
    _NAV2 = f.read()

_fp = re.findall(r'footprint:\s*"(\[\[.*?\]\])"', _NAV2)
check("iki costmap de footprint kullanıyor", len(_fp), 2)
# robot_radius dairesel gövde varsayar; Ackermann araçta boy ile eni bir
# tutmak dar geçişlerde gerçek olmayan pay üretiyordu.
check("robot_radius parametresi yok",
      re.search(r'^\s*robot_radius:', _NAV2, flags=re.M) is None, True)

for _i, _metin in enumerate(_fp):
    _kose = [[float(x) for x in c.split(',')]
             for c in re.findall(r'\[([-\d., ]+)\]', _metin)]
    check(f"footprint[{_i}] dört köşe", len(_kose), 4)
    _boy = max(k[0] for k in _kose) - min(k[0] for k in _kose)
    _gen = max(k[1] for k in _kose) - min(k[1] for k in _kose)
    check(f"footprint[{_i}] boyu", round(_boy, 3), _ARAC_BOY)
    # 🔑 Footprint eni ölçülen ene EŞİT DEĞİL, ondan GENİŞ olmak zorunda.
    # Aracın en geniş yeri iki kez ölçülüp 5 cm ayrıştı (gövde boyunca
    # tekerlek dışları 1,17 · orta eksenden tekerleğe 0,61 → 1,22) ve
    # kullanıcı kararı büyüğünü almak oldu. Hata asimetrik: geniş footprint
    # geçebileceği yerden geçirmez (zaman), dar footprint SÜRTTÜRÜR (temas,
    # puan, hasar). Test bu yüzden alt sınır koyuyor — daraltan bir değişiklik
    # düşer, genişleten düşmez.
    check(f"footprint[{_i}] eni ölçülenden dar değil", _gen >= _ARAC_GEN, True)
    check(f"footprint[{_i}] eni seçilen değer",        round(_gen, 3), _FOOTPRINT_EN)
    # Dikdörtgen base_footprint'e göre ortalanmış olmalı: kaydırılmış bir
    # gövde, aracın önünü ya da arkasını costmap'te yanlış yere koyar.
    check(f"footprint[{_i}] ortalanmış",
          abs(max(k[0] for k in _kose) + min(k[0] for k in _kose)) < 1e-6, True)

with open(os.path.join(_KOK, 'urdf/arac.urdf'), encoding='utf-8') as f:
    _URDF = f.read()
_kutu = re.search(r'<box size="([\d.]+) ([\d.]+) ([\d.]+)"/>', _URDF)
check("urdf gövde kutusu boyu", float(_kutu.group(1)), _ARAC_BOY)
check("urdf gövde kutusu eni",  float(_kutu.group(2)), _ARAC_GEN)
check("urdf gövde kutusu yüksekliği", float(_kutu.group(3)), _ARAC_YUK)

# Dönüş yarıçapı analizi de aynı gövdeyi varsaymalı, yoksa geçilebilirlik
# hesabı gerçekte olmayan bir araç için çıkar.
with open(os.path.join(_KOK, 'scripts/parkur_cad/donus.py'), encoding='utf-8') as f:
    _DONUS = f.read()
_da = re.search(r'boy=([\d.]+), gen=([\d.]+)', _DONUS)
check("analiz betiği aynı boyu kullanıyor", float(_da.group(1)), _ARAC_BOY)
# Analiz PLANLAYICININ dikdörtgenini süpürmeli, ölçülen gövdeyi değil: daha
# dar bir dikdörtgenle "geçer" demek, Nav2'nin çarpışma göreceği bir yolu
# onaylamak olur.
check("analiz betiği footprint enini kullanıyor", float(_da.group(2)), _FOOTPRINT_EN)


_AR_KAYNAK = _kaynak('teknofest_ika/otonomi/anti_rollback.py')
# Komut aşağı akışta gerçekten uygulanacak olan topic'ten okunmalı: kendi
# override'ı oraya yazılmadığı için geri besleme oluşmaz.
check("anti_rollback komutu dinliyor",    'MUX_CMD_VEL_TOPIC' in _AR_KAYNAK, True)
check("anti_rollback niyeti sorguluyor",  'rollback_mudahale_gerekli(' in _AR_KAYNAK, True)
check("anti_rollback bayatlığı ölçüyor",  'NAV2_CMD_BAYATLAMA_S' in _AR_KAYNAK, True)
# Bayrak nabız gibi basılmalı: tek sefer basıp bırakmak, tüketici tarafta
# bayatlık ölçülemeyen kalıcı bir override üretir.
check("anti_rollback bayrağı nabız basıyor",
      _AR_KAYNAK.count('_durum_pub.publish') >= 2, True)

# Tüketici taraf: bayat bayrak override'ı BIRAKMALI. anti_rollback düğümü
# True bastıktan sonra ölürse, koruma olmadan Nav2 komutu sonsuza kadar
# kurtarma komutuyla değiştirilir ve araç sabit hızda sürülmeye devam eder.
_AC_KAYNAK2 = _kaynak('teknofest_ika/otonomi/ackermann_converter.py')
check("override bayatlığı ölçülüyor",
      'ROLLBACK_BAYATLAMA_S' in _AC_KAYNAK2, True)
check("bayatlık _cmd_vel_callback içinde uygulanıyor",
      'ROLLBACK_BAYATLAMA_S' in _AC_KAYNAK2.split('_cmd_vel_callback')[-1], True)
check("bayrağın geliş anı kaydediliyor",
      '_override_zaman' in _AC_KAYNAK2, True)
# Yokuş bayrağındaki koruma da yerinde kalmalı — ikisi aynı kusurun iki yüzü.
check("yokuş bayatlığı da duruyor",
      'YOKUS_BAYATLAMA_S' in _AC_KAYNAK2.split('_cmd_vel_callback')[-1], True)


# ─── Sürüş kartı hattı: her launch dosyasında AYNI port ve AYNI baud ────────
# Yinelenen sözlük anahtarı Python'da sessizdir — sonuncusu kazanır. İki
# launch dosyasında 'baud' iki kez yazılmıştı ve 921600'ün altına eklenen
# 115200 galip geliyordu: port doğru, hız yanlış. Sahadaki belirtisi kartın
# hiç konuşmaması, teşhisi ise `0x3C` sayaçlarında v0 sabitken v1'in artması.
# Denetim dizgeye değil AST'ye bakıyor; sözlüğü gerçekten değerlendirdiği
# için yinelenen anahtar yeniden eklenirse burada düşer.
import ast   # noqa: E402

_LAUNCH_DIZIN = os.path.join(_KOK, 'launch')
_launch_dosyalar = sorted(a for a in os.listdir(_LAUNCH_DIZIN)
                          if a.endswith('.launch.py'))
check("launch dosyaları bulundu", len(_launch_dosyalar) >= 3, True)

_yinelenen_anahtar = []
_kopru_ayarlari = []
for _ad in _launch_dosyalar:
    _kaynak_l = open(os.path.join(_LAUNCH_DIZIN, _ad), encoding='utf-8').read()
    _agac = ast.parse(_kaynak_l, _ad)
    for _dugum in ast.walk(_agac):
        if not isinstance(_dugum, ast.Dict):
            continue
        _anahtarlar = [k.value for k in _dugum.keys
                       if isinstance(k, ast.Constant) and isinstance(k.value, str)]
        if len(_anahtarlar) != len(set(_anahtarlar)):
            _yinelenen_anahtar.append(f'{_ad}:{_dugum.lineno}')
        if 'port' not in _anahtarlar or 'baud' not in _anahtarlar:
            continue
        _sozluk = {}
        for _k, _v in zip(_dugum.keys, _dugum.values):
            if isinstance(_k, ast.Constant) and isinstance(_v, ast.Constant):
                _sozluk[_k.value] = _v.value
        _kopru_ayarlari.append((_ad, _sozluk.get('port'), _sozluk.get('baud')))

check("launch dosyalarında yinelenen anahtar yok", _yinelenen_anahtar, [])
# Taret köprüsü portunu artık düğüm varsayılanından alıyor; port+baud çifti
# yazan tek yer sürüş kartı bloğu. Hiç bulunamazsa denetim boşa dönerdi.
check("launch'ta sürüş kartı bloğu var", len(_kopru_ayarlari) >= 2, True)
check("her launch aynı portu veriyor",
      sorted({p for _, p, _ in _kopru_ayarlari}), [SERIAL_ODOM])
check("her launch aynı baud'u veriyor",
      sorted({b for _, _, b in _kopru_ayarlari}), [SERIAL_BAUD_KART])

# ─── Atış kilidi SÜREYLE sınırlı — /shoot_command kenar sinyali ─────────────
# True ile False ayrı olaylar; arada yayınlayan düğüm ölürse kilidi açacak
# kimse kalmaz. Sonucu iki yerde birden ölümcül: ackermann_converter freni tam
# basılı bırakır, seri_kopru her sürüş komutunu sıfır hıza çevirir. Araç bir
# daha hiç hareket etmez ve log'a tek satır düşmez. Yokuş bayrağındaki
# bayatlık koruması buraya UYMAZ — o bayrak nabız gibi tekrarlanıyor, bu
# tekrarlanmıyor; ölçüt bu yüzden isteğin TOPLAM süresi.
from teknofest_ika.otonomi.topics import ATIS_AZAMI_S, LASER_FIRE_DURATION  # noqa: E402

_ATIS_GOVDE = _AC_KAYNAK[_AC_KAYNAK.index('def _cmd_vel_callback'):]
check("fren tarafında süre sınırı uygulanıyor",
      re.search(r'>\s*ATIS_AZAMI_S', _ATIS_GOVDE) is not None, True)
check("atış isteğinin başlangıcı okunuyor", '_atis_zaman' in _ATIS_GOVDE, True)
# Kenar sinyali: zaman YALNIZ yükselen kenarda kurulmalı. Her mesajda
# tazelenirse sınır hiç dolmaz ve koruma kâğıt üstünde kalır.
_ATIS_CB = _govde(_AC_KAYNAK, '    def _atis_cb')
check("başlangıç yükselen kenarda kuruluyor",
      'not self._atis_aktif' in _ATIS_CB, True)
# Sınır dolunca mandal da düşmeli; yalnız yerel değişkeni bırakmak, düğüm
# geri gelip yeni bir atış yayınladığında yükselen kenarı yutardı.
check("sınır dolunca mandal düşüyor",
      'self._atis_aktif = False' in _ATIS_GOVDE, True)

# Köprü tarafı: aynı kilit, aynı sınır. Denetim güvenlik zamanlayıcısından
# çağrılmalı, yoksa hiç koşmaz.
_KILIT = _govde(_SK_KAYNAK, '    def _atis_kilidi_denetle')
check("köprüde atış kilidi denetimi var", _KILIT != '', True)
check("köprü denetimi süre sınırını kullanıyor", 'ATIS_AZAMI_S' in _KILIT, True)
check("köprü sınır dolunca kilidi açıyor",
      'self._lazer_aktif = False' in _KILIT, True)
# Sahibi ölmüş bir atış isteğinin lazeri süresiz yakık bırakması, kilidi
# açmaktan daha kötü.
check("köprü sınır dolunca lazeri kapatıyor",
      re.search(r'PKT_J_LAZER,\s*0', _KILIT) is not None, True)
_GUV = _govde(_SK_KAYNAK, '    def _guvenlik_kontrol')
check("denetim güvenlik zamanlayıcısından çağrılıyor",
      '_atis_kilidi_denetle()' in _GUV, True)

# Sınır meşru en uzun tutuşun ÜSTÜNDE olmalı: ShootState tek denemede
# TIMEOUT_S kadar onay bekliyor, ardından LASER_FIRE_DURATION'ı tamamlıyor.
# Altına inerse koruma gerçek atışı keser.
_ST = _FSM_KAYNAK[_FSM_KAYNAK.index('class ShootState'):]
_ST_TIMEOUT = float(re.search(r'TIMEOUT_S\s*=\s*([\d.]+)', _ST).group(1))
check("ShootState onay penceresi okundu", _ST_TIMEOUT > 0, True)
check("sınır en uzun meşru atıştan büyük",
      ATIS_AZAMI_S > _ST_TIMEOUT + LASER_FIRE_DURATION, True)


# ─── FREN KAPISI: manuelde Jetson frene karışmaz ────────────────────────────
# Kart fren kaynaklarının BÜYÜĞÜNÜ alıyor ve bizimkini operatör çözemiyor
# (seri_kopru docstring'i, kart ekibinin sözleşmesi). Manuelde aracı kart
# kumandadan sürüyor; orada hesapladığımız her fren, sürücünün gazı bıraktığı
# anda üstüne binen ve açamadığı bir frene dönüşür.
from teknofest_ika.otonomi.topics import (   # noqa: E402
    MOD_MANUAL, MOD_FULL_AUTO, MOD_BAYATLAMA_S,
)

check("mod sabitleri topics.py'de", (MOD_MANUAL, MOD_FULL_AUTO), (0, 2))
# Yayıncı ve tüketici ayrı düğüm; sayı iki yerde tutulursa ayrışır ve
# ayrıştığı gün kimse fark etmez.
_MOD_KAYNAK2 = _kaynak('teknofest_ika/otonomi/mod_yoneticisi.py')
check("mod_yoneticisi sabiti yerel tanımlamıyor",
      re.search(r'^MOD_MANUAL\s*=', _MOD_KAYNAK2, re.M) is not None, False)

_AC3 = _kaynak('teknofest_ika/otonomi/ackermann_converter.py')
check("fren sahibi kipi dinliyor", 'MOD_AKTIF_TOPIC' in _AC3, True)

# 🔑 ASIL REGRESYON: fren TEK kapıdan çıkmalı. Yeni bir dal eklenip doğrudan
# publish edilirse kapı baypas edilir ve manuel koruması sessizce delinir.
# Ham publish yalnız iki yerde meşru: kapının kendisi ve kip geçişindeki
# sıfırlama. Aramayı fonksiyona hapsetmezsek beşinci bir dal fark edilmezdi.
_KAPI    = _govde(_AC3, '    def _fren_yayinla')
_MODCB   = _govde(_AC3, '    def _mod_cb')
_ham_top = _AC3.count('_fren_pub.publish')
_ham_ici = _KAPI.count('_fren_pub.publish') + _MODCB.count('_fren_pub.publish')
check("ham fren yayını yalnız kapıda ve kip geçişinde", _ham_top, _ham_ici)
check("kapı gerçekten yayınlıyor", _KAPI.count('_fren_pub.publish') >= 1, True)

# E-STOP kipten BAĞIMSIZ basmalı (B kararı): acil durdurma manuelde de
# geçerlidir. Diğer dallar geçmemeli, yoksa kapı hiçbir şey kısmaz.
_ESTOP_DALI = _govde(_AC3, '    def _cmd_vel_callback')
_ESTOP_DALI = _ESTOP_DALI[:_ESTOP_DALI.index('return')] if 'return' in _ESTOP_DALI else _ESTOP_DALI
check("E-STOP freni kipten muaf", 'estop=True' in _ESTOP_DALI, True)
check("muafiyet yalnız E-STOP'ta", _AC3.count('estop=True'), 1)

# Manuele GEÇİŞTE sıfır basılmalı. Susmak yetmez: kart son gönderdiğimiz
# değeri tutuyor, 1000 basılıyken susarsak o 1000 orada kalır.
check("kip geçişi yükselen kenarda yakalanıyor",
      'onceki != MOD_MANUAL' in _MODCB, True)
check("manuele geçişte sıfır basılıyor",
      re.search(r'UInt16\(data=0\)', _MODCB) is not None, True)

# Kip bilinmiyorsa fren SERBEST kalır. Ters kurmak, mod_yoneticisi otonom
# koşunun ortasında ölürse aracı %45 eğimde frensiz bırakırdı.
_MANUEL = _govde(_AC3, '    def _manuel_mi')
check("kip bilinmiyorsa fren serbest",
      'return False' in _MANUEL.split('yas')[0], True)
check("bayatlık kontrolü var",
      re.search(r'>\s*MOD_BAYATLAMA_S', _MANUEL) is not None, True)

# /mod/aktif 1 Hz nabız — bayatlama ondan büyük olmalı, yoksa kip daha ilk
# saniyede bayat sayılır ve koruma hiç çalışmaz.
_m_hz = re.search(r'create_timer\(([\d.]+),\s*self\._mod_yayinla\)', _MOD_KAYNAK2)
check("mod yayın periyodu okundu", _m_hz is not None, True)
if _m_hz:
    check("bayatlama en az üç nabız payı bırakıyor",
          MOD_BAYATLAMA_S >= 3.0 * float(_m_hz.group(1)), True)


# ─── Tekerlek çevresi karta ULAŞMALI ────────────────────────────────────────
# Kart bu sayı girilmeden 0x31'i bilerek 0 basıyor; sıfır kalırsa /odom
# ilerlemez ve Nav2 her hedefi "ilerleme yok" diye iptal eder. Değerin betikte
# yazılı olması yetmez — ros2 run satırına da GEÇMELİ. İlk sürümde değişken
# tanımlıydı ama düğüme verilmiyordu; o hâlde hiçbir şey değişmezdi.
_m_cevre_var = re.search(r':\s*"\$\{TEKERLEK_CEVRE_MM:=([\d.]+)\}"', _BETIK)
check("tekerlek çevresi betikte tanımlı", _m_cevre_var is not None, True)
check("çevre ros2 run satırına geçiyor",
      're.search' and 'tekerlek_cevre_mm:="$TEKERLEK_CEVRE_MM"' in _BETIK, True)
# Değer DAYATILMIYOR: ölçülene kadar 0 kalması doğru davranış. Denetim
# ölçüme değil, ölçüm geldiğinde karta ULAŞACAĞINA bakıyor.
from teknofest_ika.otonomi.pure_logic import (   # noqa: E402
    ayar_ham, ayar_gonderilecek, AYAR_CEVRE_MM,
)
# Sıfır "ölçüm yok" demek ve gönderilmemeli — kartın bilerek sustuğu alana
# uydurma sayı yazmak, susmasından kötüdür.
check("sıfır çevre gönderilmiyor", ayar_gonderilecek(0.0), False)
check("ölçülmüş çevre gönderilir", ayar_gonderilecek(1842.5), True)
# Ölçek mm x 10, alan int16. Ölçek yanlış kurulursa sayı sessizce on kat
# yanlış gider; aralık aşılırsa ayar hiç gönderilmez.
_ham, _sebep = ayar_ham(AYAR_CEVRE_MM, 1842.5)
check("çevre karta gönderilebilir", _sebep, 'gecerli')
check("çevre ölçeği mm x 10", _ham, 18425)
check("çevre int16'ya sığıyor", -32768 <= _ham <= 32767, True)
# int16 tavanı 3,2 m çevreye kadar yer bırakıyor — bundan büyüğü reddedilmeli.
check("aşırı çevre reddediliyor", ayar_ham(AYAR_CEVRE_MM, 4000.0)[1], 'aralik_disi')


# ─── KONU MESAJ TİPLERİ TÜM DEPODA TUTARLI OLMALI ──────────────────────────
# Kart ekibi 040'ta yakaladı: ackermann_converter /mod/aktif'e UInt16 abone
# oluyordu, yayıncı ve diğer beş tüketici UInt8 kullanıyordu. DDS farklı
# tipleri EŞLEŞTİRMEZ ve bu sessiz bir arızadır — hata yok, uyarı yok,
# sadece mesaj hiç gelmez. Sonucu: manuelde fren kapısı hiç devreye girmedi.
#
# Bu denetim dizgeye bakmıyor, AST'den `create_subscription(TİP, KONU, ...)`
# ve `create_publisher(TİP, KONU, ...)` çağrılarını çıkarıp konu sabiti
# başına tip kümesi kuruyor. Bir konuda iki tip görünürse düşer.
import ast as _ast   # noqa: E402

_TIP_HARITA = {}     # konu sabiti -> {tip: [dosya...]}
for _dosya in sorted(glob.glob(os.path.join(_KOK, 'teknofest_ika', '**', '*.py'),
                               recursive=True) +
                     glob.glob(os.path.join(_KOK, 'scripts', '*.py'))):
    try:
        _agac = _ast.parse(open(_dosya, encoding='utf-8').read(), _dosya)
    except SyntaxError:
        continue
    _takma = {}
    for _d in _ast.walk(_agac):
        if isinstance(_d, _ast.ImportFrom):
            for _n in _d.names:
                if _n.asname:
                    _takma[_n.asname] = _n.name
    for _d in _ast.walk(_agac):
        if not isinstance(_d, _ast.Call) or not isinstance(_d.func, _ast.Attribute):
            continue
        if _d.func.attr not in ('create_subscription', 'create_publisher'):
            continue
        if len(_d.args) < 2:
            continue
        _tip, _konu = _d.args[0], _d.args[1]
        # Yalnız "tip adı + konu sabiti" biçimindekiler; string konular ve
        # değişkenden gelenler atlanır (karşılaştırılabilir değiller).
        if not isinstance(_tip, _ast.Name) or not isinstance(_konu, _ast.Name):
            continue
        if not _konu.id.isupper():
            continue
        # Takma ad çözümü: `from std_msgs.msg import Bool as BoolMsg` yerel adı
        # değiştiriyor ama mesaj tipi aynı. Yerel adı gerçek tipe çeviriyoruz,
        # yoksa aynı tipin iki adı çakışma sanılır.
        _TIP_HARITA.setdefault(_konu.id, {}).setdefault(
            _takma.get(_tip.id, _tip.id), []).append(os.path.basename(_dosya))

check("konu-tip taraması bir şey buldu", len(_TIP_HARITA) >= 10, True)
_catisan = {k: v for k, v in _TIP_HARITA.items() if len(v) > 1}
if _catisan:
    for _k, _v in sorted(_catisan.items()):
        print(f"    ÇAKIŞMA {_k}: " +
              " | ".join(f"{t} ({', '.join(sorted(set(d)))})" for t, d in _v.items()))
check("hiçbir konuda mesaj tipi çakışması yok", sorted(_catisan), [])

# /mod/aktif özel olarak kilitleniyor — bu hatanın çıktığı yer.
check("/mod/aktif tek tip kullanıyor",
      sorted(_TIP_HARITA.get('MOD_AKTIF_TOPIC', {})), ['UInt8'])

# ─── Kapanmış porta yazmak düğümü ÖLDÜRMEMELİ ──────────────────────────────
# Kart her firmware yüklemesinde resetleniyor; okuma döngüsü portu kapatırken
# /cmd_vel geri çağrısı yazmaya devam ediyor. pyserial bu durumda
# SerialException değil TypeError üretiyor (iç iptal borusu None oluyor).
# Yakalanmayan istisna köprüyü öldürdü ve açılış betiği düğümleri
# denetlemediği için köprü bir daha dönmedi — boşta kalan portu
# f767_telemetri kaptı.
_GONDER = _govde(_SK_KAYNAK, '    def _paket_gonder')
check("yazmadan önce port açık mı denetleniyor", 'ser.is_open' in _GONDER, True)
# 🔑 Dizge araması YETMEZ: açıklama yorumunda da "TypeError" geçiyor ve ilk
# sürümde bu test, except demetinden TypeError silindiğinde bile geçiyordu —
# mutasyon turu yakaladı. Denetim artık except SATIRINI eşleştiriyor.
_m_exc = re.search(r'except\s*\(([^)]*)\)\s*as\s+\w+:', _GONDER)
check("except demeti bulundu", _m_exc is not None, True)
check("TypeError except demetinde",
      'TypeError' in (_m_exc.group(1) if _m_exc else ''), True)
check("hata sonrası port kapatılıyor", '_port_kapat()' in _GONDER, True)


# ─── Derinlik kaynağı: costmap ve açılış betiği aynı şeyi söylemeli ─────────
print("\n=== Derinlik kamerası kapalılığı ===")

# Derinlik bulutu Nav2'ye ölçülmemiş bir dönüşümün arkasından giriyordu:
# base_link → dm_base_frame'in üç sayısı tahmin. Yükseklik yanlışsa bulut
# düşeyde kayar, zemin min_obstacle_height'ın üstüne çıkar ve ENGEL olarak
# işaretlenir — araç kendini duvarla çevrili sanıp hiç rota üretmez.
#
# Kapatmanın iki yeri var ve ikisi ayrışırsa arıza sessiz: yalnız TF kapalıysa
# kaynak listede kalır ve costmap "Transform failure" basar; yalnız liste
# temizse boşa bir kamera ve bir TF yayıncısı koşar.
_NAV2_D = _nav2_params()

for _ad in ('global_costmap', 'local_costmap'):
    _ob = _NAV2_D[_ad][_ad]['ros__parameters']['obstacle_layer']
    # Nav2 tam olarak bu dizgeyi boşluktan bölüp okuyor; kaynağı açan tek şey
    # adın burada geçmesi, blok tanımının varlığı değil.
    _kaynaklar = _ob['observation_sources'].split()
    check(f"{_ad}: derinlik kaynağı listede değil",
          'os30a_cloud' in _kaynaklar, False)
    # Ters yön: listede olup bloğu olmayan bir ad Nav2'yi açılışta düşürür.
    for _k in _kaynaklar:
        check(f"{_ad}: {_k} kaynağının bloğu var", _k in _ob, True)

# Açılış betiğinin varsayılanları — `: "${AD:=değer}"` biçimindeki atamalar
# okunuyor, dizge aranmıyor: değer değişirse test düşer.
_VARSAYILAN = dict(re.findall(r'^:\s*"\$\{([A-Z0-9_]+):=([^}]*)\}"',
                              _kaynak('scripts/lydia_startup.sh'), flags=re.M))
check("derinlik kamerası varsayılan kapalı", _VARSAYILAN.get('DERINLIK_AKTIF'), '0')
# TF'in varsayılanı kameranınkine BAĞLI olmalı; sabit bir 1 yazmak ikisinin
# sessizce ayrışmasının kendisidir.
check("OS30A TF'i kameranın anahtarını izliyor",
      _VARSAYILAN.get('OS30A_TF_AKTIF'), '$DERINLIK_AKTIF')

# Kamera launch'ı gerçekten o anahtarın dalında olmalı — anahtarı tanımlayıp
# kullanmamak bu depoda daha önce görülen desen.
_BOOT_D = _kaynak('scripts/lydia_startup.sh')
_m_dal = re.search(r'if \[ "\$DERINLIK_AKTIF" != "1" \]; then(.*?)\nfi\n',
                   _BOOT_D, flags=re.S)
check("derinlik dalı bulundu", _m_dal is not None, True)
check("OS30A launch'ı o dalın içinde",
      'ros2 launch ydlidar_os30a' in (_m_dal.group(1) if _m_dal else ''), True)
# Kamera kapalıyken derinlik bulutunu tüketen başka bir yol açık kalmamalı.
# Varsayılanın DEĞERİ okunuyor: adın dosyada geçmesi True'ya çevrilmesini
# engellemez.
_m_di = re.search(r'declare_parameter\("derinlik_isle",\s*(\w+)\)',
                  _kaynak('teknofest_ika/gorsel/preprocessing_node.py'))
check("derinlik_isle varsayılanı okundu", _m_di is not None, True)
check("preprocessing derinlik işlemesi varsayılan kapalı",
      _m_di.group(1) if _m_di else None, 'False')


# ─── Kart ayarları (0x09) açılış betiğinden geçmeli ────────────────────────
print("\n=== Kart ayarlarının sahaya ulaşması ===")

# Ayarlar kartın FLASH'ında değil RAM'inde: kalıcılık köprüde ve köprüye
# değerler yalnız parametreden giriyor. Betikte geçirilmeyen bir ayar sahada
# HİÇ girilemez — ölçüm yapılır, girildi sanılır, kart eski değerde kalır.
_SK_AYAR = set(re.findall(
    r"declare_parameter\('(tekerlek_cevre_mm|gosterge_darbe_tur|"
    r"enkoder_disli_orani|direksiyon_orani|direksiyon_isaret)'",
    _kaynak('teknofest_ika/gomulu/seri_kopru.py')))
check("köprü beş ayarı da tanımlıyor", len(_SK_AYAR), 5)

# Yalnız seri_kopru çağrısına bakılıyor: aynı adın betiğin başka bir yerinde
# yorum olarak geçmesi ayarı düğüme ulaştırmaz.
_BOOT_A = _kaynak('scripts/lydia_startup.sh')
_m_sk = re.search(r'ros2 run teknofest_ika seri_kopru --ros-args(.*?)&\n',
                  _BOOT_A, flags=re.S)
check("seri_kopru çağrısı bulundu", _m_sk is not None, True)
_CAGRI = _m_sk.group(1) if _m_sk else ''
for _ad in sorted(_SK_AYAR):
    check(f"{_ad} düğüme geçiriliyor", f'-p {_ad}:=' in _CAGRI, True)

# Dördünün varsayılanı 0 olmalı: sıfır "ölçülmedi" demek ve köprü onu
# göndermiyor. Uydurma bir sayı, kartın bilerek sustuğu alana yanlış ölçek
# yazmaktır. Çevre ve dişli oranı ayrıca KART TARAFINDA ölçülü ve kart
# "göndermeyin" diyor; buradan gönderilirse kartın kendi değerini ezer.
for _ad in ('TEKERLEK_CEVRE_MM', 'GOSTERGE_DARBE_TUR', 'ENKODER_DISLI_ORANI',
            'DIREKSIYON_ISARET'):
    check(f"{_ad} varsayılanı ölçülmedi (0)", _VARSAYILAN.get(_ad), '0')

# Kolon/teker oranı ölçüldü ve kartta karşılığı YOK — göndermek bizim işimiz.
# Dizge karşılaştırması yerine değer köprünün ölçek yolundan geçiriliyor:
# betikteki sayı doğru yazılmış olsa bile yanlış ölçekte int16'ya sığmazsa
# ya da elenirse kart alanı boş kalır ve direksiyon büyüklüğü yanlış sürer.
_ORAN_VAR = float(_VARSAYILAN.get('DIREKSIYON_ORANI', '0'))
check("kolon/teker oranı gönderilecek", ayar_gonderilecek(_ORAN_VAR), True)
check("kolon/teker oranı ham değeri", ayar_ham(AYAR_DIREKSIYON, _ORAN_VAR)[0],
      13091)
# Oran redüktör çıkışı 2 tur = 720° kolon ÷ 55° teker ölçümünden geliyor.
check("oran ölçümle tutuyor", round(720.0 / 55.0, 3), round(_ORAN_VAR, 3))
# Kolon kelepçesi ile oran birlikte ackermann'ın tavanını belirliyor: komut
# edilen en büyük teker açısı kolonda ±750°'yi AŞMAMALI, yoksa kart kırpar ve
# Nav2 istediği açıyı hiç alamadan dümdüz gider.
check("max_steering_angle kolon kelepçesinin içinde",
      math.degrees(0.5236) * _ORAN_VAR <= 750.0, True)


# ─── Seri portu iki okuyucu paylaşamaz ─────────────────────────────────────
print("\n=== F767 port kavgası ===")

# f767_telemetri ve seri_kopru varsayılan olarak AYNI portu istiyor. İkisi
# birden açtığında çekirdek "multiple access on port" diyor ve çerçeveler
# bölünüyor; kimin kazandığı açılış sırasına kalıyor. Telemetri kazanırsa
# /kart/* konularının TAMAMI boş kalır — odometri, IMU, RC, E-STOP, mod.
check("iki okuyucu aynı portu istiyor (kapı gerekli)",
      _VARSAYILAN.get('F767_TELEMETRI_PORT'), _VARSAYILAN.get('SERI_PORT'))

# Kapının kendisi ÇALIŞTIRILARAK denetleniyor: dizge araması "kapı var" der
# ama koşulu tersine çevrilmiş bir kapıyı da onaylar.
_m_kapi = re.search(
    r'(if \[ "\$F767_TELEMETRI_AKTIF" = "1" \] && \[ "\$F767_TELEMETRI_PORT" '
    r'= "\$SERI_PORT" \]; then.*?\nfi\n)',
    _kaynak('scripts/lydia_startup.sh'), flags=re.S)
check("port kavgası kapısı bulundu", _m_kapi is not None, True)

if _m_kapi:
    import subprocess

    def _kapi_sonucu(telemetri_port, seri_port):
        betik = (f'F767_TELEMETRI_AKTIF=1\n'
                 f'F767_TELEMETRI_PORT={telemetri_port}\n'
                 f'SERI_PORT={seri_port}\n'
                 + _m_kapi.group(1) +
                 'echo "SONUC=$F767_TELEMETRI_AKTIF"\n')
        cikti = subprocess.run(['bash', '-c', betik], capture_output=True,
                               text=True).stdout
        return re.search(r'SONUC=(\d)', cikti).group(1)

    # Aynı port → telemetri geri çekilmeli. Hattın sahibi köprü: teşhis
    # servisi sürüşün kendisinden önce gelemez.
    check("aynı portta telemetri kapanıyor",
          _kapi_sonucu('/dev/f767', '/dev/f767'), '0')
    # Ayrı port → telemetri çalışmaya devam etmeli, yoksa kapı panoyu
    # gereksiz yere köreltir.
    check("ayrı portta telemetri açık kalıyor",
          _kapi_sonucu('/dev/f767_teshis', '/dev/f767'), '1')


# ─── SLAM anahtarı ve Nav2'nin çerçevesi ───────────────────────────────────
print("\n=== SLAM kapalı, Nav2 odom'da ===")

# Haritalama sürüş zincirinin dışında: hedef taramadan doğuyor ve odom'da
# gönderiliyor. Ama `map` çerçevesini basan tek şey slam_toolbox'tı — SLAM
# kapatılıp Nav2 `map`'te bırakılırsa bt_navigator ilk tick'te dönüşümü
# bulamaz ve TEK HEDEF KABUL ETMEZ. Yani anahtar ile çerçeve tek bir karardır;
# ikisini ayrı ayrı doğru bulmak yetmiyor, birlikte tutarlı olmaları gerekiyor.
check("SLAM varsayılan kapalı", _VARSAYILAN.get('SLAM_AKTIF'), '0')

_NAV = yaml.safe_load(_kaynak('config/nav2_params.yaml'))

# Yalnız GERÇEKTEN başlatılan sunucular. amcl ve map_server bu depoda ölü
# yapılandırma (lifecycle_manager node_names'te yoklar), çerçeveleri
# bağlayıcı değil.
for _dugum, _yol in [
    ('bt_navigator',    ['bt_navigator']),
    ('behavior_server', ['behavior_server']),
    ('global_costmap',  ['global_costmap', 'global_costmap']),
    ('local_costmap',   ['local_costmap', 'local_costmap']),
]:
    _d = _NAV
    for _k in _yol:
        _d = _d[_k]
    check(f"{_dugum} odom çerçevesinde",
          _d['ros__parameters']['global_frame'], 'odom')

# Hedefin çerçevesi ile bt_navigator'ın çerçevesi BİRLİKTE değişmeli.
# Değerin kendisi yukarıda ayrıca kilitli; buradaki kontrol iki DOSYA
# arasındaki bağı tutuyor: Nav2 map'e taşınırsa hedef odom'da kalır ve
# aradaki dönüşümü basan kimse olmadığı için her hedef sessizce reddedilir.
check("hedef çerçevesi bt_navigator'ınkiyle aynı",
      KAYAN_HEDEF_FRAME,
      _NAV['bt_navigator']['ros__parameters']['global_frame'])

_GC = _NAV['global_costmap']['global_costmap']['ros__parameters']
# static_layer'ın tek girdisi /map'ti. Listede kalırsa katman hiç veri almadan
# her güncellemeye boş bir katman bindirir.
check("static_layer plugin listesinde değil",
      'static_layer' in _GC['plugins'], False)
# static_layer'sız global costmap büyük ölçüde BİLİNMEYEN kalıyor. Planlayıcı
# bunun içinden plan üretebildiği için kilitlenme olmuyor — o yüzden bu bayrak
# artık static_layer'ın kaldırılmasının ÖN KOŞULU, keyfî bir ayar değil.
check("planlayıcı bilinmeyenin içinden plan üretiyor",
      _NAV['planner_server']['ros__parameters']['GridBased']['allow_unknown'],
      True)

# Kapılar ÇALIŞTIRILARAK denetleniyor: dizge araması koşulu tersine çevrilmiş
# bir kapıyı da onaylar (bu depoda üç kez görüldü).
import subprocess  # noqa: E402
import tempfile  # noqa: E402

_BOOT_S = _kaynak('scripts/lydia_startup.sh')
_m_slam = re.search(r'(if \[ "\$SLAM_AKTIF" != "1" \]; then\n.*?\nfi\n)',
                    _BOOT_S, flags=re.S)
check("SLAM kapısı bulundu", _m_slam is not None, True)

if _m_slam:
    def _slam_kalkti_mi(aktif):
        with tempfile.TemporaryDirectory() as _tmp:
            # ros2, sed ve sleep gölgeleniyor: kabuk fonksiyonu PATH'ten önce
            # gelir, yani gerçek bir düğüm başlatılmadan ve 17 saniye
            # beklenmeden dalın hangisi olduğu ölçülür.
            #
            # 🔑 Çağrılan satırlar çıktılarını `> "$LOG/*.log"` ile
            # yönlendiriyor, yani shim'in stdout'u dalın içinde YUTULUYOR.
            # İlk yazdığım sürüm bunu görmedi ve SLAM_AKTIF=1'de de boş çıktı
            # okuyup "başlatmıyor" diyordu. İşaret ayrı bir dosyaya yazılıyor:
            # fonksiyonun içindeki açık yönlendirme dışarıdakini yener.
            _iz = os.path.join(_tmp, 'iz.txt')
            betik = (f'SLAM_AKTIF={aktif}\n'
                     f'LOG={_tmp}\nWS={_tmp}\nSLAM_SCAN_TOPIC=/scan/filtered\n'
                     f'IZ={_iz}\n'
                     'ros2() { echo "ROS2 $*" >> "$IZ"; }\n'
                     'sed() { :; }\n'
                     'sleep() { :; }\n'
                     + _m_slam.group(1)
                     # Dalın iki `ros2` çağrısı da arka plana atılıyor. `sleep`
                     # boş olduğu için kabuk `fi`'yi geçip çıkarken alt kabuk
                     # izi yazmayı bitirmemiş olabiliyor: 300 koşuda 2 kez
                     # map_image_node satırı eksik ölçüldü. `wait` o yarışı
                     # kapatır; dalın kendisinde böyle bir sorun yok, gerçek
                     # `ros2` çağrıları düğümü kendisi başlatıyor.
                     + 'wait\n')
            subprocess.run(['bash', '-c', betik], capture_output=True, text=True)
            if not os.path.exists(_iz):
                return ''
            with open(_iz, encoding='utf-8') as f:
                return f.read()

    _kapali, _acik = _slam_kalkti_mi(0), _slam_kalkti_mi(1)
    check("SLAM_AKTIF=0 slam_toolbox'ı başlatmıyor",
          'slam_toolbox' in _kapali, False)
    check("SLAM_AKTIF=1 slam_toolbox'ı başlatıyor",
          'slam_toolbox' in _acik, True)
    # map_image_node'un tek girdisi /map. SLAM'siz başlatmak, hiç kare
    # üretmeyen bir düğümü açılış doğrulamasında "ayakta" saydırmak olurdu.
    check("SLAM_AKTIF=0 map_image_node'u başlatmıyor",
          'map_image_node' in _kapali, False)
    check("SLAM_AKTIF=1 map_image_node'u başlatıyor",
          'map_image_node' in _acik, True)

# Açılış doğrulaması SLAM kapalıyken map_image_node'u ARAMAMALI: her açılışta
# basılan sahte bir "ayağa kalkmadı" uyarısı, kontrolün kendisini değersiz
# kılar ve gerçek eksik düğümü gizler.
# Atama çok satırlı; kapı satırı ONDAN SONRA geliyor. Tembel `.*?` ilk
# satırda durup opsiyonel grubu boş bırakıyordu — yani test kapıyı hiç
# çalıştırmadan "yok" diyordu. Kapanış tırnağına kadar açıkça eşleştiriliyor.
_m_bek = re.search(
    r'(_BEKLENEN="seri_kopru[^"]*"\n'
    r'\[ "\$SLAM_AKTIF" = "1" \] && _BEKLENEN="\$_BEKLENEN map_image_node"\n)',
    _BOOT_S, flags=re.S)
check("beklenen düğüm listesi bulundu", _m_bek is not None, True)

if _m_bek:
    def _beklenen(aktif):
        betik = (f'SLAM_AKTIF={aktif}\n' + _m_bek.group(1) +
                 'echo "LISTE=$_BEKLENEN"\n')
        return subprocess.run(['bash', '-c', betik], capture_output=True,
                              text=True).stdout

    check("SLAM kapalıyken map_image_node beklenmiyor",
          'map_image_node' in _beklenen(0), False)
    check("SLAM açıkken map_image_node bekleniyor",
          'map_image_node' in _beklenen(1), True)


# ─── Costmap bayatlamasın ──────────────────────────────────────────────────
print("\n=== Costmap tazeliği ===")

_CM = {
    'global': _NAV['global_costmap']['global_costmap']['ros__parameters'],
    'local':  _NAV['local_costmap']['local_costmap']['ros__parameters'],
}

# ① Kaynak susarsa costmap donuyor. Temizleme de işaretleme de AYNI kaynaktan
# geldiği için, /scan/filtered kesildiğinde son engeller hiç silinmez ve araç
# hayalet duvarlardan kaçmaya çalışır. Denetim yalnız taramaya konur; iki
# bulut kaynağı yalnız görecek şey varken yayın yaptığı için onlara konursa
# kalıcı sahte alarm olur (② ve ③).
for _ad, _p in _CM.items():
    _scan = _p['obstacle_layer']['scan']
    check(f"{_ad} costmap taramanın tazeliğini denetliyor",
          _scan.get('expected_update_rate', 0.0) > 0.0, True)
    # Ölçülen tarama 9.96 Hz. Eşik periyodun altına inerse her açılışta sahte
    # alarm başlar ve denetim değersizleşir; 5 s üstü ise arızayı geç söyler.
    check(f"{_ad} tazelik eşiği makul aralıkta",
          0.3 <= _scan['expected_update_rate'] <= 5.0, True)

# ② Koşullu yayın yapan kaynağa denetim konmaz. Bağ koda dayanıyor: kaynak
# koşulsuz yayına çevrilirse bu test düşer ve denetimin eklenmesi gündeme
# gelir — yani karar ikisini birlikte tutuyor.
check("cone_fusion koni yokken yayın yapmıyor",
      'if not cone_centers:' in _kaynak('teknofest_ika/gorsel/cone_fusion_node.py'),
      True)
check("kayar engel kaynağı engel yokken susuyor",
      'if not self._engel_aktif:' in
      _kaynak('teknofest_ika/gorsel/kayar_engel_costmap.py'), True)
for _ad, _p in _CM.items():
    for _kaynak_adi in ('yolo_cone_cloud', 'moving_obs_cloud'):
        check(f"{_ad}/{_kaynak_adi} tazelik denetimi almıyor",
              'expected_update_rate' in _p['obstacle_layer'][_kaynak_adi], False)

# ③ İşaretleme menzili temizleme menzilini AŞAMAZ. Aşarsa, ışının hiç
# ulaşamadığı bir hücreye engel basılır ve o hücreyi silebilecek tek mekanizma
# kalmaz — pencere kayana kadar orada durur. Kaynak başına denetlenir, çünkü
# menziller kaynak başına ayrı yazılıyor.
for _ad, _p in _CM.items():
    for _kn, _ka in _p['obstacle_layer'].items():
        if not isinstance(_ka, dict) or 'obstacle_max_range' not in _ka:
            continue
        if _kn not in _p['obstacle_layer']['observation_sources'].split():
            continue   # listede olmayan blok okunmuyor
        check(f"{_ad}/{_kn} temizleme menzili işaretlemeyi kapsıyor",
              _ka['raytrace_max_range'] >= _ka['obstacle_max_range'], True)

# ④ Temizleme yetkisi olmayan kaynağın bıraktığı işaretin ömrünü SINIRLAYAN
# tek şey pencerenin kayması. Pencere büyüdükçe hayalet engel koşu boyunca
# yaşar. Alt sınırı hedefin kendisi belirliyor: en uzak hedef pencerenin
# içinde kalmazsa planlayıcı hedefi göremez.
import inspect  # noqa: E402
from teknofest_ika.otonomi import pure_logic as _pl  # noqa: E402
_MAKS_ILERI = inspect.signature(_pl.kayan_hedef).parameters['maks_ileri'].default
_YARI_BOY = 1.90 / 2.0
# Yalnız global denetleniyor: planlayıcının gördüğü pencere o. Local 8 m,
# kontrolcünün anlık çevresi ve hedefi kapsamak zorunda değil.
check("global pencere en uzak hedefi kapsıyor",
      _CM['global']['width'] / 2.0 >= _MAKS_ILERI + _YARI_BOY, True)
# Üst sınır: pencere ne kadar büyükse bayat işaret o kadar uzun yaşıyor.
# 20 m, 8 m'lik hedefe 1.05 m pay bırakan en küçük makul değer.
check("global pencere gereksiz büyük değil", _CM['global']['width'] <= 24, True)
check("global ve local pencere karıştırılmamış",
      _CM['global']['width'] > _CM['local']['width'], True)


# ─── Kayan sürüş: kör sürme ve gürültünün yola sayılması ───────────────────
print("\n=== Kayan sürüş ölçüm kapıları ===")

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    olcum_bayat_mi, yol_artimi,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    KAYAN_TARAMA_BAYATLAMA_S, KAYAN_YOL_TABAN_HIZ_MS,
    KAYAN_ODOM_BAYATLAMA_S, KAYAN_HEDEF_PERIYOT_S,
)

# ① Bayatlama kapısı. "Hiç gelmedi" (0.0) bayatla aynı sınıf: ikisinde de
# elde güncel ölçüm yok.
check("hiç gelmemiş ölçüm bayat sayılıyor", olcum_bayat_mi(0.0, 100.0, 1.0), True)
# Yukarıdaki tek başına yetmiyor: wall-clock'ta `simdi - 0.0` zaten sınırı
# aştığı için `== 0.0` dalı silinse de sonuç aynı çıkıyor (mutasyon kaçtı).
# Ayırt eden durum saatin küçük olduğu hâl — zaman kaynağı bir gün
# time.monotonic()'e çevrilirse (açılıştan beri geçen saniye) bu dal
# yük taşımaya başlar ve "hiç gelmedi" sessizce "taze" okunur.
check("saat küçükken de hiç gelmemiş ölçüm bayat",
      olcum_bayat_mi(0.0, 0.5, 1.0), True)
check("taze ölçüm geçiyor",                 olcum_bayat_mi(99.5, 100.0, 1.0), False)
# Sınırın kendisi HENÜZ bayat değil: eşitlikte düşmek, tam periyotta gelen
# bir akışı her seferinde arıza saymak olurdu.
check("sınırdaki ölçüm bayat değil",        olcum_bayat_mi(99.0, 100.0, 1.0), False)
check("sınırı geçen ölçüm bayat",           olcum_bayat_mi(98.9, 100.0, 1.0), True)

# ② Tarama kapısı ODOMETRİNİNKİYLE aynı sınıfta olmalı. Asimetri tam olarak
# düzeltilen kusurdu: odometri korunuyordu, tarama korunmuyordu.
check("tarama ve odometri kapıları aynı sınıfta",
      KAYAN_TARAMA_BAYATLAMA_S, KAYAN_ODOM_BAYATLAMA_S)
# Bayatlama sınırı yeniden hedefleme periyodundan kısa olmalı: uzun olsaydı
# araç, kapı açılmadan önce ölü taramadan üretilmiş bir hedefe sürerdi.
check("bayatlama sınırı hedef periyodunun altında",
      KAYAN_TARAMA_BAYATLAMA_S < KAYAN_HEDEF_PERIYOT_S, True)

# ③ İki kapı da TEK fonksiyondan geçmeli. Ayrı ayrı yazılmış iki karşılaştırma
# bu kusurun ta kendisiydi: biri güncellendi, öteki unutuldu.
import ast as _ast  # noqa: E402
_AGAC = _ast.parse(_kaynak('teknofest_ika/otonomi/misyon_fsm.py'))

# Arama SINIF SINIRI tanımak zorunda: `_on_scan` dosyada iki kez geçiyor
# (KoridorIzleyici ve KayanHedefSurucusu). Sınırsız arama ilkini bulup
# yanlış sınıfı denetliyordu — bu ders bu depoda daha önce de çıkmıştı.
_SINIF = next((d for d in _ast.walk(_AGAC)
               if isinstance(d, _ast.ClassDef) and d.name == 'KayanHedefSurucusu'), None)
check("KayanHedefSurucusu bulundu", _SINIF is not None, True)

def _uye(ad):
    if _SINIF is None:
        return None
    return next((d for d in _SINIF.body
                 if isinstance(d, _ast.FunctionDef) and d.name == ad), None)

_sur = _uye('sur')
check("kayan sürüş döngüsü bulundu", _sur is not None, True)
if _sur:
    _cagri = [n for n in _ast.walk(_sur)
              if isinstance(n, _ast.Call) and getattr(n.func, 'id', '') == 'olcum_bayat_mi']
    check("sürüş döngüsünde İKİ bayatlama kapısı var", len(_cagri), 2)
    # Argümanları ayrışmalı: aynı sabiti iki kez geçirmek, iki kapıyı tek
    # akışa bağlamak olurdu.
    _sabitler = {getattr(c.args[2], 'id', None) for c in _cagri if len(c.args) == 3}
    check("iki kapı ayrı sabitlere bağlı",
          _sabitler, {'KAYAN_ODOM_BAYATLAMA_S', 'KAYAN_TARAMA_BAYATLAMA_S'})

# Kapı, YALNIZ geri çağrının bastığı bir alanı okuyor. Damga basılmazsa alan
# sonsuza kadar 0.0 kalır ve kapı her aşamada haksız yere kapanır — araç hiç
# sürmez. Kapının varlığını test etmek, damganın varlığını test etmeden eksik.
def _damga_basiyor_mu(geri_cagri, alan):
    _f = _uye(geri_cagri)
    if _f is None:
        return False
    return any(isinstance(h, _ast.Attribute) and h.attr == alan
               for n in _ast.walk(_f) if isinstance(n, _ast.Assign)
               for h in n.targets)

check("tarama geri çağrısı zaman damgası basıyor",
      _damga_basiyor_mu('_on_scan', '_scan_zaman'), True)
check("odometri geri çağrısı zaman damgası basıyor",
      _damga_basiyor_mu('_on_odom', '_odom_zaman'), True)

# Ölü bandın SAF FONKSİYONDA doğru olması yetmiyor: `_yol`'a ham adımı ekleyen
# bir satır fonksiyonu devre dışı bırakır ve fonksiyonun kendi testleri yeşil
# kalır. Birikimin o fonksiyondan GEÇTİĞİ ayrıca kilitleniyor.
_odom = _uye('_on_odom')
_yol_yazan = [n for n in _ast.walk(_odom or _ast.Module(body=[], type_ignores=[]))
              if isinstance(n, (_ast.AugAssign, _ast.Assign))
              and any(isinstance(h, _ast.Attribute) and h.attr == '_yol'
                      for h in ([n.target] if isinstance(n, _ast.AugAssign) else n.targets))]
check("_yol'a yazan tek satır var", len(_yol_yazan), 1)
check("yol birikimi ölü banttan geçiyor",
      any(getattr(c.func, 'id', '') == 'yol_artimi'
          for n in _yol_yazan for c in _ast.walk(n.value)
          if isinstance(c, _ast.Call)), True)

# ④ Yol artımı: gürültü elenirken gerçek hareket ELENMEMELİ.
_DT = 1.0 / 50.0                      # EKF 50 Hz (ekf.yaml frequency)
_GERCEK = 0.65 * _DT                  # nominal hızda bir örnekteki yer değiştirme
check("gerçek hareket yola sayılıyor",
      yol_artimi(_GERCEK, _DT, KAYAN_YOL_TABAN_HIZ_MS), _GERCEK)
# Aracın yerinden kalkabildiği ölçülmüş en düşük hız 0.45 m/s; eşik bunu
# elerse otonom kalkış sessizce mesafe saymaz.
check("kalkış hızı yola sayılıyor",
      yol_artimi(0.45 * _DT, _DT, KAYAN_YOL_TABAN_HIZ_MS) > 0.0, True)
check("gürültü yola sayılmıyor",
      yol_artimi(0.0005, _DT, KAYAN_YOL_TABAN_HIZ_MS), 0.0)
# Eşik iki sınır arasında olmalı. ALT sınır ilk denemede ihlal edilmişti:
# 0.05 m/s, 50 Hz'de 1 mm titremeyle TAM eşit çıkıyor ve hiçbir şey elenmiyor.
check("eşik 1 mm titremeyi eliyor (alt sınır)",
      KAYAN_YOL_TABAN_HIZ_MS > 0.001 / _DT, True)
# ÜST sınır: kalkış hızının altında kalmalı, yoksa otonom kalkış hiç sayılmaz.
check("eşik kalkış hızının belirgin altında (üst sınır)",
      KAYAN_YOL_TABAN_HIZ_MS <= 0.45 / 2.0, True)
# Bölme tanımsız: bir örnek atlamak, sonsuz bir artım eklemekten ucuz.
check("dt sıfırken artım yok",  yol_artimi(0.009,  0.0, KAYAN_YOL_TABAN_HIZ_MS), 0.0)
check("dt negatifken artım yok", yol_artimi(0.009, -0.1, KAYAN_YOL_TABAN_HIZ_MS), 0.0)

# ⑤ Kusurun kendisi: DURAN araçta 60 saniye. Gürültü mutlak değer olarak
# toplandığı için düzeltme olmadan metrelerce "yol" birikiyordu; aşamanın
# bitiş ölçütü tam olarak bu sayı.
_ORNEK, _JITTER = 50 * 60, 0.002   # örnek başına 2 mm ≙ 0.1 m/s
_duzeltmesiz = _ORNEK * _JITTER
_duzeltmeli  = sum(yol_artimi(_JITTER, _DT, KAYAN_YOL_TABAN_HIZ_MS)
                   for _ in range(_ORNEK))
check("düzeltmesiz duran araç metrelerce yol biriktiriyordu",
      _duzeltmesiz > 2.0, True)
check("duran araçta yol birikmiyor", _duzeltmeli, 0.0)
# Ve hareket eden araçta ölçüt bozulmuyor: 20 m'lik aşama 20 m okumalı.
_hareketli = sum(yol_artimi(_GERCEK, _DT, KAYAN_YOL_TABAN_HIZ_MS)
                 for _ in range(int(20.0 / _GERCEK)))
check("hareket eden araçta yol korunuyor", round(_hareketli, 1), 20.0)


# ─── Sıkışma: Nav2 kurtarmalarına yer açmak ────────────────────────────────
print("\n=== Kayan hedef bastırma ve hedef ömrü ===")

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    hedef_yeniden_gonderilsin_mi,
)
from teknofest_ika.otonomi.topics import KAYAN_HEDEF_OLU_BANT_M  # noqa: E402

# ① Bastırma yalnız "etkin hedef VAR ve hedef kaymadı" hâlinde.
check("ilk hedef her zaman gönderilir",
      hedef_yeniden_gonderilsin_mi((1.0, 0.0), None, True, 0.25), True)
# Nav2 hedefi abort etmiş olabilir; bastırma o durumda aracı hedefsiz
# bekletirdi ve bunu kimse söylemezdi.
check("etkin hedef yokken kayma aranmaz",
      hedef_yeniden_gonderilsin_mi((1.0, 0.0), (1.0, 0.0), False, 0.25), True)
check("kaymayan hedef yeniden gönderilmez",
      hedef_yeniden_gonderilsin_mi((1.05, 0.0), (1.0, 0.0), True, 0.25), False)
check("kayan hedef gönderilir",
      hedef_yeniden_gonderilsin_mi((1.30, 0.0), (1.0, 0.0), True, 0.25), True)
# Eşiğin kendisi gönderim tarafında: sınırda susmak, tam eşik kadar kaymış
# gerçek bir hareketi bastırmak olurdu.
check("tam eşikteki kayma gönderilir",
      hedef_yeniden_gonderilsin_mi((1.25, 0.0), (1.0, 0.0), True, 0.25), True)
# Kayma düzlemde, tek eksende değil.
check("çapraz kayma da sayılıyor",
      hedef_yeniden_gonderilsin_mi((1.20, 0.20), (1.0, 0.0), True, 0.25), True)

# ② Ölü bant iki uç arasında sıkışmalı.
# ÜST — aracın kalkabildiği en düşük hız (0.45 m/s) bir periyotta hedefi bu
# kadar kaydırır; eşik onu bastırırsa GERÇEK hareket duruyor sanılır.
_EN_YAVAS_KAYMA = 0.45 * KAYAN_HEDEF_PERIYOT_S
check("ölü bant gerçek hareketi bastırmıyor",
      KAYAN_HEDEF_OLU_BANT_M <= _EN_YAVAS_KAYMA / 2.0, True)
# ALT — santimetrelik LiDAR/EKF oynamasının üstünde olmalı, yoksa duran
# araçta da gönderim sürer ve düzeltme hiçbir işe yaramaz.
check("ölü bant gürültünün üstünde", KAYAN_HEDEF_OLU_BANT_M >= 0.10, True)

# ③ Hedefin bittiğini fark etme — preemption yarışı dahil.
# Nav2Client ROS düğümü istiyor; burada yalnız handle yaşam döngüsü
# denetleniyor, o yüzden nesne __init__'siz kuruluyor.
from teknofest_ika.otonomi.misyon_fsm import Nav2Client  # noqa: E402


class _SahteLog:
    def warn(self, *a, **k):  pass
    def info(self, *a, **k):  pass
    def error(self, *a, **k): pass


class _SahteNode:
    def get_logger(self):
        return _SahteLog()


class _SahteSonuc:
    def __init__(self, durum):
        self._durum = durum

    def result(self):
        class _R:
            status = self._durum
        return _R()


def _istemci():
    c = Nav2Client.__new__(Nav2Client)
    c.node = _SahteNode()
    c._son_handle = None
    return c


_c = _istemci()
check("hedefsiz istemcide etkin hedef yok", _c.etkin_hedef_var(), False)
_h1 = object()
_c._son_handle = _h1
check("hedef varken etkin hedef bildiriliyor", _c.etkin_hedef_var(), True)

# Abort (status 6) etkin hedefi düşürmeli: düşmezse bastırma aracı sonsuza
# kadar hedefsiz bekletir.
_c._hedef_bitti(_SahteSonuc(6), _h1)
check("abort edilen hedef etkin sayılmıyor", _c.etkin_hedef_var(), False)

# 🔑 Preemption yarışı: yeni hedef kabul edildikten SONRA eskisinin sonucu
# geliyor. Körlemesine temizlemek, az önce kabul edilen hedefi yok saymak
# olurdu ve araç her preemption'da bir periyot boyunca hedefsiz görünürdü.
_c2 = _istemci()
_eski, _yeni = object(), object()
_c2._son_handle = _yeni
_c2._hedef_bitti(_SahteSonuc(6), _eski)
check("bayat handle'ın bitişi yeni hedefi düşürmüyor", _c2.etkin_hedef_var(), True)

# Sonuç okunamasa bile (future patladı) etkin hedef düşmeli — okunamayan bir
# sonuç, hedefin sürdüğünün kanıtı değil.
class _PatlayanSonuc:
    def result(self):
        raise RuntimeError('kırık future')


_c3 = _istemci()
_h3 = object()
_c3._son_handle = _h3
_c3._hedef_bitti(_PatlayanSonuc(), _h3)
check("okunamayan sonuçta hedef etkin kalmıyor", _c3.etkin_hedef_var(), False)

# ④ Yapısal bağ: bastırma kapısı gerçekten gönderimin ÖNÜNDE olmalı ve
# sonuç aboneliği kurulmalı. Saf fonksiyonun doğru olması, çağrıldığını
# göstermiyor (bu ders bu depoda daha önce çıktı).
_sur_g = _uye('sur')
_gonder = [n for n in _ast.walk(_sur_g or _ast.Module(body=[], type_ignores=[]))
           if isinstance(n, _ast.Call)
           and getattr(n.func, 'attr', '') == 'hedef_gonder']
check("sürüş döngüsünde hedef gönderimi var", len(_gonder) >= 1, True)

_korunan = []
_karsilastirma_yenilenir = False
for _dugum in _ast.walk(_sur_g or _ast.Module(body=[], type_ignores=[])):
    if not isinstance(_dugum, _ast.If):
        continue
    if not any(getattr(c.func, 'id', '') == 'hedef_yeniden_gonderilsin_mi'
               for c in _ast.walk(_dugum.test) if isinstance(c, _ast.Call)):
        continue
    _korunan += [n for b in _dugum.body for n in _ast.walk(b)
                 if isinstance(n, _ast.Call)
                 and getattr(n.func, 'attr', '') == 'hedef_gonder']
    _karsilastirma_yenilenir = _karsilastirma_yenilenir or any(
        isinstance(h, _ast.Name) and h.id == 'son_gonderilen'
        for b in _dugum.body for n in _ast.walk(b)
        if isinstance(n, _ast.Assign) for h in n.targets)
check("her gönderim bastırma kapısının içinde",
      len(_korunan), len(_gonder))
# Karşılaştırma noktası gönderimde yenilenmezse `son_gonderilen` sonsuza
# kadar None kalır, kapı hep True döner ve bastırma HİÇ devreye girmez —
# düzeltme sessizce etkisizleşir. Mutasyon turu bu deliği açtı.
check("gönderimde karşılaştırma noktası yenileniyor",
      _karsilastirma_yenilenir, True)

# `_uye` KayanHedefSurucusu'na bağlı; bu geri çağrı Nav2Client'ta. Sınıf
# sınırını atlamak, dosyada aynı adı taşıyan başka bir üyeyi denetlemek olurdu.
_NAV2C = next((d for d in _ast.walk(_AGAC)
               if isinstance(d, _ast.ClassDef) and d.name == 'Nav2Client'), None)
check("Nav2Client bulundu", _NAV2C is not None, True)
_kabul = next((d for d in (_NAV2C.body if _NAV2C else [])
               if isinstance(d, _ast.FunctionDef)
               and d.name == '_hedef_kabul_edildi'), None)
check("hedef sonucu dinleniyor",
      any(getattr(n.func, 'attr', '') == 'get_result_async'
          for n in _ast.walk(_kabul or _ast.Module(body=[], type_ignores=[]))
          if isinstance(n, _ast.Call)), True)


# ─── Kat edilen yol sayacı aşamaya ait ─────────────────────────────────────
print("\n=== Aşama yol sayacının sahipliği ===")

# Sayaç `sur()`'un girişinde sıfırlanıyordu. `sur()` AYNI aşama için birden
# çok kez çağrılıyor — STOP (aşama başına 3 kez) ve hata kurtarma — yani her
# çağrı aşamayı BAŞTAN başlatıyordu. 22 metrenin 20'sini gitmiş bir araç
# timeout olup kurtarmadan döndüğünde 22 metreyi yeniden sürer ve parkurun
# geri kalanı 20 m kayardı.
from teknofest_ika.otonomi.misyon_fsm import KayanHedefSurucusu  # noqa: E402


def _surucu():
    k = KayanHedefSurucusu.__new__(KayanHedefSurucusu)
    k._lock      = __import__('threading').Lock()
    k._yol       = 0.0
    k._konum     = None
    k._asama_no  = None
    return k


_k = _surucu()
_k.asama_basla(0)
_k._yol = 12.5                      # aşamanın 12,5 metresi gidildi

# STOP ve kurtarma aynı aşama numarasıyla geri geliyor: sayaç KORUNMALI.
_k.asama_basla(0)
check("aynı aşamaya dönüşte yol korunuyor", _k._yol, 12.5)

# Yeni aşama: sıfırlanmalı, yoksa bir sonraki istasyon erken biter.
_k.asama_basla(1)
check("yeni aşamada yol sıfırlanıyor",      _k._yol, 0.0)

# Aşama atlanırsa (vazgeçilen aşama) numara sıçrar — yine sıfırlanmalı.
_k._yol = 3.0
_k.asama_basla(4)
check("atlanan aşamada da sıfırlanıyor",    _k._yol, 0.0)
# İlk çağrı: sayaç hiç kurulmamışken de sıfırlama yapılmalı.
_k2 = _surucu()
_k2._yol = 9.9
_k2.asama_basla(0)
check("ilk aşamada sıfırlanıyor",           _k2._yol, 0.0)

# ── Yapısal: sıfırlama artık sürüş çağrısında OLMAMALI ─────────────────────
_sur_y = _uye('sur')
_yol_sifirlayan = [
    n for n in _ASTM.walk(_sur_y or _ASTM.Module(body=[], type_ignores=[]))
    if isinstance(n, _ASTM.Assign)
    and any(isinstance(h, _ASTM.Attribute) and h.attr == '_yol' for h in n.targets)
]
check("sürüş çağrısı yol sayacını sıfırlamıyor", len(_yol_sifirlayan), 0)

# ── Yapısal: çağrı DÖNGÜNÜN DIŞINDA olmalı ─────────────────────────────────
# Döngünün içine alınırsa STOP'tan sonraki `continue` sayacı yine sıfırlar ve
# düzeltme sessizce etkisizleşir — kilitlenmesi gereken asıl şey bu.
_exec = next((d for d in _ast.walk(_AGAC)
              if isinstance(d, _ast.ClassDef) and d.name == 'NavigateState'), None)
_exec = next((d for d in (_exec.body if _exec else [])
              if isinstance(d, _ast.FunctionDef) and d.name == 'execute'), None)
check("NavigateState.execute bulundu", _exec is not None, True)


def _asama_basla_cagrilari(dugum):
    return [n for n in _ASTM.walk(dugum) if isinstance(n, _ASTM.Call)
            and getattr(n.func, 'attr', '') == 'asama_basla']


_hepsi   = _asama_basla_cagrilari(_exec) if _exec else []
_dongude = [c for d in _ASTM.walk(_exec or _ASTM.Module(body=[], type_ignores=[]))
            if isinstance(d, (_ASTM.While, _ASTM.For))
            for c in _asama_basla_cagrilari(d)]
check("aşama başlangıcı bir kez çağrılıyor", len(_hepsi), 1)
check("çağrı sürüş döngüsünün dışında",      len(_dongude), 0)


# ─── A: hedef ileri yönde ve tek yayla gidilebilir olmalı ──────────────────
print("\n=== Hedef erişilebilirliği ===")

from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    hedef_ulasilabilir_mi, kayan_hedef as _kayan_hedef_fn, ic_duvar_hedefi,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    KAYAN_HEDEF_MIN_ILERI_M, IC_DUVAR_MIN_ILERI_M,
    PLANLAYICI_DONUS_YARICAPI_M,
)

_RMIN = PLANLAYICI_DONUS_YARICAPI_M
_MINI = KAYAN_HEDEF_MIN_ILERI_M

# R_min, L ve δ_max'tan TÜRETİLİR ama nav2_params'a elle yazılıyor. L
# değişince orada unutulursa kapı ile planlayıcı ayrışır ve kapı,
# planlayıcının çözemeyeceği bir hedefi geçirir.
check("R_min = L / tan(δ_max)",
      abs(_RMIN - 1.44 / math.tan(0.5236)) < 0.01, True)

# ① ARKADAKİ hedef elenmeli. `hypot` ile ölçen bir kapı işaretsizdir ve aracın
# 2,5 m gerisindeki bir noktayı geçirir. Planlayıcı REEDS_SHEPP olduğu için
# oraya bir yol ÜRETİLEBİLİR — ama araç arkadan kör, o yol görülmemiş alandan
# geçer. Geri yay sıkışma kurtarması için açık, hedef üretimi için değil.
check("arkadaki hedef eleniyor",
      hedef_ulasilabilir_mi((-3.0, 0.0, 0.0), _MINI, _RMIN), False)
check("hafif arkadaki hedef de eleniyor",
      hedef_ulasilabilir_mi((-0.5, 2.5, 0.0), _MINI, _RMIN), False)
check("tam yandaki hedef eleniyor",
      hedef_ulasilabilir_mi((0.0, 3.0, 0.0), _MINI, _RMIN), False)
check("düz ileri hedef geçiyor",
      hedef_ulasilabilir_mi((3.0, 0.0, 0.0), _MINI, _RMIN), True)
check("çok yakın hedef eleniyor",
      hedef_ulasilabilir_mi((1.0, 0.0, 0.0), _MINI, _RMIN), False)

# ② Dönüş yarıçapı ölçütü: R = (x²+y²)/(2|y|) — çemberin orijinde x eksenine
# teğet olmasından çıkan kapalı çözüm, yaklaşım değil. İki yönde de simetrik.
# Sınır noktaları _RMIN'den TÜRETİLİYOR, elle yazılmıyor: R_min dingil arası
# ya da δ_max ölçülünce değişiyor ve sabit koordinatlı bir vaka sessizce
# anlamını kaybeder (2.42 döneminde (2.0, 1.0) hedefi 8 cm payla geçiyordu,
# 2.49'da pay 1 cm'ye iniyor).
#   x sabit, R = (x² + y²) / (2|y|) → verilen R için y = R − √(R² − x²)
def _y_for_R(x, R):
    return R - math.sqrt(max(0.0, R * R - x * x))


_X = 2.0
_Y_SINIR = _y_for_R(_X, _RMIN)          # tam R_min: kapıda
for _isaret in (1.0, -1.0):
    # Sınırın %20 içi: yarıçap R_min'in üstünde → geçer
    check(f"tek yayla dönülebilen hedef geçiyor ({_isaret:+.0f})",
          hedef_ulasilabilir_mi((_X, 0.80 * _Y_SINIR * _isaret, 0.0),
                                _MINI, _RMIN), True)
    # Sınırın %20 dışı: yarıçap R_min'in altında → elenir
    check(f"tek yayla dönülemeyen hedef eleniyor ({_isaret:+.0f})",
          hedef_ulasilabilir_mi((_X, 1.20 * _Y_SINIR * _isaret, 0.0),
                                _MINI, _RMIN), False)
# Uzak hedefte yarıçap kısıtı gevşiyor — 8 m'de 3 m yanal serbest.
check("uzak hedefte yanal serbestlik artıyor",
      hedef_ulasilabilir_mi((8.0, 3.0, 0.0), _MINI, _RMIN), True)

# ③ Sabitler tek kaynakta olmalı.
# `min_ileri` iki yerde yaşıyor: kapının sabiti ve kayan_hedef'in imza
# varsayılanı. Ayrışırlarsa merkez çizgisi kapının geçireceği bir hedefi
# baştan üretmez (ya da tersi) ve sebebi hiçbir logda görünmez.
import inspect as _ins  # noqa: E402
check("min_ileri kapı ile üreticide aynı",
      _ins.signature(_kayan_hedef_fn).parameters['min_ileri'].default, _MINI)
# Dönüş yarıçapı planlayıcının aradığı sayı olmak zorunda: kapı bu sayıyla
# eliyor, planlayıcı o sayıyla arıyor.
check("dönüş yarıçapı planlayıcıyla aynı",
      _NAV['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'],
      _RMIN)

# ④ Kapı İKİ üretim yoluna birden uygulanmalı. `ic_duvar_hedefi`'nde hiç
# mesafe kapısı yoktu ve duvar noktasını x >= -1.0 ile kabul ediyor.
_uret = _uye('hedef_uret')
check("hedef üretimi bulundu", _uret is not None, True)
_iade = [n for n in _ASTM.walk(_uret or _ASTM.Module(body=[], type_ignores=[]))
         if isinstance(n, _ASTM.Return) and isinstance(n.value, _ASTM.Tuple)
         and not (isinstance(n.value.elts[0], _ASTM.Constant)
                  and n.value.elts[0].value is None)]
check("hedef döndüren iki yol var", len(_iade), 2)
_kapi_cagri = [c for d in _ASTM.walk(_uret or _ASTM.Module(body=[], type_ignores=[]))
               if isinstance(d, _ASTM.If)
               for c in _ASTM.walk(d.test) if isinstance(c, _ASTM.Call)
               and getattr(c.func, 'id', '') == 'hedef_ulasilabilir_mi']
check("her iki yol da kapıdan geçiyor", len(_kapi_cagri), 2)

# ⑤ İki yol AYNI ÖLÇÜTLERLE geçmemeli. Ölçüm: tek ölçüte indirmek U
# dönüşlerinde iç duvar takibini kırıyor (hedefsiz 1→4 ve 0→2).
_tabanlar = sorted(getattr(c.args[1], 'id', None) for c in _kapi_cagri
                   if len(c.args) == 3)
check("iki yol ayrı mesafe tabanı kullanıyor",
      _tabanlar, ['IC_DUVAR_MIN_ILERI_M', 'KAYAN_HEDEF_MIN_ILERI_M'])
# Yay ölçütü YALNIZ merkez çizgisinde: iç duvar çağrısı None geçiyor.
_yaylar = sorted(
    'None' if isinstance(c.args[2], _ASTM.Constant) and c.args[2].value is None
    else getattr(c.args[2], 'id', '?')
    for c in _kapi_cagri if len(c.args) == 3)
check("yay ölçütü yalnız merkez çizgisinde",
      _yaylar, ['None', 'PLANLAYICI_DONUS_YARICAPI_M'])

# ⑥ Yay ölçütü kapatıldığında ÖTEKİ İKİ KAPI DÜŞMEMELİ — kapatma "her şeyi
# kabul et" demek değil.
check("yay kapalıyken arkadaki hedef yine eleniyor",
      hedef_ulasilabilir_mi((-1.0, 0.0, 0.0), 1.2, None), False)
check("yay kapalıyken çok yakın hedef yine eleniyor",
      hedef_ulasilabilir_mi((0.5, 0.0, 0.0), 1.2, None), False)

# ⑦ Regresyonun kendisi kilitleniyor: CAD'de ÖLÇÜLEN gerçek bir U dönüşü
# hedefi. Yay ölçütü iç duvara uygulanırsa bu hedef elenir ve araç dönüşün
# içinde bayat hedefle sürer.
_U_HEDEFI = (1.48, -0.77, 0.0)      # 6→7 sol U, ölçülen: d=1,67 m, açı −27°
check("ölçülen U dönüşü hedefi iç duvar kipinde geçiyor",
      hedef_ulasilabilir_mi(_U_HEDEFI, IC_DUVAR_MIN_ILERI_M, None), True)
check("aynı hedef yay ölçütüne takılıyordu",
      hedef_ulasilabilir_mi(_U_HEDEFI, IC_DUVAR_MIN_ILERI_M,
                            PLANLAYICI_DONUS_YARICAPI_M), False)

# ⑧ İç duvar tabanı iki yönden sınırlı.
# ALT: gövdenin yarısı 0,95 m — bundan yakın hedef aracın AYAK İZİNİN içinde.
check("iç duvar tabanı gövdenin dışında", IC_DUVAR_MIN_ILERI_M >= 0.95, True)
# ÜST: CAD'de ölçülen en yakın iç duvar hedefi 1,32 m; bundan büyük taban
# meşru hedef kaybettirir.
check("iç duvar tabanı ölçülen en yakın hedefin altında",
      IC_DUVAR_MIN_ILERI_M <= 1.32, True)
check("iç duvar tabanı merkez çizgisinden küçük",
      IC_DUVAR_MIN_ILERI_M < KAYAN_HEDEF_MIN_ILERI_M, True)


# ─── B: süre ölçümleri ayarlanabilir saatte olmamalı ───────────────────────
print("\n=== Saat kaynağı ===")

# Jetson'ın RTC'si ölü ve bir ağ bağlantısı belirdiğinde saat günlerce ileri
# atlıyor. time.time() ile ölçülen aralıklar o sıçramada milyonlarca saniye
# okunur: iki bayatlama kapısı, MisyonSaati, STOP cooldown'ı ve hedef periyodu
# TEK bir olayla birden tetiklenir → her aşama 'failed', sonra MISSION_ABORT.
# Ölçüt AST'den okunuyor, dizgeden değil: bu kararın gerekçesi kaynakların
# YORUMLARINDA `time.time()` diye geçiyor ve dizge araması kendi
# dokümantasyonunu ihlal sayıyordu (ilk sürüm dördünü birden düşürdü).
def _saat_cagrilari(yol):
    agac = _ASTM.parse(_kaynak(yol))
    bulunan = set()
    for n in _ASTM.walk(agac):
        if not isinstance(n, _ASTM.Call):
            continue
        f = n.func
        if (isinstance(f, _ASTM.Attribute)
                and getattr(f.value, 'id', '') == 'time'):
            bulunan.add(f.attr)
    return bulunan


for _d in ('teknofest_ika/otonomi/misyon_fsm.py',
           'teknofest_ika/otonomi/mod_yoneticisi.py',
           'teknofest_ika/otonomi/anti_rollback.py',
           'teknofest_ika/utils/pid_controller.py'):
    _cagri = _saat_cagrilari(_d)
    # Karıştırma en kötüsü: bir damga monotonic, karşılaştırması time.time()
    # olursa fark anlamsız çıkar ve hiçbir hata basılmaz. O yüzden ölçüt
    # "monotonic kullanılıyor" değil, "ayarlanabilir saat HİÇ ÇAĞRILMIYOR".
    check(f"{os.path.basename(_d)} ayarlanabilir saat çağırmıyor",
          'time' in _cagri, False)
    check(f"{os.path.basename(_d)} monotonik saat çağırıyor",
          'monotonic' in _cagri, True)

# Koşu saatinin kendisi de aynı kaynakta olmalı — sıçrama §6.12'yi anında
# doldurup koşuyu bitirir.
_saat = next((d for d in _ast.walk(_AGAC)
              if isinstance(d, _ast.ClassDef) and d.name == 'MisyonSaati'), None)
check("MisyonSaati bulundu", _saat is not None, True)
check("koşu saati monotonik",
      all(getattr(n.func, 'attr', '') == 'monotonic'
          for n in _ASTM.walk(_saat or _ASTM.Module(body=[], type_ignores=[]))
          if isinstance(n, _ASTM.Call)
          and getattr(getattr(n.func, 'value', None), 'id', '') == 'time'),
      True)


# ─── Gözcü: ölen düğümü kim geri getirir ───────────────────────────────────
print("\n=== Gözcü (düğüm süpervizyonu) ===")

_BOOT_G = _kaynak('scripts/lydia_startup.sh')

check("gözcü varsayılan açık", _VARSAYILAN.get('GOZCU_AKTIF'), '1')
# Tek turluk keşif sarsıntısına bakıp sağlam düğüm öldürülmesin diye iki tur
# teyit var; periyot ona göre seçilmeli. `ros2 node list` bir keşif sorgusu ve
# ~1 s sürüyor, o yüzden periyot saniyeler mertebesinde.
check("yoklama periyodu makul", 5 <= float(_VARSAYILAN.get('GOZCU_PERIYOT_S', 0)) <= 60, True)
# Sınır olmadan belirleyici bir hata gözcüyü sonsuz döngüye sokar.
check("yeniden başlatma sınırlı", 1 <= int(_VARSAYILAN.get('GOZCU_AZAMI_DENEME', 0)) <= 5, True)

# ── Yeniden başlatma komutu, ilk başlatan komutla AYNI olmalı ──────────────
# preprocessing_node `-r /scan_lidar:=/scan` olmadan yanlış konuya abone olur
# ve hiçbir şey üretmez — hata da basmaz. Komut iki yerde yazılı olduğu için
# argümanlar karşılaştırılıyor.
def _arg_kumesi(metin):
    return set(re.findall(r'(-[rp] [^\s\\]+)', metin))


_m_ilk_pre = re.search(
    r'^ros2 run teknofest_ika preprocessing_node --ros-args(.*?)&$',
    _BOOT_G, flags=re.S | re.M)
_m_goz_pre = re.search(
    r'        preprocessing_node\)\n(.*?)\n            ;;',
    _BOOT_G, flags=re.S)
check("preprocessing ilk başlatma bulundu", _m_ilk_pre is not None, True)
check("preprocessing gözcü dalı bulundu",   _m_goz_pre is not None, True)
if _m_ilk_pre and _m_goz_pre:
    check("preprocessing yeniden başlatma argümanları aynı",
          _arg_kumesi(_m_goz_pre.group(1)), _arg_kumesi(_m_ilk_pre.group(1)))

_m_ilk_pano = re.search(r'^PANO_PORT="\$PANO_PORT" python3 "\$WS/scripts/web_dashboard\.py"'
                        r' --ros-args(.*?)&$', _BOOT_G, flags=re.S | re.M)
_m_goz_pano = re.search(r'        web_dashboard\)\n(.*?)\n            ;;',
                        _BOOT_G, flags=re.S)
check("pano ilk başlatma bulundu", _m_ilk_pano is not None, True)
if _m_ilk_pano and _m_goz_pano:
    check("pano yeniden başlatma argümanları aynı",
          _arg_kumesi(_m_goz_pano.group(1)), _arg_kumesi(_m_ilk_pano.group(1)))

# ── Döngü ÇALIŞTIRILARAK denetleniyor ──────────────────────────────────────
# Dizge araması bir döngünün yönünü ölçemez: koşulu tersine çevrilmiş bir
# gözcüyü de onaylar. Gruplar ve döngü gerçek betikten kesilip bash'te
# koşturuluyor, yalnız `sleep`, `ros2` ve düğüm başlatma gölgeleniyor.
_m_serbest  = re.search(r'(_GOZCU_SERBEST=".*?"\n)',  _BOOT_G, flags=re.S)
_m_dikkatli = re.search(r'(_GOZCU_DIKKATLI=".*?"\n)', _BOOT_G, flags=re.S)
_m_dongu    = re.search(r'(if \[ "\$GOZCU_AKTIF" != "1" \]; then\n.*?\nfi\n)',
                        _BOOT_G, flags=re.S)
check("gözcü grupları bulundu", bool(_m_serbest and _m_dikkatli), True)
check("gözcü döngüsü bulundu",  _m_dongu is not None, True)

if _m_serbest and _m_dikkatli and _m_dongu:
    def _gozcu_kos(senaryo, tur, aktif=1, azami=3,
                   beklenen='preprocessing_node misyon_fsm seri_kopru'):
        """Gözcüyü `tur` kez döndürür; başlatılan düğümlerin izini döner."""
        with tempfile.TemporaryDirectory() as _t:
            betik = (
                f'LOG={_t}\nWS={_t}\nPANO_PORT=8083\n'
                f'GOZCU_AKTIF={aktif}\nGOZCU_PERIYOT_S=0\n'
                f'GOZCU_AZAMI_DENEME={azami}\n'
                f'_BEKLENEN="{beklenen}"\n'
                + _m_serbest.group(1) + _m_dikkatli.group(1) +
                # Düğüm başlatma gölgeleniyor: gerçek `ros2 run` çağrılmadan
                # HANGİ düğümün başlatılacağı ölçülür.
                '_gozcu_baslat() { echo "BASLAT:$1" >> "$LOG/iz"; }\n'
                f'_senaryo() {{\n{senaryo}\n}}\n'
                '_tur=0\n'
                f'sleep() {{ _tur=$((_tur+1)); [ "$_tur" -gt {tur} ] && exit 0;'
                ' _senaryo "$_tur" > "$LOG/canli"; return 0; }\n'
                'ros2() { cat "$LOG/canli" 2>/dev/null; }\n'
                + _m_dongu.group(1))
            r = subprocess.run(['bash', '-c', betik], capture_output=True,
                               text=True)
            try:
                with open(os.path.join(_t, 'iz'), encoding='utf-8') as f:
                    iz = f.read()
            except FileNotFoundError:
                iz = ''
            return iz, r.stdout

    _HEPSI = 'echo "/preprocessing_node"; echo "/misyon_fsm"; echo "/seri_kopru"'
    _EKSIK = 'echo "/misyon_fsm"; echo "/seri_kopru"'   # preprocessing yok

    # ① TEK turluk eksiklik işlem üretmemeli — keşif sarsıntısı olabilir.
    _iz, _ = _gozcu_kos(f'case "$1" in 1) {_EKSIK} ;; *) {_HEPSI} ;; esac', tur=3)
    check("tek turluk eksiklik yeniden başlatma üretmiyor",
          'BASLAT:' in _iz, False)

    # ② İki tur üst üste eksikse başlatılmalı.
    _iz, _ = _gozcu_kos(f'{_EKSIK}', tur=3)
    check("iki tur eksik serbest düğüm başlatılıyor",
          'BASLAT:preprocessing_node' in _iz, True)

    # ③ ASLA grubu başlatılmamalı ama adıyla bildirilmeli — yeniden başlarsa
    #    koşu 0. waypoint'ten başlar ve saat sıfırlanır.
    _iz, _cikti = _gozcu_kos(
        'echo "/preprocessing_node"; echo "/seri_kopru"', tur=3)
    check("misyon_fsm yeniden başlatılmıyor", 'BASLAT:misyon_fsm' in _iz, False)
    check("misyon_fsm ölümü bildiriliyor",    'misyon_fsm' in _cikti, True)

    # ④ DİKKATLİ grubu da başlatılmamalı — portu yeniden açmak kartı
    #    resetleyebilir, ön koşul otomatikleştirilemiyor.
    _iz, _cikti = _gozcu_kos(
        'echo "/preprocessing_node"; echo "/misyon_fsm"', tur=3)
    check("seri_kopru yeniden başlatılmıyor", 'BASLAT:seri_kopru' in _iz, False)
    check("seri_kopru ölümü bildiriliyor",    'seri_kopru' in _cikti, True)

    # ⑤ `ros2 node list` bomboşsa sorun bizdedir: 25 düğümün 25'ini ölmüş
    #    sayıp başlatmak, gözcünün önlediği arızadan kötüdür.
    _iz, _ = _gozcu_kos('echo -n ""', tur=4)
    check("boş düğüm listesinde hiçbir şey başlatılmıyor", 'BASLAT:' in _iz, False)

    # ⑥ Deneme sınırı: sınırdan fazla başlatma olmamalı.
    _iz, _cikti = _gozcu_kos(f'{_EKSIK}', tur=12, azami=2)
    check("yeniden başlatma sınırı uygulanıyor",
          _iz.count('BASLAT:preprocessing_node'), 2)
    check("sınıra ulaşınca vazgeçildiği yazılıyor", 'VAZGEÇİLDİ' in _cikti, True)

    # ⑦ Kapalıyken hiçbir şey yapmamalı (eski davranış).
    _iz, _cikti = _gozcu_kos(f'{_EKSIK}', tur=4, aktif=0)
    check("gözcü kapalıyken başlatma yok", 'BASLAT:' in _iz, False)

# Grupları ayrıştır. Tanım bu bloğun BAŞINDA: aşağıdaki kapsam testleri
# de kullanıyor ve önceki yerleşimde onlardan SONRA geliyordu.
_SERB = set(re.findall(r'[\w]+', _m_serbest.group(1).split('"')[1])) if _m_serbest else set()
_DIKK = set(re.findall(r'[\w]+', _m_dikkatli.group(1).split('"')[1])) if _m_dikkatli else set()

# ── HER beklenen düğüm bir grupta olmalı ───────────────────────────────────
# 🔑 Bu testin sebebi: gözcünün ilk sürümünde 21 beklenen düğümün 7'si hiçbir
# grupta değildi ve sessizce varsayılana (ASLA) düşüyordu. Davranış güvenliydi
# — yanlış yeniden başlatma yok — ama gözcü onları KAPSAMIYORDU ve aralarında
# `ackermann_converter` vardı: ölürse /cmd_vel çevrimi durur ve araç durur.
# Varsayılanın güvenli olması, sınıflandırmanın eksik kalmasını meşru kılmıyor;
# eksiklik ancak sayılırsa görülür.
_GOZCU_ASLA_BEKLENEN = {'misyon_fsm', 'e_stop_node'}
_m_bek_hepsi = re.findall(r'_BEKLENEN="([^"]*)"', _BOOT_G)
_BEK = set()
for _p in _m_bek_hepsi:
    _BEK |= {w for w in _p.replace('$_BEKLENEN', '').split()}
check("beklenen düğüm listesi okundu", len(_BEK) >= 15, True)
_sinifsiz = sorted(_BEK - _SERB - _DIKK - _GOZCU_ASLA_BEKLENEN)
check("sınıflandırılmamış beklenen düğüm yok", _sinifsiz, [])
# ASLA grubu AÇIKÇA yazılı olmalı — "listede yok, demek ki ASLA" bir karar
# değil, karar verilmemişliğin sonucu.
check("ASLA grubu bilinçli seçilmiş",
      sorted(_GOZCU_ASLA_BEKLENEN & _BEK), ['e_stop_node', 'misyon_fsm'])

# ── Listelerde ÖLÜ girdi olmamalı ──────────────────────────────────────────
# İlk sürümde DİKKATLİ listesinde nav2'nin altı düğümü vardı ama `_BEKLENEN`'de
# yoklardı — hiç eşleşmeyen, kural yazdığımı sandığım ölü girdiler.
check("SERBEST listesinde ölü girdi yok", sorted(_SERB - _BEK), [])
check("DİKKATLİ listesinde ölü girdi yok", sorted(_DIKK - _BEK), [])

# ── Ölmesi aracı durduran düğümler kapsam DIŞINDA kalmamalı ────────────────
for _n in ('ackermann_converter', 'anti_rollback', 'seri_kopru'):
    check(f"{_n} gözcü kapsamında", _n in (_SERB | _DIKK), True)


# ── Gruplandırılmamış düğüm ASLA sayılmalı ─────────────────────────────────
# Yeni bir düğüm eklenip gruplandırılması unutulursa varsayılan davranış
# "dokunma" olmalı; "başlat" olursa unutulan bir düğüm sessizce riskli hâle
# gelir. Serbest ve dikkatli listelerin KESİŞİMİ de boş olmalı.
check("serbest ve dikkatli listeler ayrışık", sorted(_SERB & _DIKK), [])
# E-STOP ve görev durumu hiçbir koşulda serbest olmamalı.
for _n in ('misyon_fsm', 'e_stop_node'):
    check(f"{_n} serbest grupta değil", _n in _SERB, False)
    check(f"{_n} dikkatli grupta da değil", _n in _DIKK, False)



# ─── 20. Kip titremesi, kesme tekrarı ve manuel kip kapısı ──────────────────
print("\n=== 20. Kip, kesme ve manuel kip ===")

_MODY = _kaynak('teknofest_ika/otonomi/mod_yoneticisi.py')
_SERI = _kaynak('teknofest_ika/gomulu/seri_kopru.py')


def _sabit(kaynak, ad):
    """Modül düzeyindeki sayısal sabiti AST'den okur."""
    for d in ast.parse(kaynak).body:
        if isinstance(d, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == ad for t in d.targets):
            return ast.literal_eval(d.value)
    return None


# Kip bayatlama eşiği köprünün TEKRAR periyoduna bağlı. Eşik periyodun altına
# inerse kart kip 2'de sabitken bile mod her tekrar arasında manuele düşer ve
# her geri dönüşte /mission_start basılır — FSM baştan başlar.
_KIP_T   = _sabit(_MODY, 'KIP_TIMEOUT_S')
_KIP_P   = _sabit(_MODY, 'KIP_TEKRAR_PERIYOT_S')
_TAZELE  = _sabit(_SERI, 'DURUM_TAZELEME_S')
check("kip tekrar periyodu köprüyle aynı", _KIP_P, _TAZELE)
check("kip eşiği tekrar periyodunun en az 2 katı", _KIP_T >= 2.0 * _KIP_P, True)

# Kesme kaynağı PERİYODİK yayınlanmalı. Yalnız değişimde basılırsa açılışta
# e_stop_node'dan önce çıkar ve kimse duymaz.
check("kesme tekrar periyodu tanımlı", _sabit(_MODY, 'KESME_TEKRAR_S') is not None, True)
check("kesme periyodik yayınlanıyor",
      'self.create_timer(KESME_TEKRAR_S, self._kesme_yayinla)' in _MODY, True)
check("kesme yayını değişim dalında değil",
      '_kesme_pub.publish' in _MODY.split('def _kesme_yayinla')[0].split('def _durum_cb')[-1],
      False)

# Manuel kipte uçuştaki Nav2 hedefi iptal edilmeli. Kart komutu zaten yok
# sayıyor, ama iptal edilmeyen hedef ayakta kalır ve SwC geri otonoma
# alındığında araç bayat bir hedefe kalkar.
_FSM = _kaynak('teknofest_ika/otonomi/misyon_fsm.py')
check("go_to manuel kapısı alıyor",   'manuel_check_fn' in _FSM, True)
check("kayan sürüş manuel kapısı alıyor", 'manuel_fn' in _FSM, True)
check("manuel kapısı hedefi iptal ediyor",
      "manuel_check_fn()" in _FSM and "return 'manuel_mod'" in _FSM, True)
check("NavigateState manuel sonucu işliyor", "result == 'manuel_mod'" in _FSM, True)
# C4: görev kaldığı yerden sürer — manuel dalı aşama indeksini ilerletmiyor
# ve bir sonuç döndürmüyor, aynı aşamaya `continue` ile dönüyor.
_MANUEL_DAL = _FSM.split("result == 'manuel_mod'")[1].split('continue')[0]
check("manuel dalı aşamayı bitirmiyor", 'return' in _MANUEL_DAL.replace(
    "return 'e_stop'", ''), False)
check("manuel dalı wp_index'e dokunmuyor", 'wp_index' in _MANUEL_DAL, False)

# Manuelde 0x38 karşılaştırması sahte alarm üretir: kart Jetson komutunu yok
# sayıp kumandadan geleni geri yolluyor.
_GB = _SERI.split('def _geri_bildirim')[1].split('def ')[0]
check("komut karşılaştırması otonom kiple sınırlı", 'KART_KIP_OTONOM' in _GB, True)


# ─── 21. Kart canlılığı, kamera konuları ve planlayıcı ──────────────────────
print("\n=== 21. Canlılık, kamera, planlayıcı ===")

# /kart/durum|hata|kip önbellekten tekrarlanıyor: hat çöp okurken de akarlar.
# Canlılık ölçütü yalnız gerçek çerçeveden doğan /kart/surus olabilir.
_WD = _kaynak('teknofest_ika/otonomi/watchdog.py')
# Denetim İZLENEN SÖZLÜĞE bakmalı, dosyanın tamamına değil: ad mesaj tipi
# haritasında da geçiyor ve orada kalması izlendiği anlamına gelmez.
_TEMEL = _WD.split('TEMEL_TOPICLER = {')[1].split('\n}')[0]
check("watchdog kart canlılığını /kart/surus'tan ölçüyor",
      'KART_SURUS_TOPIC:' in _TEMEL, True)
check("izlenen konunun mesaj tipi tanımlı",
      'KART_SURUS_TOPIC:' in _WD.split('_TOPIC_MSG_TYPE = {')[1].split('\n}')[0], True)
for _tekrarli in ('KART_DURUM_TOPIC', 'KART_HATA_TOPIC', 'KART_KIP_TOPIC'):
    check(f"watchdog {_tekrarli} ile canlılık ölçmüyor", _tekrarli in _WD, False)

# Arka kamera konusu, yayıncısı olan kamera gözcüsüyle eşleşmeli.
from teknofest_ika.otonomi.topics import CAMERA_REAR_TOPIC as _REAR  # noqa: E402
check("arka kamera konusu", _REAR, '/camera/arka/image_raw')
check("ölü ön kamera sabiti kalmadı",
      'CAMERA_FRONT_TOPIC' in _kaynak('teknofest_ika/otonomi/topics.py'), False)
for _y in ('scripts/lydia_startup.sh', 'launch/gercek_arac.launch.py'):
    check(f"{_y}: arka kamera konusu tek ad",
          '/camera/rear/' in _kaynak(_y), False)

# Planlayıcı geri yayı ÜRETEBİLMELİ (dar koridorda DUBIN plan bulamıyor), ama
# araç arkadan kör olduğu için geri gitmek pahalı kalmalı.
_GB_PARAM = _nav2_params()['planner_server']['ros__parameters']['GridBased']
check("planlayıcı REEDS_SHEPP", _GB_PARAM['motion_model_for_search'], 'REEDS_SHEPP')
check("geri yay açık", _GB_PARAM['allow_reverse_expansion'], True)
check("geri gitmek cezalı", _GB_PARAM['reverse_penalty'] >= 5.0, True)

# Kayan hedef kapısı geri manevraya GÜVENMEZ: hedef ileride olmak zorunda.
check("arkadaki hedef REEDS_SHEPP'te de eleniyor",
      hedef_ulasilabilir_mi((-3.0, 0.0, 0.0), _MINI, _RMIN), False)

# Açılış: iki okuyucu hattı bozuyor, telemetri varsayılan kapalı.
_ST = _kaynak('scripts/lydia_startup.sh')
check("telemetri varsayılanı kapalı", ': "${F767_TELEMETRI_AKTIF:=0}"' in _ST, True)
# autostart sabit zaman aşımıyla düşerse bringup bir daha denenmiyor.
check("nav2 lifecycle doğrulaması var", '_nav2_aktif_mi' in _ST, True)
check("nav2 doğrulaması bt_navigator'a bakıyor",
      'bt_navigator controller_server' in _ST, True)



# ─── 22. Görev tetiği, çerçeve ve yağmur onarımı ───────────────────────────
print("\n=== 22. Tetik, çerçeve, yağmur ===")

_FSM22 = _kaynak('teknofest_ika/otonomi/misyon_fsm.py')
_PRE22 = _kaynak('teknofest_ika/gorsel/preprocessing_node.py')
from teknofest_ika.otonomi.pure_logic import yagmur_lekeleri  # noqa: E402

# ① IDLE SEVİYE TETİĞİ. /mission_start tek atışlık bir kenar ve VOLATILE:
# mod_yoneticisi açılış betiğinde misyon_fsm'den dakikalarca önce başlıyor,
# yani SwC açılışta otonomdaysa kenar bu düğüm doğmadan çıkıp kayboluyor ve
# FSM sonsuza kadar IDLE'da bekliyor. /mod/aktif 1 Hz tekrarlanan bir SEVİYE
# sinyali; doğuş anından bağımsız olarak geçerli kipi veriyor.
_IDLE = _FSM22.split('class IdleState')[1].split('\nclass ')[0]
check("IdleState /mod/aktif dinliyor", 'MOD_AKTIF_TOPIC' in _IDLE, True)
check("IdleState seviye tetiğiyle çıkıyor", '_mod_otonom' in _IDLE, True)
check("bekleme koşulu iki tetiği de içeriyor",
      '(self._start_received or self._mod_otonom)' in _IDLE, True)
# Kip HİÇ görülmemişken çıkmamalı: "manuel değil" ile "otonom" aynı şey değil.
check("kip görülmeden başlangıç yok", 'self._mod_otonom     = False' in _IDLE, True)
check("tetik FULL_AUTO ile karşılaştırılıyor", 'MOD_FULL_AUTO' in _IDLE, True)
# Elle tetik yolu korunuyor (pano/terminal/test).
check("elle tetik duruyor", 'MISSION_START_TOPIC' in _IDLE, True)

# ⑥ HEDEF ÇERÇEVESİ. bt_navigator hedefi kendi global çerçevesine çevirmek
# zorunda; SLAM kapalıyken `map`'i basan kimse yok, yani orada doğan bir
# hedef hiçbir zaman çözülemez ve yalnız bir TF hatası bırakır.
check("Nav2 global çerçevesi nav2_params ile aynı",
      NAV2_GLOBAL_FRAME, _nav2_params()['bt_navigator']['ros__parameters']['global_frame'])
check("kayan hedef çerçevesi de çözülebilir",
      KAYAN_HEDEF_FRAME in (NAV2_GLOBAL_FRAME, 'odom'), True)
check("hedef kurucusunda elle yazılmış 'map' yok",
      "frame_id: str = 'map'" in _FSM22, False)

# ③ YAĞMUR ONARIMI. Maske bileşen başına TAM GÖRÜNTÜ taramasıyla kurulursa
# parlak bir sahnede kare başına ~16 ms eder (640×480, 376 bileşen); iki
# kamerada 30 Hz'de tek başına bir çekirdek. inpaint'in kendisi de leke
# sayısıyla büyüyor, o yüzden çok lekeli sahnede hiç çağrılmamalı.
check("bileşen başına tam görüntü taraması kalmadı",
      'labels == label_id' in _PRE22, False)
check("seçim pure_logic'te", 'yagmur_lekeleri(' in _PRE22, True)
# Kararı pure_logic veriyor ama uygulayan düğüm: 'onar' dışındaki her
# sebepte görüntüye DOKUNULMADAN dönülmeli, yoksa kapı süslemeye dönüşür.
_RR = _PRE22.split('def _remove_rain')[1].split('\n    def ')[0]
check("düğüm sebebi uyguluyor", "if sebep != 'onar':" in _RR, True)
check("onarım sebep kapısından SONRA",
      "sebep != 'onar'" in _RR
      and _RR.index("sebep != 'onar'") < _RR.index('cv2.inpaint'), True)

# Seçim mantığı DAVRANIŞLA sınanıyor; kaynakta ad aramak, kapının gerçekten
# işlediğini göstermiyor. alanlar[0] arka plandır ve asla seçilmemeli.
#                         arka  küçük  büyük  küçük
_ALANLAR = [500, 50, 300, 10]
check("küçük lekeler seçiliyor",
      yagmur_lekeleri(_ALANLAR, 120, 80), ([1, 3], 'onar'))
check("arka plan hiç seçilmiyor",
      0 in yagmur_lekeleri(_ALANLAR, 10**9, 80)[0], False)
check("büyük leke elenir (gerçek nesne bozulmasın)",
      yagmur_lekeleri([0, 300], 120, 80), ([], 'leke_yok'))
# Adet kapısı: sınırın üstünde HİÇBİR şey onarılmıyor — onarımın maliyeti
# leke sayısıyla büyüdüğü için en pahalı durum tam da yağmur OLMAYAN sahne.
check("sınırdaki adet onarılıyor",
      yagmur_lekeleri([0] + [5] * 80, 120, 80)[1], 'onar')
check("sınırın üstü tamamen atlanıyor",
      yagmur_lekeleri([0] + [5] * 81, 120, 80), ([], 'cok_leke'))
check("çok lekede hiçbir kimlik dönmüyor",
      yagmur_lekeleri([0] + [5] * 400, 120, 80)[0], [])



# ─── 23. Atış kip kapısı ve LiDAR ön koşulu ────────────────────────────────
print("\n=== 23. Atış kapısı, LiDAR ön koşulu ===")

_SERI23 = _kaynak('teknofest_ika/gomulu/seri_kopru.py')
_FSM23  = _kaynak('teknofest_ika/otonomi/misyon_fsm.py')
_ST23   = _kaynak('scripts/lydia_startup.sh')
from teknofest_ika.otonomi.pure_logic import lazer_sonmeli  # noqa: E402

# ④ LAZER KAPISI. Kart manuelde yalnız SÜRÜŞ komutlarını yok sayıyor; lazeri
# de yok sayıp saymadığı bilinmiyor. Kapı köprüde çünkü karta açılan tek kapı
# orası — /shoot_command'a basan her yayıncıyı birlikte kapsıyor.
_SHOOT_CB = _SERI23.split('def _shoot_cb')[1].split('\n    def ')[0]
check("açma otonom kiple sınırlı", 'KART_KIP_OTONOM' in _SHOOT_CB, True)
check("reddetme paket göndermeden dönüyor",
      _SHOOT_CB.index('return') < _SHOOT_CB.index('PKT_J_LAZER'), True)
# Kapatma HER kipte geçmeli: ters kurmak, kip atış sırasında değişince
# lazeri açık bırakırdı.
check("kapatma kipe bağlı değil",
      'if msg.data and self._kip != KART_KIP_OTONOM:' in _SHOOT_CB, True)

# Giriş kapısı tek başına yetmez: istek tek bir True ile açılıp saniyelerce
# açık kalıyor, o pencerede kip değişirse _shoot_cb bir daha çağrılmıyor.
_KIP_DEN = _SERI23.split('def _lazer_kip_denetle')[1].split('\n    def ')[0]
check("periyodik kip denetimi var", 'def _lazer_kip_denetle' in _SERI23, True)
check("denetim lazeri söndürüyor", 'PKT_J_LAZER, 0, 0' in _KIP_DEN, True)
check("karar pure_logic'te", 'lazer_sonmeli(' in _KIP_DEN, True)

# Söndürme kararı DAVRANIŞLA sınanıyor. Kip None olabilir (kart henüz
# bildirmemiştir) ve bilinmeyen kip otonom SAYILMAMALI: ateş yetkisini
# varsayıma dayandırmak, kapının hiç olmamasıyla aynı yere çıkar.
check("otonomda yanan lazere dokunulmuyor", lazer_sonmeli(True,  2, 2), False)
check("manuelde yanan lazer söndürülüyor",  lazer_sonmeli(True,  0, 2), True)
check("boş/dur kipinde de söndürülüyor",    lazer_sonmeli(True,  1, 2), True)
check("kip bilinmiyorsa söndürülüyor",      lazer_sonmeli(True, None, 2), True)
check("zaten sönükse iş yok",               lazer_sonmeli(False, 0, 2), False)
check("denetim güvenlik döngüsüne bağlı",
      'self._lazer_kip_denetle()' in _SERI23.split('def _guvenlik_kontrol')[1][:300], True)

# ShootState kapalı kapıya karşı denemesini yakmamalı: istek basılsaydı onay
# hiç gelmez ve üç hakkın biri 8 s'lik timeout'a giderdi.
_SHOOT_ST = _FSM23.split('class ShootState')[1].split('\nclass ')[0]
check("ShootState manuel kipi görüyor", 'manual_mod' in _SHOOT_ST, True)
check("manuel dalı isteği kapatıyor",
      "Bool(data=False)" in _SHOOT_ST.split('manual_mod')[1][:400], True)

# ⑧ LIDAR ÖN KOŞULU. Cihazın yokluğu zaten yazılıyordu ama satır Nav2
# kararından dakikalarca önce akıp gidiyor; sürücünün üç denemede de
# açılamaması ise hiç iz bırakmıyordu.
check("lidar bayrağı kuruluyor", '_LIDAR_VAR=0' in _ST23, True)
check("bayrak sürücü log'undan doğrulanıyor",
      'grep -q "Lidar has started" "$LOG/lidar.log" && _LIDAR_VAR=1' in _ST23, True)
check("üç deneme de başarısızsa uyarı var",
      'LiDAR sürücüsü üç denemede de açılmadı' in _ST23, True)
check("nav2 kapısı bayrağa bakıyor",
      'if [ "$_LIDAR_VAR" != "1" ]; then' in _ST23, True)
# Uyarı Nav2 blokunun İÇİNDE olmalı; dışında kalırsa Nav2 kapalıyken de basar.
_NAV_BLOK = _ST23.split('if [ "$NAV2_AKTIF" = "1" ]; then')[1]
check("uyarı Nav2 blokunun içinde", '_LIDAR_VAR' in _NAV_BLOK, True)
# Bayat ön koşul yorumu: kayan modda waypoint koordinatları hiç okunmuyor.
check("waypoint ön koşulu harita moduna daraltıldı",
      'YALNIZ `harita` modunda' in _ST23, True)



# ─── 24. Nav2 yeniden başlatma, ölü harita ve derleme tazeliği ─────────────
print("\n=== 24. Nav2 restart, ölü harita, derleme ===")

_ST24 = _kaynak('scripts/lydia_startup.sh')
_SETUP24 = _kaynak('setup.py')
_NAV24 = _nav2_params()

# ── A · Nav2 yeniden başlatma ─────────────────────────────────────────────
# navigation_launch.py düğümleri AYRI SÜREÇ olarak açıyor (use_composition
# varsayılanı False) ve launch'ın kapanışı kademeli: SIGINT → 5 s → SIGTERM
# → 5 s → SIGKILL (launch/actions/execute_local.py varsayılanları). Sabit ve
# kısa bir bekleme, ikinci bringup'ı eskiler çıkarken başlatır.
check("nav2 kapanışı süreye değil duruma bekliyor", '_nav2_ayakta' in _ST24, True)
check("sabit sleep 3 kalmadı",
      'pkill -f navigation_launch.py 2>/dev/null\n            sleep 3' in _ST24, False)
check("bekleme tavanlı", '_bekle" -lt 15' in _ST24, True)
check("tavanı aşarsa adıyla kapatılıyor",
      'adıyla kapatılıyor' in _ST24, True)

# 🔑 Tespit KOMUT SATIRINDAN yapılmamalı: `pgrep -f` çağıranın kendi komut
# satırını da tarar ve desen orada geçtiği an fonksiyon kendini bulur —
# hiç nav2 süreci yokken bile tavan kadar beklenir (denendi, tam bu oldu).
_AYAKTA = _ST24.split('_nav2_ayakta() {')[1].split('}')[0]
check("tespit comm üzerinden (pgrep -f DEĞİL)", '-f ' in _AYAKTA, False)

# Desen ALTERNATİF ALTERNATİF çözülüyor, metinde aranmıyor: 'controller_serv'
# kırpılmamış 'controller_server' içinde de geçer, yani `in` ile bakan bir
# test kırpmanın bozulmasını göremez (denendi, tam bu mutasyon kaçtı).
_desen = re.search(r'pgrep\s+"\^\((.*?)\)\$"', _AYAKTA, re.S)
check("pgrep deseni anchor'lı bulunabiliyor", _desen is not None, True)
_adlar = set(_desen.group(1).split('|')) if _desen else set()
# comm 15 KARAKTERE kırpılıyor (ölçüldü); beklenen küme
# navigation_launch.py'deki sekiz executable'ın kırpılmış hâli.
_BEKLENEN_COMM = {
    'controller_server'[:15], 'planner_server'[:15], 'bt_navigator'[:15],
    'behavior_server'[:15],   'smoother_server'[:15],
    'velocity_smoother'[:15], 'waypoint_follower'[:15],
    'lifecycle_manager'[:15],
}
check("desen tam olarak kırpılmış sekiz ad", _adlar, _BEKLENEN_COMM)
check("hiçbir ad 15 haneyi aşmıyor",
      all(len(a) <= 15 for a in _adlar), True)

# navigation_launch.py SEKİZ düğüm açıyor; temizlik listesi waypoint_follower'ı
# atlarsa betiğin yeniden çalıştırılması geride yönetilen bir düğüm bırakır.
_TEMIZLIK = _ST24.split('for _p in web_dashboard.py')[1].split('; do')[0]
for _n in ('controller_server', 'planner_server', 'bt_navigator',
           'behavior_server', 'smoother_server', 'velocity_smoother',
           'waypoint_follower', 'lifecycle_manager'):
    check(f"temizlik listesinde {_n}", _n in _TEMIZLIK, True)

# ── B1 · Ölü lokalizasyon yapılandırması ──────────────────────────────────
# Üçü de lifecycle_manager'ın node_names listesinde değildi, yani hiç ayağa
# kaldırılmıyorlardı; durmaları "haritayla çalışıyoruz" izlenimi veriyordu.
for _olu in ('amcl', 'map_server', 'map_saver'):
    check(f"{_olu} bloğu kaldırıldı", _olu in _NAV24, False)
check("harita yokluğu gerekçesiyle yazılı",
      'Lokalizasyon YOK' in _kaynak('config/nav2_params.yaml'), True)
# Yönetilen düğüm listesi hiçbirine atıf yapmamalı.
_YONETILEN = _NAV24['lifecycle_manager_navigation']['ros__parameters']['node_names']
for _olu in ('amcl', 'map_server', 'map_saver'):
    check(f"yönetilen listede {_olu} yok", _olu in _YONETILEN, False)
# setup.py'deki maps/worlds glob'ları: dizinler yok, girdiler ölüydü ve
# --symlink-install'ı engellediği kayıtlıydı.
check("setup.py maps glob'u kaldırıldı", "'maps/*'" in _SETUP24, False)
check("setup.py worlds glob'u kaldırıldı", "'worlds/*'" in _SETUP24, False)
# ⚠️ Burada `maps/` dizininin VAR OLMADIĞI sınanmıyordu ve sınanmamalı: depo
# testi çalışma ağacının yerel kalıntılarına bakamaz. Araçta git'in izlemediği
# eski bir `maps/test_harita.pgm` duruyor ve o kontrol orada sahte alarm
# veriyordu — laptop'ta yeşil, araçta kırmızı. Asıl kural setup.py'de glob'un
# kalmaması ve o yukarıdaki iki satırla kilitli.

# ── B3 · Derleme tazeliği ─────────────────────────────────────────────────
# Overlay'in VAR olması yetmiyor: kopya tabanlı kurulumda `git pull` sonrası
# .py değişiklikleri etkisizdir ve betik sessizce ESKİ kodu başlatır.
# Yol tam olarak eşleşmeli: 'colcon_build.rc' kendisi 'colcon_build.rcX'
# içinde de geçer ve substring arayan bir test bozulmayı göremez.
check("derleme tazeliği colcon_build.rc'ye bakıyor",
      'build/teknofest_ika/colcon_build.rc"' in _ST24, True)
check("kurulum kipi ayırt ediliyor", '_KURULUM=symlink' in _ST24 and
      '_KURULUM=kopya' in _ST24, True)
# 🔴 Kip göstergesi EGG-LINK olmalı, build/ içindeki symlink DEĞİL. O symlink
# eski bir --symlink-install denemesinden geride kalıyor ve sonraki düz
# derlemeler silmiyor: araçta Ağustos tarihli bir tanesi duruyor ve kurulum
# kopya. Ona bakan denetim "symlink kipi" sanıp .py bayatlığını hiç uyarmaz,
# yani kontrol en gerekli olduğu yerde susar (araçta ölçüldü).
check("kip göstergesi egg-link", '-name "*.egg-link"' in _ST24, True)
# if TEK SATIR olmalı: _kosul_derinligi satır bazlı sayıyor ve çok satırlı
# bir `if` sayılmazken eşleşen `fi` sayılır, derinlik negatife düşer ve
# "bms_koprusu koşulsuz başlıyor" testi sahte alarm verir (bu oldu).
check("egg-link if'i tek satır", 'if [ -n "$_EGG" ]; then' in _ST24, True)

# Metin aramak yetmiyor: `_EGG=$(: find ...)` gibi bir değişiklik deseni yerinde
# bırakıp tespiti tamamen etkisiz kılıyor (mutasyon bunu kaçırdı). O yüzden
# BETİKTEN ÇIKARILAN satır iki sahte kurulum ağacına karşı KOŞTURULUYOR.
import subprocess as _sp26
import tempfile as _tf26
_EGG_SATIR = [l for l in _ST24.splitlines() if l.strip().startswith('_EGG=')]
check("egg-link tespit satırı tek", len(_EGG_SATIR), 1)
if len(_EGG_SATIR) == 1:
    with _tf26.TemporaryDirectory() as _d26:
        _sp26.run(['mkdir', '-p',
                   f'{_d26}/kopya/install/teknofest_ika/lib/python3.10/site-packages/teknofest_ika',
                   f'{_d26}/sym/install/teknofest_ika/lib/python3.10/site-packages'], check=True)
        open(f'{_d26}/sym/install/teknofest_ika/lib/python3.10/site-packages/'
             'teknofest-ika.egg-link', 'w').close()

        def _kip(kok):
            betik = (f'_WS_KOK="{kok}"\n' + _EGG_SATIR[0].strip() + '\n'
                     'if [ -n "$_EGG" ]; then echo symlink; else echo kopya; fi\n')
            return _sp26.run(['bash', '-c', betik], capture_output=True,
                             text=True).stdout.strip()

        check("koşturuldu: kopya kurulum → kopya",  _kip(f'{_d26}/kopya'), 'kopya')
        check("koşturuldu: symlink kurulum → symlink", _kip(f'{_d26}/sym'), 'symlink')
        check("koşturuldu: kurulum yok → kopya",    _kip(f'{_d26}/yok'), 'kopya')
check("build/ symlink'i ölçüt DEĞİL",
      '-L "$_WS_KOK/build/teknofest_ika/teknofest_ika"' in _ST24, False)
# symlink kipinde .py değişikliği uyarı ÜRETMEMELİ (kaynak zaten canlı),
# yalnız setup.py — console_scripts stub'ları iki kipte de kopya.
_TAZE = _ST24.split('_YAPI_IZI="')[1].split('export ROS_DOMAIN_ID')[0]
check("symlink dalı yalnız setup.py'ye bakıyor",
      '$WS/setup.py" -newer' in _TAZE, True)
check("kopya dalı tüm .py'lere bakıyor",
      "'*.py' -newer" in _TAZE, True)



# ─── 25. BNO055 kalibrasyon baytı ve IMU izleme ────────────────────────────
print("\n=== 25. CALIB_STAT ve IMU izleme ===")

# 🔴 Bayt çipin CALIB_STAT'ı: bit 7-6 sys, 5-4 gyr, 3-2 acc, 1-0 mag.
# `sys<<4 | gyr` diye okunursa `(bayt >> 4) & 0x0F` sys yerine `sys<<2 | gyr`
# verir. Çipin tipik açılış durumu sys=0, gyr=3'te bu 3 çıkar — yani
# kalibrasyonu HİÇ olmayan bir yön EKF'e TAM AĞIRLIKLA girer.
check("sys=0 gyr=3 acc=0 mag=0", calib_stat_coz(0b00_11_00_00), (0, 3, 0, 0))
check("hepsi 3",                 calib_stat_coz(0b11_11_11_11), (3, 3, 3, 3))
check("hepsi 0",                 calib_stat_coz(0b00_00_00_00), (0, 0, 0, 0))
check("yalnız sys",              calib_stat_coz(0b11_00_00_00), (3, 0, 0, 0))
check("yalnız mag",              calib_stat_coz(0b00_00_00_11), (0, 0, 0, 3))
check("acc ve gyr karışmıyor",   calib_stat_coz(0b00_10_01_00), (0, 2, 1, 0))
# Dört alan da 0-3 aralığında kalmalı: kayma varsa burada taşar.
for _b in range(256):
    _p = calib_stat_coz(_b)
    if not all(0 <= v <= 3 for v in _p) or len(_p) != 4:
        check(f"CALIB_STAT {_b} aralık dışı", _p, "0-3 dörtlüsü")
        break
else:
    check("256 baytın hepsi 0-3 dörtlüsü", True, True)
# Yeniden kurulabilmeli: dört alandan bayt geri üretilince aynı değer çıkmalı.
check("çözüm tersine çevrilebilir",
      all(calib_stat_coz((a << 6) | (b << 4) | (c << 2) | d) == (a, b, c, d)
          for a in range(4) for b in range(4) for c in range(4) for d in range(4)),
      True)

# Asıl sonuç: yanlış çözüm EKF'e verilen yaw güvenini 15 kat şişiriyordu.
_B = 0b00_11_00_00          # sys=0, gyr=3 — 11 Eylül'de araçtaki durum
check("doğru çözümde yaw şüpheli", yaw_kovaryansi(calib_stat_coz(_B)[0]), 0.30)
check("eski çözüm güvenilir sanıyordu", yaw_kovaryansi((_B >> 4) & 0x0F), 0.02)

_SK25 = _kaynak('teknofest_ika/gomulu/seri_kopru.py')
check("köprü çözücüyü kullanıyor", 'calib_stat_coz(kalib)' in _SK25, True)
check("elle bit kaydırma kalmadı", '(kalib >> 4) & 0x0F' in _SK25, False)
check("bayat 'sys<<4' yorumu kalmadı", 'sys<<4' in _SK25, False)

# IMU izleme: çip takılıyken sessizlik gerçek arıza ve KENDİLİĞİNDEN DÜZELMEZ
# (kart BNO'yu bir kez bulduktan sonra kablo koparsa yeniden aramıyor).
_ST25 = _kaynak('scripts/lydia_startup.sh')
check("BNO_TAKILI anahtarı var", ': "${BNO_TAKILI:=1}"' in _ST25, True)
check("watchdog'a imu_izle geçiliyor", '-p imu_izle:=' in _ST25, True)
check("imu_izle BNO_TAKILI'ya bağlı",
      '-p imu_izle:="$([ "$BNO_TAKILI" = "1" ]' in _ST25, True)
# 🔑 IMU_GUVENLIK_AKTIF ayrı kalmalı: o düğüm roll eşiğine göre aracı
# durduruyor ve eşiği IMU'nun MONTAJ YÖNÜNE güveniyor — yön sahada
# doğrulanmadı. "Veri akıyor mu" ile "yönüne güvenilir mi" aynı şey değil.
check("imu güvenlik hâlâ varsayılan kapalı",
      ': "${IMU_GUVENLIK_AKTIF:=0}"' in _ST25, True)



# ─── 26. LiDAR portu udev symlink'inde ─────────────────────────────────────
print("\n=== 26. LiDAR portu ===")

_ST26 = _kaynak('scripts/lydia_startup.sh')

# 🔴 ydlidar sürücüsü portu KENDİ yaml'ından okuyor ve orada ham düğüm yazılı
# (`port: /dev/ttyUSB0`). Betiğin `[ -e /dev/lidar ]` kontrolü symlink'e,
# sürücü başka bir şeye bakıyor: ikisi ayrışabilir. ST-LINK ile LiDAR aynı
# hub'ın arkasında olduğundan ttyUSB numarası açılışlar arasında kayabiliyor
# ve sürücü yanlış cihazı açar — belirti yalnız "Lidar has started"ın
# gelmemesi olur, sebebi hiçbir log satırında yazmaz.
check("LIDAR_PORT anahtarı var", ': "${LIDAR_PORT:=/dev/lidar}"' in _ST26, True)
check("varsayılan ham düğüm DEĞİL", 'LIDAR_PORT:=/dev/ttyUSB' in _ST26, False)
check("çalışma anı kopyası üretiliyor",
      'tmini_pro.runtime.yaml' in _ST26, True)
check("kopya LIDAR_PORT ile yazılıyor",
      '\\1$LIDAR_PORT' in _ST26, True)
check("sürücü kopyayı alıyor",
      'params_file:="$_LIDAR_PARAMS"' in _ST26, True)
check("özgün yol doğrudan verilmiyor",
      'params_file:=/home/lydia/lydia_ortam/tmini_pro.yaml' in _ST26, False)
# Dosya yoksa özgün yola düşülmeli: o hâlde davranış eskisi gibi olur,
# betik sessizce portsuz bir kopya üretmemeli.
check("yaml yokken özgün yola düşülüyor",
      '_LIDAR_PARAMS="$_LIDAR_YAML"' in _ST26, True)

# sed deseni yalnız `port:` satırını değiştirmeli; baudrate ve frame_id
# bozulursa sürücü hiç açılmaz ya da TF zinciri kopar.
import re as _re26
_ORNEK = (
    "ydlidar_ros2_driver_node:\n"
    "  ros__parameters:\n"
    "    port: /dev/ttyUSB0\n"
    "    frame_id: laser_frame\n"
    "    baudrate: 230400\n"
    "    fixed_resolution: false\n"
)
_SONUC = _re26.sub(r'^( *port: *).*', r'\g<1>/dev/lidar', _ORNEK, flags=_re26.M)
check("sed: port değişti",        '    port: /dev/lidar' in _SONUC, True)
check("sed: frame_id bozulmadı",  'frame_id: laser_frame' in _SONUC, True)
check("sed: baudrate bozulmadı",  'baudrate: 230400' in _SONUC, True)
check("sed: başka satır değişmedi",
      _SONUC.count('\n'), _ORNEK.count('\n'))
# frame_id, betiğin bastığı statik TF'in child'ıyla aynı olmak zorunda.
check("frame_id statik TF ile aynı", 'base_link laser_frame' in _ST26, True)



# ─── 27. SwB taret anahtarı otonomda gazı kesiyor ──────────────────────────
print("\n=== 27. SwB gaz kesme uyarısı ===")

# 🔴 051 §6.3: kart, SwB yukarıdayken OTONOMDA DA gazı uygulamıyor (taret
# kipi rölanti kümesinde). Fren ve direksiyon etkilenmiyor, yani belirti
# "direksiyon dönüyor ama araç kımıldamıyor" ve sebebi hiçbir logda yoktu.
_SK27 = _kaynak('teknofest_ika/gomulu/seri_kopru.py')
_DRM27 = _SK27.split('def _drm_isle')[1].split('\n    def ')[0]
check("SwB uyarısı var", 'DRM_TARET' in _DRM27, True)
check("uyarı gazı işaret ediyor", 'GAZ YOK' in _DRM27, True)
# Uyarı yalnız OTONOM kipte basılmalı: manuelde SwB'yi kaldırmak normal.
check("uyarı otonom kiple sınırlı", 'KART_KIP_OTONOM' in _DRM27, True)
check("uyarı throttle'lı", 'throttle_duration_sec' in _DRM27, True)


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
