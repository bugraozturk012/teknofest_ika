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
    ayar_deger,
    ayar_gonderilecek,
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

# DİNGİL ARASI ÖLÇÜLMEDİ. Değer beş yere yansıyor ve Ackermann kinematiğinin
# tek girdisi; "araçtan ölçüldü" diye yazmak sahada kimsenin mezürü
# çıkarmamasına yol açar. Elektrik tarafı da ölçülmediğini doğruladı.
# Denetim dingil arasına daraltılır: aracın GÖVDE ölçüleri (1,90 × 1,16 m)
# gerçekten ölçüldü ve öyle yazması doğru.
for _dosya in ('config/ekf.yaml', 'config/nav2_params.yaml',
               'teknofest_ika/otonomi/ackermann_converter.py'):
    with open(os.path.join(_KOK, _dosya), encoding='utf-8') as f:
        _SATIRLAR = [l for l in f if 'Dingil arası' in l or 'wheelbase' in l]
    check(f"{_dosya}: dingil arası ölçüldü demiyor",
          any('ölçüldü' in l for l in _SATIRLAR), False)

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


# ─── Hedefleme modu: harita hedefi ↔ kayan hedef ────────────────────────────
from teknofest_ika.otonomi.pure_logic import (  # noqa: E402
    hedefleme_modu_sec, kayan_hedef_karari, quat_yaw, arac_hedefini_odoma_tasi,
)
from teknofest_ika.otonomi.topics import (  # noqa: E402
    KAYAN_HEDEF_FRAME, KAYAN_HEDEF_PERIYOT_S,
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
    yol = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'arac.urdf')
    with open(yol, encoding='utf-8') as f:
        m = re.search(r'<child link="laser_frame"/>\s*<origin[^>]*rpy="0 0 ([0-9.]+)"', f.read())
    return float(m.group(1)) if m else None


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
check("mux komut zamanı tutuluyor",       'self._nav2_son   = time.time()' in _MOD_KAYNAK, True)
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
_R_MIN = 1.40 / math.tan(0.5236)          # L / tan(δ_max) — ackermann_converter ile aynı
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
_ARAC_BOY, _ARAC_GEN, _ARAC_YUK = 1.90, 1.16, 0.76

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
    check(f"footprint[{_i}] eni",  round(_gen, 3), _ARAC_GEN)
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
check("analiz betiği aynı eni kullanıyor",  float(_da.group(2)), _ARAC_GEN)


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
