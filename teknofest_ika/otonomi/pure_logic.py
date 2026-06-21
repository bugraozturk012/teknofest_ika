#!/usr/bin/env python3
"""
pure_logic.py — Donanım/ROS Bağımsız Kritik Hesaplamalar
============================================================
Bu modül BİLEREK rclpy/ROS2 mesaj tiplerine bağımlı DEĞİLDİR — sadece
math/struct/threading/time gibi standart kütüphane kullanır.

Amaç: misyon-kritik matematiksel/mantıksal kararları (Ackermann kinematiği,
binary paket CRC'si, IMU tabanlı hız kısıtı, ardışık-frame doğrulama, vb.)
ROS Node sınıflarının İÇİNE gömülü tutmak yerine buraya çıkarmak —
böylece:

  1. test_birim.py bu fonksiyonları GERÇEKTEN import edip test edebilir
     (rclpy kurulu/sourced olmasa bile) — önceden testler bu mantığın
     birebir kopyasını kendi içinde yeniden yazıyordu, üretim kodundaki
     bir hata test tarafında hiç yakalanmıyordu.
  2. imu_guvenlik.py ve anti_rollback.py gibi farklı node'larda aynı
     quaternion→roll/pitch dönüşümü iki kez bağımsız yazılmak zorunda
     kalmıyor (DRY).

Üretim node'ları (ackermann_converter.py, seri_kopru.py, imu_guvenlik.py,
anti_rollback.py, yolo_adapter_node.py, misyon_fsm.py) bu modüldeki
fonksiyonları import edip kullanır; mantığı tekrar yazmazlar.
"""

import math
import struct
import threading


# ─────────────────────────────────────────────────────────────────────────
# 1. Ackermann Kinematiği
# ─────────────────────────────────────────────────────────────────────────

def ackermann_steering(v: float, omega: float, wheelbase: float,
                        delta_max: float) -> float:
    """
    δ = arctan(L × ω / v)  — Ackermann direksiyon açısı [rad].

    v≈0 (durağan dönüş) durumunda: ω işaretine göre maksimum açıya
    sıçranır (Nav2 recovery/spin davranışında direksiyonu sonuna çevirip
    durmak için) — "son değeri koru" DEĞİL, bilinçli bir güvenlik kararı.
    """
    if abs(v) < 1e-4:
        return math.copysign(delta_max, omega) if abs(omega) > 1e-4 else 0.0
    return max(-delta_max, min(delta_max, math.atan(wheelbase * omega / v)))


# ─────────────────────────────────────────────────────────────────────────
# 2. Enkoder Overflow Koruması (AS5600 10-bit ADC)
# ─────────────────────────────────────────────────────────────────────────

def encoder_delta(yeni: int, eski: int, maks: int = 1024) -> int:
    """Dairesel (wrap-around) enkoder farkı — 0/1023 sınırını doğru aşar."""
    d = yeni - eski
    yarim = maks // 2
    if d > yarim:
        d -= maks
    if d < -yarim:
        d += maks
    return d


# ─────────────────────────────────────────────────────────────────────────
# 3. Binary Seri Protokol — Paket Oluşturma/Doğrulama (seri_kopru.py ile
#    BİREBİR aynı format: [0xAA][CMD][D0][D1][D2][D3][CRC][0x55])
# ─────────────────────────────────────────────────────────────────────────

PKT_BOYUT = 8
PKT_BASLA = 0xAA
PKT_BITIS = 0x55


def crc_hesapla(komut: int, veri: bytes) -> int:
    """CRC = XOR(komut ^ veri[0] ^ veri[1] ^ veri[2] ^ veri[3])"""
    return komut ^ veri[0] ^ veri[1] ^ veri[2] ^ veri[3]


def paket_olustur(komut: int, v0: int, v1: int) -> bytes:
    """8 byte binary paket oluşturur (v0, v1: int16, big-endian)."""
    v0 = max(-32768, min(32767, v0))
    v1 = max(-32768, min(32767, v1))
    veri = struct.pack('>hh', v0, v1)
    crc = crc_hesapla(komut, veri)
    return struct.pack('BB4sBB', PKT_BASLA, komut, veri, crc, PKT_BITIS)


def paket_dogrula(ham: bytes) -> bool:
    """Başlangıç, bitiş ve CRC kontrolü."""
    if len(ham) != PKT_BOYUT:
        return False
    if ham[0] != PKT_BASLA or ham[7] != PKT_BITIS:
        return False
    beklenen_crc = crc_hesapla(ham[1], ham[2:6])
    return ham[6] == beklenen_crc


# ─────────────────────────────────────────────────────────────────────────
# 4. Batarya Yüzdesi (4S LiPo)
# ─────────────────────────────────────────────────────────────────────────

def batarya_yuzdesi(voltaj: float, v_min: float, v_max: float) -> float:
    """
    Voltajı 0.0-1.0 aralığına clamp'lenmiş orana çevirir — seri_kopru.py'nin
    sensor_msgs/BatteryState.percentage alanı (ROS sözleşmesi: 0.0-1.0) ile
    birebir aynı ölçek. Yüzde göstermek isteyen çağıran *100 yapar.
    """
    return max(0.0, min(1.0, (voltaj - v_min) / (v_max - v_min)))


# ─────────────────────────────────────────────────────────────────────────
# 5. IMU Quaternion → Roll / Pitch (derece)
#    imu_guvenlik.py VE anti_rollback.py tarafından paylaşılır.
# ─────────────────────────────────────────────────────────────────────────

def quat_to_roll_pitch_deg(qw: float, qx: float, qy: float, qz: float):
    """ZYX Euler — roll (X ekseni), pitch (Y ekseni), derece cinsinden."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch


# ─────────────────────────────────────────────────────────────────────────
# 6. Anti-Rollback Karar Mantığı
# ─────────────────────────────────────────────────────────────────────────

def rollback_riskli(pitch_deg: float, velocity: float,
                     pitch_threshold_deg: float,
                     vel_threshold: float) -> bool:
    """Rampada geri kayma riski: pitch eşik üstü VE hız negatif (geri)."""
    return pitch_deg > pitch_threshold_deg and velocity < -vel_threshold


# ─────────────────────────────────────────────────────────────────────────
# 7. imu_guvenlik Hız Kısıt Mantığı
# ─────────────────────────────────────────────────────────────────────────

def imu_guvenlik_hiz(roll_deg: float, pitch_deg: float, batarya_yuzde: float,
                      roll_warn: float, roll_stop: float, roll_estop: float,
                      pitch_down: float, normal_max_hiz: float,
                      frenleme_hiz: float, bat_dusuk_yuzde: float,
                      bat_kritik_yuzde: float, bat_dusuk_hiz: float) -> float:
    """
    imu_guvenlik.py._yayinla() ile birebir aynı karar ağacı.
    Roll/pitch/batarya durumuna göre güvenli azami hızı [m/s] döndürür.
    """
    roll_abs = abs(roll_deg)

    if roll_abs >= roll_estop:
        hiz = 0.0
    elif roll_abs >= roll_stop:
        hiz = 0.0
    elif roll_abs >= roll_warn:
        oran = 1.0 - (roll_abs - roll_warn) / (roll_stop - roll_warn)
        hiz = normal_max_hiz * 0.5 * max(0.0, oran)
    elif pitch_deg <= -pitch_down:
        hiz = frenleme_hiz
    else:
        hiz = normal_max_hiz

    if batarya_yuzde < bat_kritik_yuzde:
        hiz = min(hiz, 0.0)
    elif batarya_yuzde < bat_dusuk_yuzde:
        hiz = min(hiz, bat_dusuk_hiz)

    return hiz


# ─────────────────────────────────────────────────────────────────────────
# 8. Ardışık-Frame Doğrulayıcı (yolo_adapter_node.py: STOP + tabela)
# ─────────────────────────────────────────────────────────────────────────

class ConsecutiveFrameFilter:
    """
    Aynı aday (candidate) art arda `gerekli_frame` kez gelmeden onaylanmaz.
    Aday değişirse sayaç sıfırlanır.

    sticky=True  (varsayılan, parkur-aşaması tabelaları için): onaylanan
        değer, YENİ bir aday onaylanana kadar korunur — tek kare kaybı
        (örn. ışık/açı) confirmed değerini bozmaz.
    sticky=False (örn. §6.10 STOP tespiti için): aday `bos_deger`'e
        düşer düşmez confirmed de ANINDA `bos_deger`'e döner — STOP
        tabelası görüş alanından çıktığında "durma" durumunun hemen
        sona ermesi gerekir, sticky davranış aracı sonsuza dek durdurur.
    """

    def __init__(self, gerekli_frame: int, bos_deger=None, sticky: bool = True):
        self._gerekli = gerekli_frame
        self._bos = bos_deger
        self._sticky = sticky
        self._son_aday = bos_deger
        self._sayac = 0
        self._confirmed = bos_deger

    def isle(self, aday):
        """Yeni aday değeri işler, güncel onaylı değeri döndürür."""
        if aday != self._bos and aday == self._son_aday:
            self._sayac += 1
        else:
            self._son_aday = aday
            self._sayac = 1 if aday != self._bos else 0

        if self._sayac >= self._gerekli:
            self._confirmed = aday
        elif not self._sticky:
            self._confirmed = self._bos

        return self._confirmed

    @property
    def confirmed(self):
        return self._confirmed


# ─────────────────────────────────────────────────────────────────────────
# 9. FSM §6.10 STOP Cooldown Mantığı (misyon_fsm.NavigateState._stop_check)
# ─────────────────────────────────────────────────────────────────────────

def stop_check(stop_var: bool, cooldown_bitis: float, now: float) -> bool:
    """Cooldown penceresindeyse her zaman False; değilse stop_var'ı yansıtır."""
    if now < cooldown_bitis:
        return False
    return stop_var


# ─────────────────────────────────────────────────────────────────────────
# 10. HizlanmaState Hız Profili (§6.11)
# ─────────────────────────────────────────────────────────────────────────

def hizlanma_hiz_profili(dist: float, max_hiz: float, toplam_mesafe: float,
                          fren_basi_mesafe: float, min_hiz: float = 0.3) -> float:
    """
    0..fren_basi_mesafe  → max_hiz
    fren_basi..toplam    → doğrusal düşüş, min_hiz'in altına inmez
                           (seri_kopru heartbeat timeout'unu engellemek için).
    """
    if dist < fren_basi_mesafe:
        return max_hiz
    kalan = toplam_mesafe - dist
    fren_uzunlugu = toplam_mesafe - fren_basi_mesafe
    return max(min_hiz, max_hiz * (kalan / fren_uzunlugu))


# ─────────────────────────────────────────────────────────────────────────
# 11. DetectionsStore — misyon_fsm SMACH state'lerinin paylaştığı,
#     thread-safe /ika/detections son-durum deposu.
# ─────────────────────────────────────────────────────────────────────────

class DetectionsStore:
    """
    /ika/detections topic'inden gelen son JSON paketini saklar.
    SMACH state'leri bu store'dan okur — doğrudan topic callback'e bağımlı değiller.
    threading.Lock ile thread-safe erişim sağlanır.
    """

    _DEFAULT = {
        'tabela':           255,
        'koni_var':         False,
        'koni_hata_x':      0.0,
        'kayar_engel_x':    0.5,
        'kayar_yon':        'bilinmiyor',
        'hedef_var':        False,
        'hedef_hata_x':     0.0,
        'hedef_hata_y':     0.0,
        'bariyer_sol_m':    1.5,
        'bariyer_sag_m':    1.5,
        'fps':              0.0,
        'e_stop':           False,
        'manual_mod':       False,
        'hizlanma_bitti':   False,   # Tabela_11_son (class_id=3) tespit edildi
        'stop_var':         False,   # §6.10 STOP işareti (class_id=12) tespit edildi
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._data = dict(self._DEFAULT)

    def update(self, data: dict):
        with self._lock:
            self._data = {**self._DEFAULT, **data}

    def update_field(self, key: str, value):
        with self._lock:
            self._data[key] = value

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def get_field(self, key, default=None):
        with self._lock:
            return self._data.get(key, default)
