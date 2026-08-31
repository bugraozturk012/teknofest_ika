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


def ackermann_komut(v: float, omega: float, wheelbase: float,
                     delta_max: float, taban_hiz: float) -> tuple:
    """
    (hız, δ, doydu) — istenen eğriliği aracın çizebileceğine kırpar ve
    taşma oranında hızı düşürür.

    δ'yı tek başına kırpmak yetmez: R_min = L / tan(δ_max)'ten dar bir yay
    istendiğinde direksiyon doyar ama hız isteneni korur, yani araç
    çizemeyeceği virajı tam hızda dener ve dışarı taşar. Doyma sessizdir —
    ne komutta ne geri beslemede izi kalır.

    Kırpma eğrilik (κ = ω/v) üzerinden yapılır; κ'yı ±1/R_min'de sınırlamak
    δ'yı ±δ_max'ta sınırlamakla özdeştir, dolayısıyla direksiyon çıktısı
    değişmez. Değişen tek şey hız: κ ne kadar taşıyorsa o oranda düşürülür,
    böylece kontrolcü metre başına daha çok döngü koşar ve sapmayı erken
    yakalar. Yavaşlamak imkânsız virajı mümkün kılmaz — yay yine R_min'dir.

    taban_hiz kalkış sürtünmesi tabanıdır: oransal kırpma tek başına aracı
    viraj ortasında hareket edemeyeceği bir hıza düşürebilir. İstenen hız
    zaten tabanın altındaysa yükseltilmez.
    """
    delta = ackermann_steering(v, omega, wheelbase, delta_max)
    if abs(v) < 1e-4:
        return v, delta, False

    kappa_max = math.tan(delta_max) / wheelbase      # 1 / R_min
    kappa     = omega / v
    if abs(kappa) <= kappa_max:
        return v, delta, False

    oran   = kappa_max / abs(kappa)
    yeni_v = math.copysign(max(abs(v) * oran, min(abs(v), taban_hiz)), v)
    return yeni_v, delta, True


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
                      bat_kritik_yuzde: float, bat_dusuk_hiz: float,
                      taban_hiz: float = 0.0) -> float:
    """
    imu_guvenlik.py._yayinla() ile birebir aynı karar ağacı.
    Roll/pitch/batarya durumuna göre güvenli azami hızı [m/s] döndürür.

    taban_hiz kalkış sürtünmesi eşiğidir ve yalnız yan eğim rampasına
    uygulanır: rampanın çıktısı sıfır ile taban arasında kalamaz. O aralık
    aracın icra edemediği bir bölge — motor döner, araç kalkmaz, yan eğimin
    ortasında duraksayıp bir daha hareket edemez. Ya işe yarayan bir hızla
    ilerlenir ya da bilerek durulur.

    Yokuş aşağı fren moduna uygulanmaz: orada kısıt kalkış değil hızı geri
    tutmak, ve düşük komut daha çok frenlemek demek. Onu tabana yükseltmek
    %45 inişte frenlemeyi azaltırdı.
    """
    roll_abs = abs(roll_deg)

    if roll_abs >= roll_estop:
        hiz = 0.0
    elif roll_abs >= roll_stop:
        hiz = 0.0
    elif roll_abs >= roll_warn:
        oran = 1.0 - (roll_abs - roll_warn) / (roll_stop - roll_warn)
        hiz = normal_max_hiz * 0.5 * max(0.0, oran)
        if 0.0 < hiz < taban_hiz:
            hiz = taban_hiz
    elif pitch_deg <= -pitch_down:
        hiz = frenleme_hiz
    else:
        hiz = normal_max_hiz

    if batarya_yuzde <= bat_kritik_yuzde:
        hiz = min(hiz, 0.0)
    elif batarya_yuzde <= bat_dusuk_yuzde:
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

def hizlanma_hiz_profili(dist: float, max_hiz: float,
                          olcum_mesafe: float) -> float:
    """
    Şartname §6.11: 30 metre boyunca bitiş çizgisinin SONUNA KADAR ivmeli
    gidilir; durma payı çizginin ötesinde ayrıca verilir. Ölçülen bölümün
    içinde yavaşlamak, yalnız ilk altı takıma puan veren bir kalemde
    sıralama kaybetmek demek.

    Çizgide hız doğrudan sıfıra çekilir, rampa kurulmaz: fren zinciri
    (fren_hedef_hesapla) yavaşlama isteğinin BÜYÜKLÜĞÜNE bakıyor ve durma
    payına yayılmış yumuşak bir iniş eşiğin (FREN_IVME_ESIK_MIN) çok
    altında kalıp freni hiç tetiklemiyor — araç yalnızca boşta yavaşlardı.
    Sıfır komutu hem ivme terimini doyuruyor hem de hedef_hiz == 0 dalından
    tam dur oranını taban yapıyor.
    """
    return max_hiz if dist < olcum_mesafe else 0.0


def aci_sarmala(aci: float) -> float:
    """Açıyı (-pi, pi] aralığına indirir."""
    return math.atan2(math.sin(aci), math.cos(aci))


def tarama_yan_mesafe(ranges, angle_min: float, angle_increment: float,
                       merkez_aci: float, yarim_pencere: float,
                       menzil_min: float, menzil_max: float):
    """
    Taramada `merkez_aci` çevresindeki pencerenin medyan mesafesi [m],
    geçerli ışın yoksa None.

    Medyan alınıyor çünkü tek bir ışın bariyerdeki boşluğa, direğe ya da
    yağmura denk gelebiliyor; ortalama bu aykırı değerlerden kayar.
    Menzil dışı ve NaN ışınlar pencereye hiç girmez.

    Pencere karşılaştırması sarmalı yapılıyor: LiDAR gövdeye 93,3° dönük
    monte, dolayısıyla aracın sağı tarama çerçevesinde ±180° civarına
    düşüyor ve düz çıkarma o sınırda pencereyi ikiye böler.
    """
    if angle_increment == 0.0 or not ranges:
        return None

    gecerli = []
    for i, r in enumerate(ranges):
        aci = angle_min + i * angle_increment
        if abs(aci_sarmala(aci - merkez_aci)) > yarim_pencere:
            continue
        if r != r:                      # NaN
            continue
        if r < menzil_min or r > menzil_max:
            continue
        gecerli.append(r)

    if not gecerli:
        return None
    gecerli.sort()
    orta = len(gecerli) // 2
    if len(gecerli) % 2:
        return gecerli[orta]
    return (gecerli[orta - 1] + gecerli[orta]) / 2.0


def koridor_sapmasi(sol, sag, koridor_genisligi: float,
                     tolerans: float = 0.6):
    """
    (sapma, durum) — sapma > 0 ise araç koridorun SAĞINA kaymıştır.

    Şartname §6.9 tümsekli bölümde parkurun ortalanmasını istiyor; §6.1'e
    göre koridor 3 m ve iki yanı sürekli bariyerli, bariyerler 80 ± 10 cm
    yüksekliğinde — yani LiDAR düzleminde (55 cm) iki duvar da görünüyor.

    sol + sağ toplamı koridor genişliğini tutmuyorsa ölçüm koridora ait
    değildir: bir duvar görülmemiş, aşama açıklığına ya da bir yan yola
    denk gelinmiştir. O okuma sapma diye raporlanırsa aracı yanlış yöne
    çeker, bu yüzden 'koridor_yok' ile ayrılır.
    """
    if sol is None or sag is None:
        return 0.0, 'olcum_yok'
    if abs((sol + sag) - koridor_genisligi) > tolerans:
        return 0.0, 'koridor_yok'
    return (sol - sag) / 2.0, 'gecerli'


def durma_degerlendir(asilan_mesafe: float, hiz: float, butce: float,
                       hiz_esigi: float = 0.05) -> str:
    """
    'durdu' | 'butce_asildi' | 'devam' — bitiş çizgisi geçildikten sonra.

    Şartname §6.11 çizginin ötesinde 10 m emniyetli durma payı veriyor ve
    o pay içinde duramayan araca ceza yazıyor. Duruşun gerçekten olup
    olmadığı ölçülmezse ceza ancak hakem masasında öğrenilir.
    """
    if abs(hiz) <= hiz_esigi:
        return 'durdu'
    if asilan_mesafe >= butce:
        return 'butce_asildi'
    return 'devam' 


# ─────────────────────────────────────────────────────────────────────────
# 11b. Otomatik Fren Oranı (ackermann_converter.py)
# ─────────────────────────────────────────────────────────────────────────

def fren_hedef_hesapla(onceki_hiz: float, hedef_hiz: float, dt: float,
                        esik_min: float, esik_max: float,
                        tam_dur_oran: float) -> float:
    """
    Hedef hızdaki ani düşüşten fren oranı [0-1] hesaplar.
    Yalnızca YAVAŞLAMA isteğinde (hız büyüklüğü azalırken) fren üretir;
    hızlanırken veya sabit hızda 0 döner. Hedef hız tam 0 ise (gerçek dur
    komutu) en az tam_dur_oran uygulanır, ivme yavaş hesaplansa bile.
    """
    if dt <= 0.0:
        return 0.0
    yavaslama = abs(onceki_hiz) - abs(hedef_hiz)
    if yavaslama <= 0.0:
        oran = 0.0
    else:
        ivme = yavaslama / dt
        if ivme <= esik_min:
            oran = 0.0
        elif ivme >= esik_max:
            oran = 1.0
        else:
            oran = (ivme - esik_min) / (esik_max - esik_min)
    if hedef_hiz == 0.0:
        oran = max(oran, tam_dur_oran)
    return oran


def fren_yumusat(mevcut: float, hedef: float, ramp_oran_per_s: float, dt: float) -> float:
    """Fren oranını [0-1] ramp_oran_per_s hızıyla sınırlayarak hedefe yaklaştırır."""
    if dt <= 0.0:
        return mevcut
    max_degisim = ramp_oran_per_s * dt
    fark = max(-max_degisim, min(max_degisim, hedef - mevcut))
    return mevcut + fark


# ─────────────────────────────────────────────────────────────────────────
# 11. DetectionsStore — misyon_fsm SMACH state'lerinin paylaştığı,
#     thread-safe /ika/detections son-durum deposu.
# ─────────────────────────────────────────────────────────────────────────

class DetectionsStore:
    """
    misyon_fsm'in paylaşılan son-durum deposu. İki ayrı kaynaktan beslenir:

      update()       — /ika/detections JSON paketi (yolo_adapter_node).
                       Paketin sahibi olduğu alanlar her karede yeniden gelir;
                       gelmeyen alan _DEFAULT'a döner (bir karede görülen hedef
                       sonraki karede yoksa "hâlâ var" sayılmamalıdır).
      update_field() — ayrı topic'ler: /e_stop, /mod_aktif,
                       /moving_obs/direction.

    threading.Lock ile thread-safe erişim sağlanır.
    """

    # Sahibi /ika/detections DEĞİL. update() bu alanlara dokunmaz: aksi halde
    # ön kamera karesi başına (29.9 Hz) E-STOP, manuel mod ve kayar engel yönü
    # silinir ve FSM'in bu üç bayrağa bakan tüm kontrolleri hep False okur.
    _DIS_KAYNAKLI = ('e_stop', 'manual_mod', 'kayar_yon')

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
            dis = {k: self._data[k] for k in self._DIS_KAYNAKLI}
            self._data = {**self._DEFAULT, **data, **dis}

    def update_field(self, key: str, value):
        with self._lock:
            self._data[key] = value

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def get_field(self, key, default=None):
        with self._lock:
            return self._data.get(key, default)


# ═══════════════════════════════════════════════════════════════════════════════
# ŞARTNAME §6.12 KOŞU SAATİ / §9 PAS HAKKI
# ═══════════════════════════════════════════════════════════════════════════════

def kosu_butcesi(istenen_s: float, kalan_s: float) -> float:
    """
    Bir aşamaya verilebilecek gerçek süre (§6.12).

    Aşama timeout'u tek başına koşuyu sınırlamıyordu: 11 aşama × 120 s = 22
    dakika, 15 dakikalık koşu limitinin bir buçuk katı. Kalan süreden uzun bir
    timeout, dolduğunda araç zaten parkurdan çıkarılmış olacağı için yalnız
    kâğıt üzerinde vardır.
    """
    return max(0.0, min(istenen_s, kalan_s))


def pas_verilebilir(pas_gecilir: bool, label: str, pas_kullanildi: int,
                    pas_hakki: int, pas_gecilemez) -> tuple:
    """
    Şartname §9 pas kuralları. Dönüş: (izin: bool, gerekce: str).

    Üç kapı da geçilmeli:

      1. Aşama için pas bilinçli olarak açılmış olmalı. Varsayılan kapalıdır:
         pas, timeout'un otomatik sonucu değil sahadaki insan kararıdır —
         §9 pas geçmeyi "takım üyeleri parkura girerek araçlarını pas geçilen
         parkurun sonuna insan gücüyle taşıyarak konumlandıracaktır" diye
         tanımlar.
      2. Hızlanma parkuru ve otonom koşuda trafik konileri pas geçilemez;
         konfigürasyon ne derse desin bu iki aşama atlanmaz.
      3. Pas hakkı koşu başına 1 adettir. Sayaç olmadığı sürece art arda
         timeout'a giren üç aşama üç kez atlanıyordu; ikinci pasın hakem
         tablosunda karşılığı yok.
    """
    if not pas_gecilir:
        return False, 'kapali'
    if label in pas_gecilemez:
        return False, 'sartname_yasak'
    if pas_kullanildi >= pas_hakki:
        return False, 'hak_bitti'
    return True, 'izin'


def rc_mod_otonom(ch5_us: float, esik_us: float) -> bool:
    """
    RC mod potundan (CH6 VRB) otonom istenip istenmediğini döndürür.

    Karar bilerek İKİLİ ve tek eşiklidir, çünkü aynı kanalı Mega da okuyor
    (arduino/src/main.cpp guncel_mod, config.h RC_MOD_ESIK) ve eşiğin altında
    RC'yi doğrudan sürüp Jetson'ın sürüş paketlerini yok sayıyor. ROS tarafı
    kanalı başka bir yerden bölerse potun arada kaldığı bantta iki taraf aynı
    anda farklı modda olur; kanal üç konumlu bir anahtar değil sürekli bir pot
    olduğu için o bant kazayla girilebilecek bir yerdir.

    Eşikte eşitlik otonom sayılır — firmware de `> esik` değil kendi
    karşılaştırmasında aynı sınırı kullanıyor.
    """
    return ch5_us >= esik_us


def kurtarma_karari(retry_count: int, max_retries: int) -> str:
    """
    Bir aşama başarısız olduğunda ne yapılacağı. Dönüş: 'tekrar' | 'sonraki'.

    'sonraki' geldiğinde aşamadan vazgeçilip waypoint indeksi ilerletilir;
    görev İPTAL EDİLMEZ. Tek bir istasyonda takılıp koşuyu bitirmek, o
    istasyonun puanından çok daha fazlasına mal oluyordu: aşağı akışta
    koniler (50), kayar engel (50), atış (50), hızlanma (25) ve sıralamaya
    giren koşu süresi (100) duruyor. 5 puanlık bir aşama uğruna bunların
    tamamı gidiyordu.

    Vazgeçmek §9 pası DEĞİLDİR — pas kararı `pas_verilebilir`'de ve fiziksel
    bir işlemdir (araç aşama sonuna elle taşınır, puan eksiye yazılır).
    Burada araç sürmeye devam eder; istasyonlar tek koridorda arka arkaya
    dizili olduğu için engelin içinden geçmeyi fiilen denemeyi sürdürür.

    Görevi bitiren iki yol buraya hiç uğramaz: E-STOP ve §6.12 süre dolması.
    """
    return 'sonraki' if retry_count >= max_retries else 'tekrar'


def rampa_faz_gecisi(faz: str, pitch_mutlak: float, pitch_esik: float,
                     faz_mesafe: float, yaklasma_max_m: float,
                     egim_max_m: float) -> str:
    """
    Rampa aşamasının faz geçişi. Dönüş: 'yaklasma' | 'egimde' | 'bitti' |
    'bulunamadi' | 'asildi'.

    Faz sinyali KONUM değil PİTCH: waypoint koordinatları kayabilir, odometri
    sapar, ama araç eğimdeyse IMU bunu mutlak olarak söyler. Pitch'in mutlak
    değeri kullanıldığı için aynı mantık hem tırmanışta hem inişte çalışır.

    İki mesafe sınırı emniyet içindir, kontrol için değil:
      yaklasma_max_m — bu kadar sürüldüğü hâlde eğim başlamadıysa araç rampanın
                       önünde değildir; körlemesine ilerlemek yerine iptal.
      egim_max_m     — eğim bu mesafede bitmediyse pitch okuması bozuk demektir
                       (IMU montaj yönü doğrulanmadan bu gerçek bir olasılık).
    """
    if faz == 'yaklasma':
        if pitch_mutlak >= pitch_esik:
            return 'egimde'
        if faz_mesafe > yaklasma_max_m:
            return 'bulunamadi'
        return 'yaklasma'
    if faz == 'egimde':
        if pitch_mutlak < pitch_esik:
            return 'bitti'
        if faz_mesafe > egim_max_m:
            return 'asildi'
        return 'egimde'
    return faz


def koridor_merkez_cizgisi(ranges, angle_min: float, angle_increment: float,
                           lidar_yaw: float, koridor_genisligi: float = 3.0,
                           adim_m: float = 0.5, maks_ileri: float = 10.0,
                           maks_yanal: float = 2.2, menzil_min: float = 0.05,
                           menzil_max: float = 12.0, tolerans: float = 0.6):
    """
    Tek taramadan koridorun merkez çizgisi — araç çerçevesinde zincir.

    Dönüş: `[(x, y, guven), ...]` — araçtan uzaklaşan sırada, +x ileri, +y sol.
    `guven` ∈ {'iki_duvar', 'tek_duvar', 'belirsiz'}; duvarın hiç görülmediği
    adımda zincir kesilir.

    Açı konvansiyonu KoridorIzleyici ile aynı: LiDAR gövdeye dönük monte
    olduğu için araç açısı = tarama açısı + `lidar_yaw`.

    NEDEN ZİNCİR, NEDEN ARACIN İLERİ EKSENİNDE KUTULAMA DEĞİL
    ─────────────────────────────────────────────────────────────────────────
    Işınları aracın ileri eksenine göre kutulamak düz şeritte çalışır ama U
    dönüşünde çöker: koridor yana kıvrıldığı için aracın 5 m ilerisinde
    koridor yoktur, iki duvar aynı kutuya hiç düşmez. Parkur CAD'ine karşı
    ölçüldüğünde sol U dönüşünde 6,8 m, sağ U'da 4,5 m boyunca hiç hedef
    üretilemiyordu.

    Bunun yerine zincir yürütülür: her adımda mevcut yöne DİK bakılıp iki
    duvar aranır, merkez bulunur, yön o merkeze göre güncellenir. Zincir
    koridorun eğriliğini takip eder.

    NEDEN EN YAKIN DEĞİL EN UZAK DÖNÜŞ
    ─────────────────────────────────────────────────────────────────────────
    Bir yandaki `maks_yanal` içindeki EN UZAK dönüş duvar sayılır. En yakını
    almak koniyi, kayar engeli ya da bariyer önündeki her şeyi duvar sanar ve
    merkez çizgisi engellerin peşinden sürüklenir. İş bölümü net: merkez
    çizgisi KORİDORU bulur, engellerden kaçınmak costmap'in işidir.

    TUTARLILIK KAPISI
    ─────────────────────────────────────────────────────────────────────────
    İki duvar da görülüyorsa toplamları koridor genişliğini tutmalı
    (`koridor_sapmasi` ile aynı gerekçe). Tutmuyorsa ölçüm koridora ait
    değildir — bariyer boşluğundan sızan ışın ya da duvar sanılan bir engel —
    ve adım 'belirsiz' işaretlenir.
    """
    if angle_increment == 0.0 or not ranges or adim_m <= 0.0:
        return []

    noktalar = []
    for i, r in enumerate(ranges):
        if r != r:                              # NaN
            continue
        if r < menzil_min or r > menzil_max:
            continue
        aci = angle_min + i * angle_increment + lidar_yaw
        noktalar.append((r * math.cos(aci), r * math.sin(aci)))
    if not noktalar:
        return []

    yari = koridor_genisligi / 2.0
    px, py = 0.0, 0.0                           # zincirin ucu (araç)
    dx, dy = 1.0, 0.0                           # mevcut yön
    cizgi = []
    for _ in range(max(1, int(maks_ileri / adim_m))):
        ax, ay = px + dx * adim_m, py + dy * adim_m       # aday nokta
        nx, ny = -dy, dx                                  # yöne dik (sol +)
        sol = sag = None
        for qx, qy in noktalar:
            ux, uy = qx - ax, qy - ay
            if abs(ux * dx + uy * dy) > adim_m / 2.0:     # zincir boyunca dilim
                continue
            t = ux * nx + uy * ny                         # yönü dikine mesafe
            if abs(t) > maks_yanal:
                continue
            if t >= 0.0:
                if sol is None or t > sol:
                    sol = t
            else:
                if sag is None or -t > sag:
                    sag = -t

        if sol is not None and sag is not None:
            ofset = (sol - sag) / 2.0
            guven = ('iki_duvar'
                     if abs((sol + sag) - koridor_genisligi) <= tolerans
                     else 'belirsiz')
        elif sol is not None:
            ofset, guven = sol - yari, 'tek_duvar'
        elif sag is not None:
            ofset, guven = yari - sag, 'tek_duvar'
        else:
            break                                # koridor bitti, zincir kesilir

        yx, yy = ax + nx * ofset, ay + ny * ofset
        uzunluk = math.hypot(yx - px, yy - py)
        if uzunluk < 1e-6:
            break
        dx, dy = (yx - px) / uzunluk, (yy - py) / uzunluk   # yön koridoru izler
        px, py = yx, yy
        cizgi.append((px, py, guven))
    return cizgi


def kayan_hedef(cizgi, min_ileri: float = 2.0, maks_ileri: float = 8.0,
                kabul=('iki_duvar',)):
    """
    Merkez çizgisinden kayan Nav2 hedefi. Dönüş: `(x, y, yaw)` ya da None.

    Hedef, `kabul` edilen güven düzeyine sahip ve araca `maks_ileri`den yakın
    olan EN SON zincir noktasıdır. Zincir sırası mesafe sırasıdır; x'e göre
    sıralamak virajda yanlış olur çünkü koridor geri kıvrılınca x azalır.

    Mesafenin kendiliğinden ayarlanması istenen bir özellik: düz şeritte
    duvarlar uzağa kadar görünür ve hedef uzaklaşır; U dönüşünde zincir erken
    kesilince hedef kendiliğinden yakınlaşır. "Virajda yavaşla" diye ayrı bir
    kural gerekmiyor, geometri bunu zaten veriyor.

    Yön, hedefe kadarki zincirin doğrultusundan alınır; aracın anlık yönünden
    değil. Böylece hedef her döngüde algıdan yeniden doğar ve odometri kayması
    hedefe HİÇ birikmez — odometri yalnız boyuna ilerleme (istasyon takibi)
    için kalır.
    """
    uygun = [(k, c) for k, c in enumerate(cizgi)
             if c[2] in kabul and math.hypot(c[0], c[1]) <= maks_ileri]
    if not uygun:
        return None
    k, (x, y, _) = uygun[-1]
    if math.hypot(x, y) < min_ileri:
        return None
    ox, oy = (cizgi[k - 1][0], cizgi[k - 1][1]) if k > 0 else (0.0, 0.0)
    return x, y, math.atan2(y - oy, x - ox)


def ic_duvar_hedefi(ranges, angle_min: float, angle_increment: float,
                    lidar_yaw: float, taraf: str, hedef_mesafe: float = 1.4,
                    ileri_m: float = 2.0, maks_yanal: float = 2.2,
                    menzil_min: float = 0.05, menzil_max: float = 12.0,
                    komsu_m: float = 0.75, min_komsu: int = 3):
    """
    U dönüşünde iç duvarı takip eden Nav2 hedefi. Dönüş: `(x, y, yaw)` ya da None.

    `taraf` ∈ {'sol', 'sag'} — iç duvarın aracın hangi yanında olduğu. Parkurda
    sol U dönüşü (y=0 → y=10) sağa dönüştür, iç duvar SAĞDA; sağ U dönüşü
    (y=10 → y=20) sola dönüştür, iç duvar SOLDA. Hangi dönüşte olunduğunu FSM
    istasyon sırasından bilir.

    NEDEN MERKEZ ÇİZGİSİ DEĞİL
    ─────────────────────────────────────────────────────────────────────────
    U dönüşü 180°'yi sensör menzili içinde tamamlıyor (iç yarıçap ~3,3 m).
    Araçtan bakınca "ileride koridor" ile "yanımdaki koridor" aynı taramada iç
    içe geçiyor; `koridor_merkez_cizgisi` orada parkur CAD'ine karşı 5–13 m
    boyunca hedef üretemiyor. İç duvar ise virajın en sağlam özelliği: hep
    yakın, hep görünür ve eğriliği koridorun eğriliği.

    YÖNTEM
    ─────────────────────────────────────────────────────────────────────────
    Seçilen yandaki duvar noktalarından araca `ileri_m` uzaklıkta olanı seçilir,
    komşularından yerel teğet kestirilir, hedef teğete dik olarak koridorun
    içine `hedef_mesafe` kadar kaydırılır. Yön teğetten alınır. Duvar eğrildikçe
    teğet de eğrilir — ayrıca viraj yarıçapı bilmeye gerek yok.

    Teğet en az `min_komsu` komşu noktadan kestirilir; daha azıysa ölçüm
    gürültüye açıktır ve hedef üretilmez. Hedefi zorlamak, duvara paralel
    sanılan bir teğetle aracı duvara sürmek demektir.

    VARSAYILANLAR PARKUR CAD'İNE KARŞI ÖLÇÜLDÜ
    ─────────────────────────────────────────────────────────────────────────
    `scripts/parkur_cad/koridor_dogrula.py` her poz için sanal tarama üretip
    hedefi planlanmış gerçek yolla karşılaştırıyor. İki sayı belirleyici:

    `ileri_m` KISA olmalı. İç yarıçap ~3,3 m'lik bir yayda 3,5 m ileriden
    alınan duvar noktası yayın çok ötesine düşüyor ve düz kaydırma yoldan
    sapıyor: 2,0 m'de ortalama sapma 0,60 m, 3,5 m'de 1,29–2,05 m.

    `maks_yanal` DAR olmalı. U dönüşü keskin olduğu için DIŞ duvar da aracın
    aynı yanına düşüyor; pencere genişse 'duvar noktası' iç duvar yerine dış
    duvar seçiliyor ve hedef koridorun karşı tarafına atlıyor.
    """
    if angle_increment == 0.0 or not ranges or taraf not in ('sol', 'sag'):
        return None
    isaret = 1.0 if taraf == 'sol' else -1.0

    duvar = []
    for i, r in enumerate(ranges):
        if r != r:
            continue
        if r < menzil_min or r > menzil_max:
            continue
        aci = angle_min + i * angle_increment + lidar_yaw
        x, y = r * math.cos(aci), r * math.sin(aci)
        yanal = y * isaret
        if yanal <= 0.0 or yanal > maks_yanal:
            continue
        if x < -1.0:                       # arkada kalan duvar yön vermez
            continue
        duvar.append((x, y))
    if len(duvar) < min_komsu + 1:
        return None

    # Araca `ileri_m` uzaklıktaki duvar noktası
    p = min(duvar, key=lambda q: abs(math.hypot(q[0], q[1]) - ileri_m))
    komsu = [q for q in duvar
             if math.hypot(q[0] - p[0], q[1] - p[1]) <= komsu_m]
    if len(komsu) < min_komsu:
        return None

    # Yerel teğet: komşuların baş-son farkı, ileri yöne bakacak şekilde
    # Teğet, duvarın araca YAKIN ucundan UZAK ucuna doğru. Yönü ayrıca
    # "ileri baksın" diye düzeltmek hata olur: U dönüşünün ortasında koridor
    # yönü aracın gerisine dönebiliyor ve düzeltme normali duvarın İÇİNE
    # çeviriyor — hedef o zaman duvarın arkasına düşüyor.
    komsu.sort(key=lambda q: math.hypot(q[0], q[1]))
    tx, ty = komsu[-1][0] - komsu[0][0], komsu[-1][1] - komsu[0][1]
    L = math.hypot(tx, ty)
    if L < 1e-6:
        return None
    tx, ty = tx / L, ty / L

    # Teğete dik, duvardan koridorun içine doğru
    nx, ny = ty * isaret, -tx * isaret
    return p[0] + nx * hedef_mesafe, p[1] + ny * hedef_mesafe, math.atan2(ty, tx)


def quat_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Kuaterniyondan yaw [rad]. Düzlemsel dönüşümler için roll/pitch gerekmez."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def arac_hedefini_odoma_tasi(hedef, odom_poz):
    """
    Araç çerçevesinde üretilmiş `(x, y, yaw)` hedefini `odom` çerçevesine taşır.
    `odom_poz` = (x, y, yaw), aracın odom'daki anlık pozu.

    NEDEN GEREKLİ
    ─────────────────────────────────────────────────────────────────────────
    Hedef Nav2'ye araç çerçevesinde verilirse Nav2 onu HER YENİDEN PLANLAMADA
    o anki araç pozuna göre yeniden çözer: hedef araçla birlikte kayar, asla
    varılmaz ve hedef denetleyicisi hiç tetiklenmez. Araç çerçevesindeki
    hedef "2 m ilerisi" demektir, "şu nokta" değil.

    `odom` seçilir, `map` değil: odom sürüklenir ama SIÇRAMAZ. Hedef zaten
    her döngüde taramadan yeniden doğduğu için 1,5 saniyelik ömrü boyunca
    sürüklenmenin etkisi ölçülemeyecek kadar küçüktür; `map` ise SLAM
    düzeltmesiyle sıçrayabilir ve hedefi bir anda metrelerce kaydırır.
    """
    ox, oy, oyaw = odom_poz
    hx, hy, hyaw = hedef
    return (ox + math.cos(oyaw) * hx - math.sin(oyaw) * hy,
            oy + math.sin(oyaw) * hx + math.cos(oyaw) * hy,
            oyaw + hyaw)


def hedefleme_modu_sec(istenen: str, waypoint_dolu: bool):
    """
    Koşunun hangi hedefleme yoluyla sürüleceğini seçer. Dönüş: `(mod, gerekçe)`.

    'harita'  waypoints.yaml'ın `map` çerçevesindeki koordinatları Nav2'ye
              hedef olarak verilir. Koordinatların anlamlı olması için SLAM
              haritası ile parkur arasındaki katı dönüşümün ölçülmüş olması
              gerekir (`parkur_cad.donusum`).
    'kayan'   hedef her döngüde LiDAR taramasından yeniden üretilir; `map`
              çerçevesine de waypoint koordinatlarına da ihtiyaç yoktur.
    'oto'     waypoint'ler doluysa 'harita', hepsi (0,0) ise 'kayan'.

    Seçim KOŞU BAŞINDA bir kez yapılır. Koşu ortasında sessizce mod
    değiştirmek en kötüsü olurdu: araç neden başka türlü davrandığını kimse
    anlamaz, log'da da tek bir satır olarak kaybolur.

    Tanınmayan bir değer 'oto' gibi ele alınır ve gerekçede bildirilir.
    Çökmek koşuyu tamamen engellerdi; sessizce 'harita'ya düşmek ise
    waypoint'ler (0,0) iken aracı hiç sürmeden "tamamlandı" dedirtirdi.
    """
    if istenen not in ('harita', 'kayan', 'oto'):
        return ('harita' if waypoint_dolu else 'kayan'), f'gecersiz_istenen:{istenen}'
    if istenen == 'oto':
        return ('harita' if waypoint_dolu else 'kayan'), 'oto'
    if istenen == 'harita' and not waypoint_dolu:
        # İstenen açıkça 'harita' ise ona uyulur; ama bu koşunun parkuru hiç
        # sürmeden biteceği anlamına gelir, çağıran yüksek sesle uyarmalı.
        return 'harita', 'harita_ama_waypointler_bos'
    return istenen, 'istendi'


def kayan_hedef_karari(kat_edilen_m: float, mesafe_m: float,
                       hedefsiz_ardisik: int, hedefsiz_sinir: int) -> str:
    """
    Kayan hedefle sürülen bir aşamanın devam edip etmeyeceği.
    Dönüş: 'tamam' | 'hedef_yok' | 'devam'.

    `kat_edilen_m` odometreden biriktirilen YOL UZUNLUĞU olmalı, başlangıç
    noktasına olan uzaklık değil: U dönüşünde ikisi 22 m'ye karşı 11 m gibi
    ayrışır ve kuş uçuşu ölçüt aşamayı yolun yarısında bitirirdi.

    Varış ölçütü önce bakılır: mesafe tamamlandıysa hedef üretilememesi
    önemsizdir, aşama zaten bitmiştir (istasyonun sonunda koridor açılıp
    tarama koridor geometrisini tutmayabilir).
    """
    if mesafe_m > 0.0 and kat_edilen_m >= mesafe_m:
        return 'tamam'
    if hedefsiz_ardisik >= hedefsiz_sinir:
        return 'hedef_yok'
    return 'devam'


# ─────────────────────────────────────────────────────────────────────────────
# LİDAR MONTAJ AÇISI DÖNÜŞÜMLERİ
#
# LaserScan'in angle_min/angle_increment'i TARAMA çerçevesindedir. LiDAR
# gövdeye dönük monte olduğu için tarama açısı ile araç açısı aynı şey
# değildir; taramayı dizi olarak indeksleyen her düğüm dönüşü kendi
# uygulamak zorundadır (TF yalnız costmap'e yerleştirmeyi düzeltir).
# ─────────────────────────────────────────────────────────────────────────────

def tarama_acisi_arac(tarama_acisi: float, lidar_yaw: float) -> float:
    """Tarama çerçevesindeki açıyı araç çerçevesine çevirir [rad, −π…π]."""
    return math.remainder(tarama_acisi + lidar_yaw, 2.0 * math.pi)


def arac_acisi_tarama(arac_acisi: float, lidar_yaw: float) -> float:
    """Araç çerçevesindeki açıyı tarama çerçevesine çevirir [rad, −π…π]."""
    return math.remainder(arac_acisi - lidar_yaw, 2.0 * math.pi)


def aci_pencerede(aci: float, alt: float, ust: float) -> bool:
    """
    `aci` [alt, ust] penceresinde mi — SARMALI karşılaştırma.

    Düz `alt <= aci <= ust` yetmez: montaj dönüşünden sonra pencere ±180°
    sınırını aşabiliyor (örn. araç çerçevesinde bitişik olan iki açı tarama
    çerçevesinde +176° ve −179° olarak görünür). Düz karşılaştırma o pencereyi
    ikiye böler ve arada kalan huzmeleri sessizce atar.
    """
    genislik = math.remainder(ust - alt, 2.0 * math.pi)
    if genislik < 0.0:
        genislik += 2.0 * math.pi
    fark = math.remainder(aci - alt, 2.0 * math.pi)
    if fark < 0.0:
        fark += 2.0 * math.pi
    return fark <= genislik


def tarama_kirpma_penceresi(alt_arac: float, ust_arac: float, lidar_yaw: float):
    """
    ARAÇ çerçevesinde tanımlı bir açı penceresini TARAMA çerçevesine taşır.
    Dönüş: `(alt, ust)` — ikisi de [−π, π).

    `alt > ust` çıkabilir ve bu bir hata DEĞİLDİR: montaj dönüşünden sonra
    pencere ±180° sınırını aşar. Çağıran o durumda `>= alt VEYA <= ust`
    uygulamalı; düz `alt <= a <= ust` pencereyi ikiye böler ve arada kalan
    huzmeleri sessizce atar.

    Pencere taramanın kendisine değil bir kez pencereye uygulanıyor: dönüşüm
    her tarama için 600+ açıya değil, açılışta iki sayıya yapılıyor.
    """
    return (arac_acisi_tarama(alt_arac, lidar_yaw),
            arac_acisi_tarama(ust_arac, lidar_yaw))
