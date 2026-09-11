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
# 2. Binary Seri Protokol — Paket Oluşturma/Doğrulama (seri_kopru.py ile
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


def paket_v0_i(ham: bytes) -> int:
    """Birinci alan, işaretli int16."""
    return struct.unpack('>h', ham[2:4])[0]


def paket_v1_i(ham: bytes) -> int:
    """İkinci alan, işaretli int16."""
    return struct.unpack('>h', ham[4:6])[0]


def paket_v0_u(ham: bytes) -> int:
    """Birinci alan, işaretsiz uint16 — bayrak kümeleri ve sayaçlar için."""
    return struct.unpack('>H', ham[2:4])[0]


def paket_v1_u(ham: bytes) -> int:
    """İkinci alan, işaretsiz uint16."""
    return struct.unpack('>H', ham[4:6])[0]


def paket_int32(ham: bytes) -> int:
    """
    İki alanı tek bir işaretli int32 olarak okur (0x30 enkoder sayımı).
    Taşmayı sürüş kartı çözüyor; burada sarma düzeltmesi uygulanmaz.
    """
    return struct.unpack('>i', ham[2:6])[0]


# ─────────────────────────────────────────────────────────────────────────
# 3. Sürüş Kartı Telemetrisi → RC Uyumluluk Dizisi
# ─────────────────────────────────────────────────────────────────────────

RC_US_MIN     = 1000.0
RC_US_NEUTRAL = 1500.0
RC_US_MAX     = 2000.0


def gaz_binde_us(binde: int) -> float:
    """
    Kartın binde cinsinden bildirdiği gazı (0x36 v0, -1000…1000) kumanda
    kanallarının µs ölçeğine taşır. Pano ve teşhis aynı ölçeği bekliyor.
    """
    return max(RC_US_MIN, min(RC_US_MAX, RC_US_NEUTRAL + binde * 0.5))


def kip_us(kip: int, otonom_kip: int = 2) -> float:
    """
    Kartın çözdüğü sürüş kipini /rc_input'un mod alanına taşır.

    Ham CH9 bilerek kullanılmaz: kanal üç konumlu bir anahtar ve orta konumu
    1500 µs'e denk geliyor, yani o değeri eşikleyen taraf orta kipi OTONOM
    okurdu — oysa orta kademe kartın yumuşak E-STOP'u. Yalnız kartın otonom
    dediği kip otonom sayılır, geri kalan her değer güvenli tarafa yazılır.
    """
    return RC_US_MAX if kip == otonom_kip else RC_US_MIN


def rc_dizisi(gaz_binde: int, ham_ch1: float, kip: int, taret_aktif: bool,
              ham_ch9: float, otonom_kip: int = 2) -> list:
    """
    seri_kopru'nun /rc_input dizisini kurar:
      [0] gaz · [1] direksiyon · [2] kip · [3] aux/lazer · [4] tilt
      [5] taret aktif · [6] ham CH9 (teşhis)

    Kart ham kanalların yalnız ikisini gönderiyor (CH1, CH9); gaz ve taret
    türetilmiş alanlardan gelir, aux ve tilt'in karşılığı yok ve etkisiz
    değerde tutulur.
    """
    return [
        gaz_binde_us(gaz_binde),
        float(ham_ch1),
        kip_us(kip, otonom_kip),
        RC_US_MIN,                                   # aux/lazer: kaynağı yok
        RC_US_NEUTRAL,                               # tilt: kaynağı yok
        RC_US_MAX if taret_aktif else RC_US_MIN,
        float(ham_ch9),
    ]


def bms_okuma_gecerli(veri: dict, yas_esigi: float) -> tuple:
    """
    BMS ucundan gelen gövde yayınlanacak kadar taze ve dolu mu.
    Dönüş: `(gecerli, sebep)`.

    🔑 Ölçüt `bagli` DEĞİL `yas`. BLE koptuğunda servis ölçüm alanlarını
    silmiyor, SON DEĞERDE donduruyor: bağlantı bayrağına bakan bir tüketici
    donmuş bir gerilimi canlı sanır. Tersi de geçerli — bağlantı o an kopuk
    görünse bile son okuma tazeyse sayı kullanılabilir.

    Bayat okumayı yayınlamaktansa susmak seçiliyor, çünkü /battery/status'u
    okuyan taraf gelen son yüzdeyi kalıcı olarak tutuyor: bir kez basılan
    yanlış değer bir daha düzelmiyor.
    """
    if not isinstance(veri, dict):
        return False, 'govde_sozluk_degil'
    yas = veri.get('yas')
    if yas is None:
        return False, 'yas_yok'
    try:
        if float(yas) > yas_esigi:
            return False, 'bayat'
    except (TypeError, ValueError):
        return False, 'yas_sayi_degil'
    if veri.get('v48') is None or veri.get('bms_enaz') is None:
        return False, 'olcum_yok'
    return True, 'gecerli'


def bms_dip_olu(dip_mv: float, dip_esigi: float) -> bool:
    """
    Kesme kararı en düşük hücreden verilir, BMS'in SOC tahmininden değil.

    LiFePO4'ün deşarj eğrisi düz: paket %60 SOC gösterirken tek bir çökmüş
    hücre dibi görmüş olabilir. Paket gerilimi ve SOC ortalamadır ve o hücreyi
    gizler; kesmesi gereken şey ise tam olarak odur.
    """
    return dip_mv <= dip_esigi


def kip_modu(kart_kip: int, bayat: bool, otonom_kip: int = 2,
             mod_manuel: int = 0, mod_otonom: int = 2) -> int:
    """
    Sürüş kartının bildirdiği kipten mod yöneticisinin modu.

    Kip kararı kartta veriliyor; Jetson'ın oyu yok. Kart susarsa son bilinen
    kipte kalmak, gerçekte manuel sürülen bir araca otonom komut basmaya
    dönüşebilir — bayat veri bu yüzden manuel sayılır.

    Orta kip (SwC yumuşak E-STOP) de manuel tarafına düşer: otonom yalnız
    kartın açıkça otonom dediği değerdir. Araç o kipte zaten kilitli, mux'ın
    ne bastığı sonucu değiştirmiyor.
    """
    if bayat:
        return mod_manuel
    return mod_otonom if kart_kip == otonom_kip else mod_manuel


def kesme_estop(drm_bayraklar: int, veri_geldi: bool,
                kesme_biti: int = 0x01) -> bool:
    """
    Kumandadaki kesme anahtarından (SwA) E-STOP.

    Alıcı, verici kapalıyken de yayın sürdürdüğü için "çerçeve gelmiyorsa dur"
    mantığı bu araçta sinyal kaybını yakalamıyor; yakalayan tek şey alıcının
    failsafe kaydı ve o kayıt CH7'yi kesme konumuna düşürüyor. Kaynak bu
    yüzden çerçeve sessizliğine değil kesme bitine bakar.

    Veri hiç gelmediyse False döner: veri yokluğunu E-STOP'a çevirmek, kaynağı
    açılışta kalıcı olarak kilitler ve hiçbir şey onu temizleyemez.
    """
    return veri_geldi and bool(drm_bayraklar & kesme_biti)


def enkoder_sessiz(hiz_mms: int, sayim_sabit_s: float,
                   hiz_esigi_mms: int = 100,
                   sure_esigi_s: float = 1.0) -> bool:
    """
    Kart hareket ettiğini söylerken enkoder sayımı duruyorsa True.

    Sürüş kartındaki karşılığı ölü bir bayrak: karşılaştırma için ikinci bir
    hız kaynağı (gösterge ucu) gerekiyordu ve o uç tasarımdan çıktı. Denetim
    bu yüzden bu tarafta kuruluyor ve iki bağımsız alana bakıyor: kartın
    anladığı hız (0x38) ile ham sayım (0x30).

    Eşik kartın kalkış tabanının altında tutulur — taban altındaki komutlar
    zaten tabana yükseltildiği için, gerçek bir sürüş komutu her zaman bunun
    üstündedir.
    """
    return abs(hiz_mms) >= hiz_esigi_mms and sayim_sabit_s >= sure_esigi_s


def surum_uyumlu(protokol: int, beklenen: int) -> bool:
    """
    Yalnız protokol sürümü karşılaştırılır. Firmware yapı numarası davranış
    değiştiren her yüklemede artıyor ve paket anlamlarını değiştirmiyor;
    onu karşılaştırmak ilk güncellemede sahte alarm verir.
    """
    return protokol == beklenen


# Sürüş kartının ayar paketi (0x09) — kimlik → ölçek. Kart bu sayıları
# FLASH'A YAZMIYOR, RAM'de tutuyor: reset olduğunda hepsi sıfırlanır ve hız
# alanı sessizce 0 basmaya döner. Kalıcılık bu yüzden kartta değil köprüde;
# değerler kart her göründüğünde yeniden gönderilir.
AYAR_CEVRE_MM      = 1   # tekerlek yuvarlanma çevresi
AYAR_DISLI_ORANI   = 2   # enkoder mili turu : teker turu
AYAR_DIREKSIYON    = 3   # direksiyon kolon/teker oranı
AYAR_DARBE_TUR     = 4   # gösterge darbe/tur
AYAR_DIR_ISARET    = 5   # direksiyon işareti (+1 / -1)

_AYAR_OLCEK = {
    AYAR_CEVRE_MM:    10.0,
    AYAR_DISLI_ORANI: 1000.0,
    AYAR_DIREKSIYON:  1000.0,
    AYAR_DARBE_TUR:   10.0,
    AYAR_DIR_ISARET:  1.0,
}
_INT16_MIN, _INT16_MAX = -32768, 32767


def ayar_ham(kimlik: int, deger: float):
    """
    Ayar değerini kartın beklediği int16'ya çevirir.
    Dönüş: `(ham, sebep)` — `ham` None ise gönderilmez.

    Ölçek kimliğe bağlı ve tek yerde duruyor: iki tarafın ayrı ayrı çarpan
    tutması, sayının sessizce on kat yanlış girilmesinin en kolay yoludur.
    """
    olcek = _AYAR_OLCEK.get(kimlik)
    if olcek is None:
        return None, 'bilinmeyen_kimlik'
    # Kartın kabul kuralları burada da uygulanır: 1–4 için sıfır ve negatif
    # reddediliyor (0 yazmak "ölçüm yok" demek ve hız alanını sessizce
    # sıfırlar), kimlik 5 yalnız ±1. Erken elemek, karşılaştırma alarmını
    # beklemekten iyi: reddedilen paket sahada "neden tutmuyor" diye aranır.
    if kimlik == AYAR_DIR_ISARET:
        if deger not in (1.0, -1.0):
            return None, 'isaret_gecersiz'
    elif deger <= 0.0:
        return None, 'pozitif_olmali'
    ham = int(round(deger * olcek))
    if not (_INT16_MIN <= ham <= _INT16_MAX):
        return None, 'aralik_disi'
    return ham, 'gecerli'


def ayar_deger(kimlik: int, ham: int):
    """Kartın geri yolladığı ham sayıyı ölçeğine döndürür; bilinmeyen kimlikte None."""
    olcek = _AYAR_OLCEK.get(kimlik)
    return None if olcek is None else ham / olcek


def ayar_gonderilecek(deger: float) -> bool:
    """
    Ayar yapılandırıldı mı. Sıfır "ölçülmedi" demek: ölçülmemiş bir sayıyı
    göndermek, kartın bilerek sustuğu alana uydurma bir değer yazmaktır.
    """
    return abs(deger) > 1e-9


def yaw_kovaryansi(sys_kalib: int, esik: int = 3,
                   guvenilir: float = 0.02, supheli: float = 0.30) -> float:
    """
    BNO055 sistem kalibrasyonundan /imu/data'nın yaw kovaryansı [rad²].

    Eşik kartınkiyle aynı tutulur: kart `sys < 3` iken HATA_BNO_KALIB basıyor,
    yani o aralıkta yaw'ı güvenilir saymak EKF'e kartın kendisinin yetersiz
    saydığı bir ölçümü tam ağırlıkla vermek olur.

    Bedeli tek bir açıyla sınırlı değil: arka aks tek parça olduğu için yönün
    ikinci bir kaynağı yok, EKF yaw'ı yalnız buradan alıyor. Hak etmediği
    ağırlıkla alınan bir yaw doğrudan rotaya çıkar.
    """
    return guvenilir if sys_kalib >= esik else supheli


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


def lazer_sonmeli(lazer_aktif: bool, kip, otonom_kip: int) -> bool:
    """
    Yanan lazerin söndürülmesi gerekiyor mu.

    Atış isteği tek bir True ile açılıp saniyelerce açık kalıyor; giriş kapısı
    (isteği karta ileten geri çağırma) yalnız mesaj geldiğinde işliyor. O
    pencerede operatör kip anahtarını otonomdan çıkarırsa giriş kapısı bir
    daha çağrılmaz ve lazer yanmaya devam eder — kapının periyodik bir eşi
    olmak zorunda.

    `kip` None olabilir: kart henüz kip bildirmemiştir. Bilinmeyen kip otonom
    SAYILMAZ, yani lazer söndürülür — ateş yetkisini varsayıma dayandırmak,
    kapının olmamasıyla aynı kapıya çıkar.
    """
    return lazer_aktif and kip != otonom_kip


def yagmur_lekeleri(alanlar, maks_alan: int, maks_adet: int):
    """
    Bağlı bileşen alanlarından hangilerinin yağmur damlası sayılacağını seçer.
    `alanlar[0]` arka plandır ve hiçbir zaman seçilmez.

    Dönüş: `(kimlikler, sebep)` — sebep 'onar' değilse kimlikler boştur ve
    görüntüye dokunulmaz.

    İKİ KAPI, ikisi de gerekli:

    · **alan** — yağmur damlası küçük ve izole bir lekedir. Şerit çizgisi,
      koninin beyaz bandı ya da atış hedefinin halkaları da parlaklık eşiğini
      geçer ama büyük/bitişik alanlar üretir; onları onarmak gerçek nesneyi
      bozmak olur.

    · **adet** — yüzlerce küçük leke YAĞMUR DEĞİLDİR, sahnenin kendi
      dokusudur (güneşli asfalt, çakıl, parlayan metal). Onarım maliyeti leke
      sayısıyla hızla büyüdüğü için bu durum tam da onarılacak bir şey
      olmadığı anda en pahalı hâle gelir: 640×480'de 350 leke kare başına
      ~25 ms, iki kamerada 30 Hz'de bir buçuk çekirdek. Sınırın üstünde
      görüntüye hiç dokunulmuyor.
    """
    kimlikler = [i for i in range(1, len(alanlar)) if alanlar[i] <= maks_alan]
    if not kimlikler:
        return [], 'leke_yok'
    if len(kimlikler) > maks_adet:
        return [], 'cok_leke'
    return kimlikler, 'onar'


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


def rampa_ara_durus_gerekli(faz: str, faz_mesafe_m: float,
                            durus_mesafe_m: float, yapildi: bool) -> bool:
    """
    §6.10'un eğim ÜZERİNDEKİ zorunlu duruşu şimdi yapılmalı mı.

    Duruş noktası eğimin ortasında, rampa dibinde DEĞİL: parkur CAD'inde
    rampa 8,60 m uzunluğunda ve 1,85 m yüksekliğinde, %45 eğimde her yamacın
    yatay koşusu 4,11 m ve yamaç boyu 4,51 m — orta nokta 2,25 m.

    Ölçüt PİTCH OLAMAZ: eğim boyunca pitch sabittir, nerede olduğunu söylemez.
    Bu yüzden mutlak bir başlangıç (pitch eşiği ile 'egimde' fazına giriş) ile
    kısa mesafeli odometri birleştiriliyor; iki metrede odometri sapması
    ihmal edilebilir.

    `durus_mesafe_m <= 0` duruşu kapatır (düz zeminde sürülen aşamalar).
    """
    if yapildi or faz != 'egimde':
        return False
    if durus_mesafe_m <= 0.0:
        return False
    return faz_mesafe_m >= durus_mesafe_m


def yokus_kalkis_freni(gecen_s: float, tutma_binde: int, tork_s: float,
                       birakma_binde_per_s: float) -> tuple:
    """
    Yokuş kalkışının o anki fren değeri. Dönüş: `(fren_binde, bitti)`.

    Üç pencere:
      1. `gecen_s < tork_s`      → fren TAM tutuyor, gaz zaten veriliyor;
                                   motor torku bu pencerede oturuyor.
      2. rampa                    → fren `birakma_binde_per_s` ile sıfıra iner.
      3. fren sıfırlandı          → `bitti = True`, override bırakılır.

    Gazın frenden ÖNCE verilmesi işin özü: sıra ters olursa fren bırakıldığı an
    tork henüz yoktur, araç geri kaçar ve sürücü ters dönen rotoru sürmeyi
    reddeder — kaçınılmak istenen tam olarak budur.
    """
    if gecen_s < tork_s:
        return tutma_binde, False
    if birakma_binde_per_s <= 0.0:
        return 0, True
    dusen = (gecen_s - tork_s) * birakma_binde_per_s
    kalan = tutma_binde - dusen
    if kalan <= 0.0:
        return 0, True
    return int(round(kalan)), False


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


def hedef_ulasilabilir_mi(hedef, min_ileri_m: float,
                          donus_yaricapi_m) -> bool:
    """
    Araç çerçevesindeki `(x, y, yaw)` hedefine ileri yönde gidilebilir mi.

    NEDEN GEREKLİ
    ─────────────────────────────────────────────────────────────────────────
    Hedef taramadan doğuyor ve üreten iki yolun ikisinde de "ileride mi"
    sorusunu soran bir kapı yoktu:

      · `koridor_merkez_cizgisi` zincirin yönünü her adımda merkeze göre
        güncelliyor ve ADIM BAŞINA DÖNÜŞ SINIRI YOK. Adım 0,5 m ileri, yanal
        kayma 2,2 m'ye kadar — tek adımda yön 77° dönebiliyor. Kıvrılmaya
        sebep olan adımlar 'belirsiz' işaretlenip hedef adayı olmaktan çıkıyor
        ama zincirin YÖNÜNÜ yine de değiştiriyorlar; sonraki 'iki_duvar'
        noktası sapmış zincirin üstünde duruyor ve kabul ediliyor.
      · `kayan_hedef`'in `min_ileri` kapısı `hypot` ile ölçüyor, yani
        İŞARETSİZ: aracın 2,5 m ARKASINDAKİ bir nokta o kapıdan geçiyor.
      · `ic_duvar_hedefi`'nde hiç mesafe kapısı yok ve duvar noktasını
        x >= -1,0 ile kabul ediyor.

    Planlayıcı `REEDS_SHEPP`, yani geri yay teorik olarak var; ama araç
    arkadan kör ve geri manevra yalnız sıkışmadan çıkmak için açık
    (`reverse_penalty` yüksek). Kayan hedefin ürettiği bir hedef arkada
    kalırsa planlayıcı ya 2 × R_min ≈ 5,0 m'lik bir dönüş arar — koridor
    3 m, sığmaz — ya da aracı göremediği alana geri sürer. İkisi de
    istenmiyor: kapı hedefi baştan eliyor.

    YAY ÖLÇÜTÜ HER ÜRETİCİ İÇİN GEÇERLİ DEĞİL
    ─────────────────────────────────────────────────────────────────────────
    `donus_yaricapi_m=None` yay ölçütünü kapatır; yön ve mesafe kapıları
    kalır. İç duvar takibi bu kipte çağrılıyor ve sebebi ölçüm:

      merkez çizgisi  hedefi metrelerce ilerideki bir YÖN NOKTASI; parkur
                      CAD'inde medyan 5–8 m'de doğuyor, yani yay bandının
                      (d < 2·R_min ≈ 4,8 m) çoğunlukla dışında. Ölçüt orada
                      bedava ve kısmen kıvrılmış zincirin ürettiği "önde ama
                      neredeyse tam yanda" hedefi yakalıyor.
      iç duvar        hedefi YAKIN BİR YANAL DÜZELTME: duvar noktası 2 m
                      menzilde, hedef ondan 1,4 m koridorun içine kaydırılıyor.
                      CAD'de 20–30° açılarda, 1,5–1,8 m'de doğan meşru
                      hedefler yay ölçütüne takılıyordu (U dönüşü başına 2–3
                      hedef). Dönüş fiilen mümkün — iç yarıçap ~3,3 m,
                      R_min 2,42 — sadece o yakın noktaya TEK yayla değil.
                      Kayan hedef varılacak nokta değil yönlendirme; 1,5 s'de
                      bir yenileniyor ve hiç varılmıyor.

    ÖLÇÜT: TEK YAYLA ERİŞİLEBİLİRLİK
    ─────────────────────────────────────────────────────────────────────────
    Aracın anlık yönüne teğet olup hedeften geçen dairenin yarıçapı, çemberin
    orijinde x eksenine teğet olması koşulundan doğrudan çıkar:

        x² + (y − R)² = R²   →   R = (x² + y²) / (2y)

    Yaklaşım değil, kapalı çözüm. |R| dönüş yarıçapının altındaysa araç o
    hedefe tek yayla dönemez. Dubins yolu iki yay + doğrudan oluştuğu için bu
    ölçüt planlayıcıdan biraz DAHA SIKI: elediği bazı hedefler aslında üç
    parçalı bir yolla çözülebilir. Sıkı tarafta durmak bilinçli — koridor 3 m
    ve o üç parçalı yolun sığacağı yer yok.

    y = 0 (tam ileri) hedefinde yarıçap sonsuz, kapı yalnız mesafeye bakar.
    """
    x, y, _ = hedef
    if x <= 0.0:                       # arkada ya da tam yanda
        return False
    if math.hypot(x, y) < min_ileri_m:
        return False
    if donus_yaricapi_m is None:       # yay ölçütü bu üretici için geçerli değil
        return True
    if abs(y) < 1e-9:                  # düz ileri: yarıçap sonsuz
        return True
    yaricap = (x * x + y * y) / (2.0 * abs(y))
    return yaricap >= donus_yaricapi_m


def hedef_yeniden_gonderilsin_mi(yeni_xy, son_gonderilen_xy,
                                 etkin_hedef_var: bool,
                                 olu_bant_m: float) -> bool:
    """
    Kayan hedefin Nav2'ye YENİDEN gönderilip gönderilmeyeceği.

    NEDEN BASTIRMA GEREKLİ
    ─────────────────────────────────────────────────────────────────────────
    Nav2'nin kurtarma dalı `<GoalUpdated/>` ile korunuyor: hedef değiştiyse
    çalışan kurtarma davranışı halt edilip ana boru hattına dönülüyor. Biz her
    periyotta yeni zaman damgasıyla gönderdiğimiz için hedef, araç hiç
    kımıldamasa bile "değişti" okunuyordu. Sonucu, aracı sıkıştığı pozdan
    çıkarabilecek tek davranışın (BackUp) hiç tamamlanamamasıydı.

    Bastırma yalnız araç kımıldamadığında devreye girer: gerçek hareket bir
    periyotta hedefi ölü bandın çok üstünde kaydırır (bkz.
    KAYAN_HEDEF_OLU_BANT_M).

    NEDEN `etkin_hedef_var` AYRI BİR KAPI
    ─────────────────────────────────────────────────────────────────────────
    Nav2 hedefi abort edebiliyor (altı kurtarma hakkı dolunca) ve kayan sürüş
    bunu ancak sonucu dinleyerek öğreniyor. Etkin hedef yokken bastırma
    yapılırsa araç, hedefi olmadığı için hiç sürmez ve bunu kimse söylemez —
    sıkışmayı çözerken yeni bir sıkışma üretmiş oluruz. Bu yüzden "hedef yok"
    her zaman gönderim sebebidir, kayma miktarına bakılmaz.

    ZAMAN TABANLI KAÇIŞ YOK
    ─────────────────────────────────────────────────────────────────────────
    "N saniyedir göndermedik, yine de gönder" kuralı bilerek konmadı: o kural
    kurtarmayı tam ihtiyaç duyulan anda yeniden keserdi. Süre güvencesi
    aşamanın kendi bütçesinde ve FSM'in iki denemesinde.
    """
    if not etkin_hedef_var or son_gonderilen_xy is None:
        return True
    dx = yeni_xy[0] - son_gonderilen_xy[0]
    dy = yeni_xy[1] - son_gonderilen_xy[1]
    return math.hypot(dx, dy) >= olu_bant_m


def olcum_bayat_mi(son_zaman: float, simdi: float, sinir_s: float) -> bool:
    """
    Bir ölçüm akışının kör sürmeyi başlatacak kadar sessizleşip sessizleşmediği.

    `son_zaman == 0.0` "hiç gelmedi" demektir ve bayatla aynı sınıfa konur:
    ikisinde de elde güncel ölçüm yoktur, ayrı ele almak yalnız iki ayrı kod
    yolu üretirdi.

    NEDEN AYRI BİR FONKSİYON
    ─────────────────────────────────────────────────────────────────────────
    Kayan sürüş iki ölçüme birden yaslanıyor: odometri aşamanın NEREDE
    biteceğini, tarama NEREYE gidileceğini söylüyor. Odometri için bu kapı
    baştan vardı, tarama için yoktu — `_scan` son taramayı süresiz tutuyor ve
    hedef ölü veriden üretilmeye devam ediyordu. En sinsi yanı `hedefsiz`
    sayacının hiç artmamasıydı: hedef ÜRETİLİYOR, yalnız koridorun
    hafızasından. Kural tek yerde durunca iki akış da aynı davranıyor.
    """
    return son_zaman == 0.0 or (simdi - son_zaman) > sinir_s


def yol_artimi(adim_m: float, dt_s: float, taban_hiz_ms: float) -> float:
    """
    Bir odometri örneğinin kat edilen yola KATACAĞI mesafe. Gürültü 0 döner.

    NEDEN ÖLÜ BANT ŞART
    ─────────────────────────────────────────────────────────────────────────
    Yol, ardışık konumların |Δ|'sı toplanarak birikiyor. Mutlak değer olduğu
    için poz gürültüsü negatif katkı VEREMEZ: duran araçta da toplam büyür.
    EKF 50 Hz'de koştuğundan bu, aşamanın bitiş ölçütünü sessizce şişirir ve
    araç istasyona varmadan sonrakine geçer.

    NEDEN METRE DEĞİL HIZ EŞİĞİ
    ─────────────────────────────────────────────────────────────────────────
    Sabit bir metre eşiği örnekleme frekansına bağlı olurdu: EKF yükte
    seyrelirse adımlar büyür ve aynı eşik gerçek yavaş hareketi elemeye
    başlardı. Hız cinsinden eşik frekanstan bağımsızdır.

    `dt_s <= 0` (aynı ana damgalanmış iki örnek, geri giden saat) bölmeyi
    tanımsız yapar; o örnek yola sayılmaz — bir örnek atlamak, sonsuz bir
    artım eklemekten ucuzdur.
    """
    if dt_s <= 0.0:
        return 0.0
    return adim_m if adim_m / dt_s >= taban_hiz_ms else 0.0


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


def rollback_mudahale_gerekli(pitch_deg: float, velocity: float,
                              komut_vx: float, komut_bayat: bool,
                              pitch_esik_deg: float, hiz_esik: float,
                              geri_komut_esigi: float = 0.05) -> bool:
    """
    Geri kayma korumasının müdahale edip etmeyeceği.

    `rollback_riskli` yalnız fiziğe bakar: burun yukarı + hız negatif. Bu tek
    başına yetmiyor, çünkü aynı tabloyu KASITLI bir geri gitme de üretir —
    Nav2'nin BackUp kurtarması rampada devreye girerse koruma onu geri kayma
    sanıp karşı komut basar ve iki katman birbirine karşı çalışır. Kurtarmanın
    tek işi aracı planlanamaz pozdan çıkarmak; engellenirse araç orada kalır.

    Bu yüzden karar komutu da görüyor: aşağı akışa geri komut gidiyorsa
    müdahale edilmez.

    `komut_bayat` iken KORUMA AÇIK kalır. Komutun ne olduğu bilinmiyorsa
    varsayım "istenmeyen kayma" olmalı; ters varsayım, komut yayını kesildiği
    anda korumayı sessizce kapatırdı — yani tam da korumaya en çok ihtiyaç
    duyulan durumda.
    """
    if not rollback_riskli(pitch_deg, velocity, pitch_esik_deg, hiz_esik):
        return False
    if komut_bayat:
        return True
    return komut_vx >= -geri_komut_esigi
