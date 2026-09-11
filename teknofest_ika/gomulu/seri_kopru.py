#!/usr/bin/env python3
"""
seri_kopru.py  —  Sürüş kartı ↔ Jetson binary seri köprüsü
===========================================================

Sistemdeki tek geçiş noktası: /odom, /imu/data, seri E-STOP, sürüş kipi ve
kart teşhis bayrakları hep buradan geçer. Karşı uç Nucleo-F767ZI; firmware
kaynağı paylaşılmıyor, dayanak elektrik ekibinin arayüz sözleşmesidir.

FİZİKSEL YOL
  /dev/f767 @ 921600 8N1 — kartın ST-LINK'i üzerinden tek USB kablosu; ayrı
  UART çekilmiyor. Symlink udev kuralıyla sabitlenir ve ham ttyACM* numarası
  her açılışta kayabildiği için ona doğrudan bağlanılmaz.

  Portu aynı anda iki süreç açamaz: bayt akışı okuyucular arasında bölüşülür
  ve iki taraf da hata vermeden bozuk veri görür. Kartın ASCII teşhis akışı
  bu yüzden köprü çalışırken susar; sıra tek yönlü, köprü kazanır.

ÇERÇEVE
  [0xAA][KOMUT][D0][D1][D2][D3][XOR][0x55] — 8 bayt sabit, big-endian
  int16 x 2. XOR = p[1]^p[2]^p[3]^p[4]^p[5].

KARTTAN GELEN — 0x30 bloğu (100 Hz; 0x3A 10 Hz, 0x3B 5 sn, 0x3C 1 Hz)
  0x30 PKT_F7_ENK        enkoder sayımı, int32 (v0 üst, v1 alt)
  0x31 PKT_F7_HIZ        ileri hız [mm/s] işaretli   | gösterge (hat iptal)
  0x32 PKT_F7_IMU_ACI    yaw x10 [derece]            | roll x10
  0x33 PKT_F7_IMU_PITCH  pitch x10 [derece]          | CALIB_STAT (sys/gyr/acc/mag)
  0x34 PKT_F7_ESTOP      basılı (0/1)                | uyuşmazlık (0/1)
  0x35 PKT_F7_SAGLIK     HATA_* bayrakları           | çalışma süresi [sn]
  0x36 PKT_F7_RC         gaz [binde, işaretli]       | DRM_* bayrakları
  0x37 PKT_F7_SURUS      gaz çıkışı [mV]             | fren [binde, işaretli]
  0x38 PKT_F7_JETSON     kartın anladığı hız [mm/s]  | anladığı açı [1/100°]
  0x39 PKT_F7_MOD        kip (0/1/2)                 | JDR_* bayrakları
  0x3A PKT_F7_RC_HAM     CH1 ham [µs]                | CH9 ham [µs]
  0x3B PKT_F7_SURUM      protokol sürümü             | firmware yapı no
  0x3C PKT_F7_HAT        alınan paket sayısı         | bozuk paket sayısı
  0x3D PKT_F7_BATARYA    Jetson paketi gerilimi [mV] | en düşük hücre [mV]
  0x3E PKT_F7_AYAR       ayar kimliği                | kartta duran değer

BİZDEN GİDEN
  0x01 PKT_J_SURUCU  hız [mm/s] işaretli · direksiyon [1/100°], ROS: + sol
  0x02 PKT_J_DUR     gaz rölanti + tam fren; KİLİT kurar, taze 0x01 çözer.
                     Direksiyon açısına dokunmaz — hareket hâlindeki araçta
                     tekerlekleri ortaya kırmak durdurmak değil, yön
                     değiştirmektir. Kart eski açıyı tutar.
                     ACİL DURUM paketidir, rutin durdurma aracı değil: kart
                     bunu jetson_dur olarak mandallıyor ve otonom kipte aynı
                     bayrak lazer isteğini düşürüp tareti merkeze döndürüyor.
                     Taret açıkken durdurmak için 0x01 hız 0 kullanılır.
  0x03 PKT_J_LAZER   0/1        0x05 PKT_J_PAN   açı [0-180]
  0x04 PKT_J_HB      heartbeat  0x06 PKT_J_TILT  açı [0-180]
  0x07 PKT_J_ESTOP   0/1        0x08 PKT_J_FREN  fren [binde 0-1000]
  0x09 PKT_J_AYAR    ayar kimliği + ölçekli değer; kart bunları FLASH'A
                     YAZMIYOR, RAM'de tutuyor. Kalıcılık burada: kart her
                     göründüğünde ve her resetinde yeniden gönderilir.

DAVRANIŞ
  Heartbeat penceresi 700 ms ve XOR'u tutan HER paket onu tazeler; 0x04'ün
  tek özelliği yan etkisinin olmaması. Bozuk çerçeveler bilerek tazelemez:
  hat bozulursa bayt akmaya devam etse bile link ölü sayılır ve kart güvenli
  tarafa düşer.

  Kip anahtarı (SwC/CH9) kartta okunur, kararı kart verir. Sıralama
  güvenlik > kumanda > Jetson; bizim komutumuz en altta.

  Fren kaynakları arasında büyük olan seçilir. Bizim 0x08'imiz işaretsizdir
  ve karta 0-1000 aralığında ulaşır: operatör fren ekleyebilir, bizimkini
  çözemez.

HIZ KISITI
  Kart otonom dalda komutu reddetmez, kırpar: tavan 1,50 m/s, taban 0,20 m/s,
  0,01 m/s altı rölanti. Sıfır olmayan ama tabanın altındaki komutlar tabana
  YÜKSELTİLİR — o aralıkta motor dönüyor ama araç kalkmıyordu. Sonucu:
  yavaşlama rampasının son bölümü 0,20'de takılır ve duruş oraya kadar
  basamaklıdır. Kırpma 0x38'de görünür.

ODOMETRİ
  Kaynak 0x31'in hazır mm/s'si; tick→metre çevrimi kartta. Tekerlek çevresi
  ve dişli oranı ölçülene kadar kart bu alanı bilerek 0 basar — uydurma
  mesafe yayınlamaktansa susuyor. Ham sayım (0x30) o sırada da akar ve
  ölçekten bağımsızdır; hareketin var olup olmadığı oradan görülür.

  Kart yeniden başladığında sayım sıfırlanır ve bunun başka izi yoktur;
  0x35 v1 çalışma süresi geri sararsa odometri temeli yenilenir.

IMU
  0x32/0x33 yalnız BNO takılıysa gelir, 50 Hz. Çip yokken paket hiç akmaz —
  susması arıza değildir. Eksen dönüşümü kart tarafında yapılır, açılar araç
  çerçevesinde gelir.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import (Bool, Int16, Int32, UInt8, UInt16,
                          Float32MultiArray, Int16MultiArray, UInt16MultiArray)
from tf2_ros import TransformBroadcaster
import serial

from teknofest_ika.otonomi.topics import (
    ACKERMANN_CMD_TOPIC, SHOOT_RESULT_TOPIC, TARET_PAN_TOPIC, TARET_TILT_TOPIC,
    ODOM_TOPIC, IMU_TOPIC, RC_INPUT_TOPIC, ENKODER_HAM_TOPIC,
    E_STOP_FORCE_SERIAL_TOPIC, E_STOP_TOPIC, SHOOT_CMD_TOPIC, FREN_KOMUT_TOPIC,
    SERIAL_ODOM, SERIAL_BAUD_KART,
    KART_KIP_TOPIC, KART_HATA_TOPIC, KART_DURUM_TOPIC, KART_LINK_TOPIC,
    KART_SURUM_TOPIC, KART_HAT_TOPIC, KART_SURUS_TOPIC, KART_CALISMA_TOPIC,
    KART_KABUL_TOPIC, BATTERY_JETSON_TOPIC, BMS_JETSON_UYARI_MV,
    KART_AYAR_TOPIC,
    HATA_GAZ_YOK, HATA_BNO_YOK, HATA_BNO_KALIB, HATA_ENK_SESSIZ,
    HATA_ESTOP_UYUSMAZ, HATA_RC_YOK, HATA_FREN_STALL, HATA_GOST_SESSIZ,
    DRM_KESME, DRM_TARET, DRM_LAZER,
    JDR_LINK,
    BEKLENEN_PROTOKOL_SURUMU, KART_HIZ_TAVAN, ATIS_AZAMI_S,
    KART_KIP_MANUEL, KART_KIP_OTONOM,
)
from teknofest_ika.otonomi.pure_logic import (
    paket_olustur, paket_dogrula,
    paket_v0_i, paket_v1_i, paket_v0_u, paket_v1_u, paket_int32,
    rc_dizisi, surum_uyumlu, enkoder_sessiz, yaw_kovaryansi,
    ayar_ham, ayar_deger, ayar_gonderilecek,
    lazer_sonmeli,
    calib_stat_coz,
    AYAR_CEVRE_MM, AYAR_DISLI_ORANI, AYAR_DIREKSIYON, AYAR_DARBE_TUR,
    AYAR_DIR_ISARET,
    RC_US_MIN, RC_US_NEUTRAL,
)


# ─── Çerçeve ───────────────────────────────────────────────────────────────
PKT_BOYUT = 8
PKT_BASLA = 0xAA
PKT_BITIS = 0x55

# Jetson → kart
PKT_J_SURUCU = 0x01
PKT_J_DUR    = 0x02
PKT_J_LAZER  = 0x03
PKT_J_HB     = 0x04
PKT_J_PAN    = 0x05
PKT_J_TILT   = 0x06
PKT_J_ESTOP  = 0x07
PKT_J_FREN   = 0x08
PKT_J_AYAR   = 0x09

# Kart → Jetson
PKT_F7_ENK       = 0x30
PKT_F7_HIZ       = 0x31
PKT_F7_IMU_ACI   = 0x32
PKT_F7_IMU_PITCH = 0x33
PKT_F7_ESTOP     = 0x34
PKT_F7_SAGLIK    = 0x35
PKT_F7_RC        = 0x36
PKT_F7_SURUS     = 0x37
PKT_F7_JETSON    = 0x38
PKT_F7_MOD       = 0x39
PKT_F7_RC_HAM    = 0x3A
PKT_F7_SURUM     = 0x3B
PKT_F7_HAT       = 0x3C
PKT_F7_BATARYA   = 0x3D
PKT_F7_AYAR      = 0x3E


# ─── Donanım güvenlik kısıtları ────────────────────────────────────────────
# ackermann_converter zaten kırpar; bunlar son savunma hattıdır. Üst akıştaki
# bir hata (Nav2, HizlanmaState, manuel override) aşırı bir değer üretse bile
# binary pakete bunun üstü yazılamaz.
MAX_DIREKSIYON = 30.0    # [derece]
# Kart otonom dalda hızı zaten kırpıyor; buradaki sınır onunla aynı tutulur.
# Daha yükseğini yazmak komutu gerçekleşmeyecek bir değere kilitler ve
# gönderdiğimizle 0x38'de okuduğumuz arasında kalıcı bir fark üretir.
MAX_HIZ_MS     = KART_HIZ_TAVAN
MAX_FREN_BINDE = 1000    # 0x08 işaretsizdir; negatif değeri kart 0'a kırpar

# Yayın kısıtı. Kart telemetriyi 100 Hz basıyor, ama bu konuların çoğu ya
# ayrık (kip, bayraklar) ya da yavaş değişiyor. Her paketi olduğu gibi ROS'a
# aktarmak saniyede yüzlerce mesaj demek ve bu, YOLO ile aynı makinede koşan
# bir grafikte bedava değil. Ayrık konular DEĞİŞİNCE yayınlanır; sonradan
# başlayan bir abone durumu kaçırmasın diye ayrıca saniyede bir tazelenir.
# Sürekli değerler sabit hızda örneklenir — /odom ve /imu/data bunun dışında,
# onları EKF sensör hızında istiyor.
DURUM_TAZELEME_S  = 1.0
SUREKLI_PERIYOT_S = 0.05   # 20 Hz

# 0x38 ile gönderdiğimiz 0x01 arasındaki kabul edilebilir fark. Kart komutu
# olduğu gibi geri yolladığı için normalde sıfır olmalı; eşik yalnız paketlerin
# farklı anlarda örneklenmesine pay bırakır.
SAPMA_HIZ_MMS = 150
SAPMA_ACI_CD  = 200


class SeriKopru(Node):

    def __init__(self):
        super().__init__('seri_kopru')

        self.declare_parameter('port',        SERIAL_ODOM)
        self.declare_parameter('baud',        SERIAL_BAUD_KART)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('sim_mode',    False)
        # odom → base_footprint dönüşümünü tek bir düğüm yayınlamalı. EKF
        # çalışırken (gercek_arac.launch.py) füzyon çıkışı otorite olur ve
        # burası kapatılır; iki yayıncı TF ağacını titretir.
        self.declare_parameter('publish_tf',  True)
        # Ham enkoder sayımını /enkoder/ham'a yayınlar. Ölçekten bağımsız
        # olduğu için tekerlek çevresi ölçülmeden de hareketi gösterir.
        self.declare_parameter('ham_enkoder', True)
        # Kart ayarları (0x09). Kart bunları flash'a yazmıyor; kalıcılık
        # burada. HEPSİ VARSAYILAN 0 = ÖLÇÜLMEDİ: sıfır olan ayar hiç
        # gönderilmez, yani hiçbiri girilmeden köprü bugünkü gibi davranır.
        # Sahada ölçüm sonrası:
        #   ros2 run teknofest_ika seri_kopru --ros-args -p tekerlek_cevre_mm:=1842.5
        self.declare_parameter('tekerlek_cevre_mm',   0.0)
        self.declare_parameter('enkoder_disli_orani', 0.0)
        self.declare_parameter('direksiyon_orani',    0.0)
        self.declare_parameter('gosterge_darbe_tur',  0.0)
        self.declare_parameter('direksiyon_isaret',   0.0)

        self._port        = self.get_parameter('port').value
        self._baud        = int(self.get_parameter('baud').value)
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._sim_mode    = bool(self.get_parameter('sim_mode').value)
        self._publish_tf  = bool(self.get_parameter('publish_tf').value)
        self._ham_enkoder = bool(self.get_parameter('ham_enkoder').value)
        self._ayarlar = {
            AYAR_CEVRE_MM:    float(self.get_parameter('tekerlek_cevre_mm').value),
            AYAR_DISLI_ORANI: float(self.get_parameter('enkoder_disli_orani').value),
            AYAR_DIREKSIYON:  float(self.get_parameter('direksiyon_orani').value),
            AYAR_DARBE_TUR:   float(self.get_parameter('gosterge_darbe_tur').value),
            AYAR_DIR_ISARET:  float(self.get_parameter('direksiyon_isaret').value),
        }
        self._ayar_gonderilen = {}   # kimlik → gönderdiğimiz ham değer

        self._ser  = None
        self._lock = threading.Lock()
        self._port_ac()

        # Odometri — yön kartın yaw'ından gelir, ilerleme 0x31'den
        self._x = 0.0
        self._y = 0.0
        self._yaw_rad     = 0.0
        self._yaw_onceki  = None
        self._hiz_zamani  = None
        self._vth         = 0.0
        self._son_cmd     = self.get_clock().now()

        # Kart durumu
        self._hata_bayrak  = 0
        self._drm_bayrak   = 0
        self._jdr_bayrak   = 0
        self._kip          = None    # ilk 0x39'a kadar bilinmiyor
        self._gaz_binde    = 0
        self._ham_ch1      = RC_US_NEUTRAL
        self._ham_ch9      = RC_US_MIN
        self._imu_yaw      = 0.0
        self._imu_roll     = 0.0
        self._estop_kart   = False
        self._estop_geldi  = False   # ilk 0x34'e kadar kaynak sessiz kalır
        self._surum_uyari  = False
        self._yapi         = None      # 0x3B v1 — değişimi izlenir
        self._son_surucu   = (0, 0)   # 0x38 karşılaştırması için
        self._kart_hiz     = 0         # 0x38'in bildirdiği hız [mm/s]
        self._sayim        = None
        self._sayim_zamani = 0.0       # sayımın son değiştiği an
        self._calisma_s    = None      # 0x35 v1 — kart reseti bu alandan görülür
        self._lazer_bildirim = False
        self._son_surus_yayin = 0.0
        self._son_kabul_yayin = 0.0
        self._son_ham_yayin   = 0.0

        qos_cmd  = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        # /odom ve /imu/data RELIABLE yayınlanır; EKF BEST_EFFORT abone olur.
        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(AckermannDriveStamped, ACKERMANN_CMD_TOPIC,
                                 self._cmd_cb, qos_cmd)
        self.create_subscription(Int16, TARET_PAN_TOPIC,
                                 lambda m: self._paket_gonder(PKT_J_PAN, m.data, 0), 10)
        self.create_subscription(Int16, TARET_TILT_TOPIC,
                                 lambda m: self._paket_gonder(PKT_J_TILT, m.data, 0), 10)
        self.create_subscription(UInt16, FREN_KOMUT_TOPIC, self._fren_cb, 10)
        self.create_subscription(Bool, SHOOT_CMD_TOPIC, self._shoot_cb, 10)
        self.create_subscription(Bool, E_STOP_TOPIC, self._e_stop_cb, 10)

        self._pub          = self.create_publisher(Odometry, ODOM_TOPIC, qos_odom)
        self._imu_pub      = self.create_publisher(Imu, IMU_TOPIC, qos_odom)
        self._tf           = TransformBroadcaster(self) if self._publish_tf else None
        self._ham_pub      = (self.create_publisher(Int32, ENKODER_HAM_TOPIC, qos_odom)
                              if self._ham_enkoder else None)
        self._rc_pub       = self.create_publisher(Float32MultiArray, RC_INPUT_TOPIC, 10)
        self._kip_pub      = self.create_publisher(UInt8,  KART_KIP_TOPIC,   10)
        self._hata_pub     = self.create_publisher(UInt16, KART_HATA_TOPIC,  10)
        self._durum_pub    = self.create_publisher(UInt16, KART_DURUM_TOPIC, 10)
        self._link_pub     = self.create_publisher(UInt16, KART_LINK_TOPIC,  10)
        self._surum_pub    = self.create_publisher(UInt16MultiArray, KART_SURUM_TOPIC, 10)
        self._hat_pub      = self.create_publisher(UInt16MultiArray, KART_HAT_TOPIC,   10)
        self._surus_pub    = self.create_publisher(Int16MultiArray, KART_SURUS_TOPIC, 10)
        self._calisma_pub  = self.create_publisher(UInt16, KART_CALISMA_TOPIC, 10)
        self._kabul_pub    = self.create_publisher(Int16MultiArray, KART_KABUL_TOPIC, 10)
        self._shoot_conf_pub = self.create_publisher(Bool, SHOOT_RESULT_TOPIC, 10)
        self._e_stop_force_pub = self.create_publisher(Bool, E_STOP_FORCE_SERIAL_TOPIC, 10)
        self._batarya_pub  = self.create_publisher(BatteryState, BATTERY_JETSON_TOPIC, 10)
        self._ayar_pub     = self.create_publisher(Int16MultiArray, KART_AYAR_TOPIC, 10)

        # Mesaj nesnesi yeniden kullanılır; alanları her yayında tazelenir.
        self._rc_msg = Float32MultiArray()

        self._lazer_aktif  = False
        self._lazer_zaman  = None   # kilidin KAPANDIĞI an (kenar, nabız değil)
        self._e_stop_aktif = False

        self._calisiyor = True
        threading.Thread(target=self._okuma_dongusu, daemon=True).start()

        # Kartın penceresi 700 ms ve XOR'u tutan her paket onu tazeliyor.
        # 400 ms'lik gönderim tek paketlik pay bırakıyordu: bir heartbeat
        # düşünce 800 ms > 700 ms olup kart Jetson'ı ölü sayıyordu. Sürüş
        # komutu aktığı sürece görünmeyen, ama komut akışının bilerek kesildiği
        # anlarda (atış duraklatması) gerçekten dar olan bir pay.
        self.create_timer(0.1,  self._hb_gonder)
        self.create_timer(0.1,  self._guvenlik_kontrol)
        self.create_timer(0.05, self._rc_yayinla)      # 20 Hz
        # E-STOP durumunu düzenli tekrarlar: e_stop_node köprüden sonra
        # başlarsa olay bazlı tek yayını kaçırır ve fiziksel butonun durumunu
        # hiç öğrenemez.
        self.create_timer(0.5,  self._estop_yayinla)
        self.create_timer(DURUM_TAZELEME_S, self._durum_tazele)
        self.create_timer(2.0,  self._port_ac)

        self.get_logger().info(
            f'SeriKopru hazır | {self._port} @ {self._baud} | '
            f'beklenen protokol sürümü {BEKLENEN_PROTOKOL_SURUMU}')

    # ── Seri port ──────────────────────────────────────────────────────────
    def _port_ac(self):
        """
        Portu açar; açıkken hiçbir şey yapmaz. Açılamazsa düğüm sessizce
        simülasyona düşmez — açılamayan port, hata vermeyen ölü bir köprü
        demektir ve bu arıza sınıfı sahada aylarca fark edilmedi. Hata
        kısılarak tekrarlanır ve deneme sürer.
        """
        if self._sim_mode or self._ser is not None:
            return
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.1,
                                      exclusive=True)
            self.get_logger().info(f'Seri port açıldı: {self._port} @ {self._baud}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(
                f'Seri port açılamadı ({self._port} @ {self._baud}): {e} — '
                'köprü veri alamıyor ve komut gönderemiyor.',
                throttle_duration_sec=5.0)

    def _port_kapat(self):
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    # ── /ackermann_cmd → 0x01 ──────────────────────────────────────────────
    def _cmd_cb(self, msg: AckermannDriveStamped):
        self._son_cmd = self.get_clock().now()

        if self._e_stop_aktif:
            return

        # Lazer aktifken hareket yok (Şartname: atış sırasında hareket -10)
        if self._lazer_aktif:
            self._atis_duraklat()
            return

        # Gaz yolu arızalıysa hız komutu göndermek anlamsız: kart kilidi
        # mandalladı, kilidi yalnız operatörün kumandadaki kesme anahtarı
        # çözer. Komut basmayı sürdürmek arızayı gizler.
        if self._hata_bayrak & HATA_GAZ_YOK:
            self._paket_gonder(PKT_J_DUR, 0, 0)
            return

        # Ayrılık ilkesi (Şartname §6.13/§7.8): bozuk bir mesaj alanı sessizce
        # yutulup hareketin son bilinen hızda takılı kalmasına izin verilmez.
        try:
            v         = msg.drive.speed
            delta_deg = math.degrees(msg.drive.steering_angle)
            delta_deg = max(-MAX_DIREKSIYON, min(MAX_DIREKSIYON, delta_deg))
            v         = max(-MAX_HIZ_MS, min(MAX_HIZ_MS, v))
        except Exception as exc:
            self.get_logger().error(f'_cmd_cb hata: {exc} — 0x02 gönderildi.')
            self._paket_gonder(PKT_J_DUR, 0, 0)
            return

        hiz_mms = int(v * 1000.0)
        aci_cd  = int(delta_deg * 100.0)
        with self._lock:
            self._son_surucu = (hiz_mms, aci_cd)
        self._paket_gonder(PKT_J_SURUCU, hiz_mms, aci_cd)

    def _atis_duraklat(self):
        """
        Atış sürerken aracı durdurur — kartın acil durum yolunu KURMADAN.

        PKT_J_DUR bu iş için kullanılamaz: kart onu `jetson_dur` olarak
        mandallıyor ve otonom kipte aynı bayrak lazer isteğini düşürüp tareti
        merkeze döndürüyor. Ateş komutu kendi ateşini iptal eder ve kilitli
        bir döngü kurulur — istek gelir, araç DUR'a düşer, taret ölür, atış
        onayı hiç gelmez, istek sürer, araç DUR'da kalır.

        Hız sıfırlanır; direksiyon SON KOMUTTA bırakılır, çünkü duran aracın
        tekerleklerini ortaya kırmak durdurmak değil yön değiştirmektir.
        Fren buradan basılmaz: /fren_komut'un sahibi ackermann_converter ve
        20 Hz yayın yapıyor, köprüden gönderilen bir fren 50 ms içinde
        üzerine yazılır. Atış boyunca tam fren o düğümde tutuluyor.
        """
        with self._lock:
            _, aci_cd = self._son_surucu
            self._son_surucu = (0, aci_cd)
        self._paket_gonder(PKT_J_SURUCU, 0, aci_cd)

    def _fren_cb(self, msg: UInt16):
        self._paket_gonder(PKT_J_FREN, max(0, min(MAX_FREN_BINDE, int(msg.data))), 0)

    def _shoot_cb(self, msg: Bool):
        """
        Atış isteğini karta iletir.

        🔴 AÇMA yalnız OTONOM kipte. Kip anahtarı operatörün elinde ve
        kural şu: SwC otonom dışındayken otonomi susar. Kart manuelde
        yalnız SÜRÜŞ komutlarını yok sayıyor; lazeri de yok sayıp saymadığı
        bilinmiyor, yani kapı burada olmazsa operatör kumandayı devraldıktan
        sonra ateş edilip edilmeyeceği karta kalıyor.
        Kapı köprüde çünkü karta açılan tek kapı burası: /shoot_command'a
        basan bugünkü ve gelecekteki her yayıncıyı birlikte kapsıyor.

        KAPATMA her kipte geçer. Ters kurmak, kip atış sırasında değişince
        lazeri açık bırakırdı.
        """
        if msg.data and self._kip != KART_KIP_OTONOM:
            self.get_logger().warn(
                f'Atış isteği reddedildi — kart otonom kipte değil '
                f'(kip={self._kip}). Lazer açılmadı.',
                throttle_duration_sec=2.0)
            return
        if msg.data and not self._lazer_aktif:
            self._lazer_zaman = self.get_clock().now()
        self._lazer_aktif = msg.data
        self._paket_gonder(PKT_J_LAZER, 1 if msg.data else 0, 0)
        self.get_logger().info(
            'Lazer AÇIK — hareket kilitlendi.' if msg.data
            else 'Lazer KAPALI — hareket serbest.')

    def _lazer_kip_denetle(self):
        """
        Kip otonomdan çıkarsa yanan lazeri söndürür.

        `_shoot_cb` yalnız mesaj geldiğinde işliyor; atış isteği tek bir True
        ile açılıp saniyelerce açık kalıyor. O pencerede operatör SwC'yi
        manuele alırsa giriş kapısı bir daha çağrılmaz ve lazer yanmaya devam
        eder — kapının periyodik bir eşi olmak zorunda.
        """
        if not lazer_sonmeli(self._lazer_aktif, self._kip, KART_KIP_OTONOM):
            return
        self._lazer_aktif = False
        self._lazer_zaman = None
        self._paket_gonder(PKT_J_LAZER, 0, 0)
        self.get_logger().warn(
            f'Kip otonomdan çıktı (kip={self._kip}) — lazer söndürüldü, '
            'hareket kilidi açıldı.')

    def _e_stop_cb(self, msg: Bool):
        onceki = self._e_stop_aktif
        self._e_stop_aktif = msg.data
        if msg.data and not onceki:
            self._paket_gonder(PKT_J_ESTOP, 1, 0)
            self._paket_gonder(PKT_J_DUR, 0, 0)
            self.get_logger().error('!!! E-STOP — tüm hareket durduruldu !!!')
        elif not msg.data and onceki:
            self._paket_gonder(PKT_J_ESTOP, 0, 0)
            self.get_logger().warn('[E-STOP] kaldırıldı — hareket izni verildi.')

    # ── Okuma döngüsü ──────────────────────────────────────────────────────
    def _okuma_dongusu(self):
        """0xAA'yı bekle, 7 bayt daha oku. Kablo gürültüsüne dayanıklı."""
        while self._calisiyor:
            ser = self._ser
            if ser is None:
                time.sleep(0.2)
                continue
            try:
                bas = ser.read(1)
                if not bas or bas[0] != PKT_BASLA:
                    continue
                kalan = ser.read(PKT_BOYUT - 1)
                if len(kalan) != PKT_BOYUT - 1:
                    continue
                ham = bytes([PKT_BASLA]) + kalan
                if not paket_dogrula(ham):
                    self.get_logger().warn(f'XOR hatası: {ham.hex()}',
                                           throttle_duration_sec=5.0)
                    continue
                self._paket_isle(ham)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Seri okuma hatası: {e}',
                                        throttle_duration_sec=5.0)
                self._port_kapat()
                time.sleep(0.2)

    # ── Gelen paket ────────────────────────────────────────────────────────
    def _paket_isle(self, ham: bytes):
        komut = ham[1]
        now   = self.get_clock().now()

        if komut == PKT_F7_ENK:
            sayim = paket_int32(ham)
            if sayim != self._sayim:
                self._sayim        = sayim
                self._sayim_zamani = now.nanoseconds * 1e-9
            if self._ham_pub is not None and self._sureklide_zamani(
                    now, '_son_ham_yayin'):
                self._ham_pub.publish(Int32(data=sayim))

        elif komut == PKT_F7_HIZ:
            self._odometri(now, paket_v0_i(ham) / 1000.0)

        elif komut == PKT_F7_IMU_ACI:
            with self._lock:
                self._imu_yaw  = paket_v0_i(ham) / 10.0
                self._imu_roll = paket_v1_i(ham) / 10.0

        elif komut == PKT_F7_IMU_PITCH:
            pitch = paket_v0_i(ham) / 10.0
            kalib = paket_v1_u(ham) & 0xFF
            with self._lock:
                yaw, roll = self._imu_yaw, self._imu_roll
            self._imu_yayinla(now, yaw, pitch, roll, kalib)

        elif komut == PKT_F7_ESTOP:
            basili   = (paket_v0_i(ham) != 0)
            uyusmaz  = (paket_v1_i(ham) != 0)
            onceki, self._estop_kart = self._estop_kart, basili
            ilk, self._estop_geldi = not self._estop_geldi, True
            if basili != onceki or ilk:
                self._e_stop_force_pub.publish(Bool(data=basili))
                if basili:
                    self.get_logger().error('!!! E-STOP (kart) — buton basıldı !!!')
                else:
                    self.get_logger().warn('[E-STOP] kart: buton bırakıldı.')
            if uyusmaz:
                self.get_logger().error(
                    'Kart iki E-STOP okumasını çelişkili bildiriyor.',
                    throttle_duration_sec=5.0)

        elif komut == PKT_F7_SAGLIK:
            self._saglik_isle(paket_v0_u(ham))
            self._calisma_isle(paket_v1_u(ham))

        elif komut == PKT_F7_RC:
            self._gaz_binde = paket_v0_i(ham)
            self._drm_isle(paket_v1_u(ham))

        elif komut == PKT_F7_SURUS:
            if self._sureklide_zamani(now, '_son_surus_yayin'):
                self._surus_pub.publish(
                    Int16MultiArray(data=[paket_v0_i(ham), paket_v1_i(ham)]))

        elif komut == PKT_F7_JETSON:
            self._kart_hiz = paket_v0_i(ham)
            if self._sureklide_zamani(now, '_son_kabul_yayin'):
                self._kabul_pub.publish(
                    Int16MultiArray(data=[paket_v0_i(ham), paket_v1_i(ham)]))
            self._geri_bildirim(paket_v0_i(ham), paket_v1_i(ham))
            self._enkoder_denetle(now)

        elif komut == PKT_F7_MOD:
            self._kip_isle(paket_v0_i(ham), paket_v1_u(ham))

        elif komut == PKT_F7_RC_HAM:
            self._ham_ch1 = float(paket_v0_u(ham))
            self._ham_ch9 = float(paket_v1_u(ham))

        elif komut == PKT_F7_SURUM:
            self._surum_isle(paket_v0_u(ham), paket_v1_u(ham))

        elif komut == PKT_F7_HAT:
            self._hat_pub.publish(UInt16MultiArray(data=[paket_v0_u(ham), paket_v1_u(ham)]))

        elif komut == PKT_F7_BATARYA:
            self._batarya_isle(now, paket_v0_u(ham), paket_v1_u(ham))

        elif komut == PKT_F7_AYAR:
            self._ayar_isle(paket_v0_i(ham), paket_v1_i(ham))

    # ── 0x35 arıza bayrakları ──────────────────────────────────────────────
    def _saglik_isle(self, bayraklar: int):
        onceki, self._hata_bayrak = self._hata_bayrak, bayraklar
        if bayraklar != onceki:
            self._hata_pub.publish(UInt16(data=bayraklar))
        yeni = bayraklar & ~onceki
        if not yeni:
            return

        if yeni & HATA_GOST_SESSIZ:
            # Gösterge ucu ileri hızın kaynağı: sustuğunda 0x31 sıfıra düşer,
            # EKF konumu ilerlemez ve Nav2 hedefi "ilerleme yok" diye iptal
            # eder — yani otonom koşu bu tek bayrağın arkasında durur.
            # Enkoder sessizliğinde olduğu gibi burada da yalnız uyarılır:
            # karşı bir davranış bağlamak, kaynağı henüz bağlanmamış bir
            # araçta sürüşü kendi kendine kilitlerdi.
            self.get_logger().error(
                'KART: gösterge ucu darbe basmıyor — ileri hız kaynağı sustu. '
                '/odom sıfırda kalır, Nav2 ilerleme göremez.')
        if yeni & HATA_GAZ_YOK:
            self.get_logger().error(
                'KART: gaz DAC ulaşılamıyor. DAC son yazılan değeri tutar, '
                'kilit MANDALLI — Jetson kurtaramaz. Kilidi yalnız kumandadaki '
                'kesme anahtarı (SwA → KES) çözer.')
        if yeni & HATA_FREN_STALL:
            # Kilit mandallı ve yalnız fren YÖN DEĞİŞTİRİNCE düşüyor; arıza
            # geçse bile kendiliğinden temizlenmiyor. Sıfır komutu yön
            # değişimi sayıldığı için kilidi bu çözüyor.
            self.get_logger().error(
                'KART: fren aktüatörü aynı yönde takıldı — kilidi çözmek için '
                'fren sıfırlanıyor.')
            self._paket_gonder(PKT_J_FREN, 0, 0)
        if yeni & HATA_ESTOP_UYUSMAZ:
            self.get_logger().error('KART: E-STOP okumaları çelişiyor.')
        if yeni & HATA_RC_YOK:
            # Kopuk kabloyu yakalar, kapanan vericiyi yakalamaz: alıcı verici
            # kapalıyken de yayın sürdürüyor. Sinyal kaybının görünür olduğu
            # tek yer alıcının failsafe kaydı, o da DRM_KESME'ye düşüyor.
            self.get_logger().error('KART: iBUS çerçevesi gelmiyor — kumanda kablosu.')
        if yeni & HATA_BNO_YOK:
            self.get_logger().error('KART: IMU cevap vermiyor.')
        if yeni & HATA_BNO_KALIB:
            self.get_logger().warn('KART: IMU kalibrasyonu yetersiz.')
        if yeni & HATA_ENK_SESSIZ:
            self.get_logger().warn('KART: araç hareket ederken enkoder kımıldamıyor.')

    # ── 0x36 v1 sürüş durum bayrakları ─────────────────────────────────────
    def _drm_isle(self, bayraklar: int):
        onceki, self._drm_bayrak = self._drm_bayrak, bayraklar
        if bayraklar != onceki:
            self._durum_pub.publish(UInt16(data=bayraklar))
        if (bayraklar & DRM_KESME) and not (onceki & DRM_KESME):
            self.get_logger().error('KUMANDA: kesme anahtarı (SwA) KES konumunda.')
        elif (onceki & DRM_KESME) and not (bayraklar & DRM_KESME):
            self.get_logger().warn('KUMANDA: kesme kaldırıldı.')

        # Lazer durumu: bit komutun UYGULANDIĞINI bildirir, lazerin yandığını
        # ölçmez — donanımda akım ya da foto geri beslemesi yok. Atış onayı
        # "röle sürüldü" anlamındadır.
        lazer = bool(bayraklar & DRM_LAZER)
        if lazer != self._lazer_bildirim:
            self._lazer_bildirim = lazer
            self._shoot_conf_pub.publish(Bool(data=lazer))

    # ── 0x35 v1 çalışma süresi ─────────────────────────────────────────────
    def _calisma_isle(self, saniye: int):
        """
        Kartın çalışma süresi geri sararsa kart yeniden başlamıştır. Bunun
        başka görünür izi yok ve enkoder sayımı da o anda sıfırlanıyor —
        sayaç sıçraması gerçek hareketle karıştırılmasın diye odometri
        temeli burada bırakılır.
        """
        onceki, self._calisma_s = self._calisma_s, saniye
        self._calisma_pub.publish(UInt16(data=saniye))
        if onceki is None:
            # Kart ilk kez görüldü: köprü ondan sonra da başlamış olabilir,
            # ayarların gitmiş olduğuna güvenilmez.
            self._ayarlari_gonder('kart ilk görüldü')
        elif saniye < onceki:
            self.get_logger().error(
                f'KART YENİDEN BAŞLADI — çalışma süresi {onceki} s → {saniye} s. '
                'Enkoder sayımı sıfırlandı, odometri temeli yenileniyor.')
            with self._lock:
                self._hiz_zamani = None
                self._yaw_onceki = None
            self._sayim = None
            # Ayarlar RAM'deydi, resette gittiler. Yeniden yazılmazsa hız
            # alanı sessizce 0 basar ve otonomi ortasından kesilir.
            self._ayar_gonderilen.clear()
            self._ayarlari_gonder('kart resetlendi')

    # ── Enkoder sessizliği ─────────────────────────────────────────────────
    def _enkoder_denetle(self, now):
        """
        Kart hareket ettiğini söylerken sayım duruyorsa uyarır. Kartın kendi
        bayrağı (HATA_ENK_SESSIZ) karşılaştıracak ikinci hız kaynağı
        kalmadığı için hiç kalkmıyor; denetim bu yüzden burada.
        """
        if self._sayim is None:
            return
        sabit_s = now.nanoseconds * 1e-9 - self._sayim_zamani
        if enkoder_sessiz(self._kart_hiz, sabit_s):
            self.get_logger().error(
                f'ENKODER SESSİZ — kart {self._kart_hiz} mm/s sürüyor ama sayım '
                f'{sabit_s:.1f} s\'dir sabit. Kaplin, kablo ya da sayaç.',
                throttle_duration_sec=5.0)

    # ── 0x39 kip ve link ───────────────────────────────────────────────────
    def _kip_isle(self, kip: int, jdr: int):
        onceki_kip, self._kip = self._kip, kip
        if kip != onceki_kip:
            self._kip_pub.publish(UInt8(data=max(0, min(255, kip))))
            self.get_logger().info(f'KART: sürüş kipi {onceki_kip} → {kip}')

        onceki_jdr, self._jdr_bayrak = self._jdr_bayrak, jdr
        if jdr != onceki_jdr:
            self._link_pub.publish(UInt16(data=jdr))
        if not (jdr & JDR_LINK):
            # Kart bizi canlı görmüyor: gönderdiğimiz her komut yok sayılıyor.
            self.get_logger().error(
                'KART: JDR_LINK düşük — kart Jetson\'ı canlı görmüyor, '
                'komutlarımız yok sayılıyor.', throttle_duration_sec=5.0)
        elif not (onceki_jdr & JDR_LINK):
            self.get_logger().info('KART: link kuruldu, komutlarımız görülüyor.')

    # ── 0x38 geri bildirim ─────────────────────────────────────────────────
    def _geri_bildirim(self, hiz_mms: int, aci_cd: int):
        """
        Kart anladığı komutu geri yollar. Gönderdiğimizle farkı tek başına
        teşhistir: ölçek hatası, işaret hatası ve kayıp paket burada görünür.

        Karşılaştırma YALNIZ otonom kipte anlamlı: manuel kipte kart tasarım
        gereği Jetson'ın sürüş komutunu yok sayıyor ve 0x38'de kendi
        uyguladığını (kumandadan geleni) yolluyor. Orada uyuşmazlık beklenen
        durumdur; uyarmak gerçek ölçek ve işaret hatalarının arasına sürekli
        bir gürültü katar.
        """
        with self._lock:
            gonderilen_hiz, gonderilen_aci = self._son_surucu
            kip = self._kip
        if kip != KART_KIP_OTONOM:
            return
        if (abs(hiz_mms - gonderilen_hiz) > SAPMA_HIZ_MMS or
                abs(aci_cd - gonderilen_aci) > SAPMA_ACI_CD):
            self.get_logger().warn(
                f'Komut uyuşmuyor — gönderilen ({gonderilen_hiz} mm/s, '
                f'{gonderilen_aci} cd), kartın anladığı ({hiz_mms} mm/s, '
                f'{aci_cd} cd).', throttle_duration_sec=5.0)

    # ── 0x09 kart ayarları ─────────────────────────────────────────────────
    def _ayarlari_gonder(self, sebep: str):
        """
        Ölçülmüş ayarları karta yazar.

        Kart bu sayıları flash'a yazmıyor, RAM'de tutuyor: her resetinde
        hepsi sıfırlanıyor ve 0x31 hız alanı sessizce 0 basmaya dönüyor —
        görünür bir belirtisi olmayan, koşuyu ortasından kesen bir arıza.
        Kalıcılık bu yüzden kartta değil burada: kart her göründüğünde ve
        her resetinde ayarlar yeniden gönderilir.

        Ölçülmemiş ayar (0) GÖNDERİLMEZ. Kartın bilerek sustuğu bir alana
        uydurma bir sayı yazmak, susmasından kötüdür.
        """
        yazilan = 0
        for kimlik, deger in sorted(self._ayarlar.items()):
            if not ayar_gonderilecek(deger):
                continue
            ham, gecerlilik = ayar_ham(kimlik, deger)
            if ham is None:
                self.get_logger().error(
                    f'Ayar {kimlik} gönderilemedi ({gecerlilik}): {deger}')
                continue
            self._ayar_gonderilen[kimlik] = ham
            self._paket_gonder(PKT_J_AYAR, kimlik, ham)
            yazilan += 1
        if yazilan:
            self.get_logger().info(
                f'Kart ayarları gönderildi ({sebep}) — {yazilan} alan.')
        else:
            self.get_logger().warn(
                f'Kart ayarı gönderilmedi ({sebep}): hiçbiri ölçülmemiş. '
                'Tekerlek çevresi girilmeden kart hız alanını 0 basar.',
                throttle_duration_sec=60.0)

    def _ayar_isle(self, kimlik: int, ham: int):
        """
        Kartın "bende şu duruyor" cevabı. Gönderdiğimizle karşılaştırmak tek
        başına teşhistir: ölçek hatası, kırpma ve kayıp paket burada görünür.
        """
        self._ayar_pub.publish(Int16MultiArray(data=[kimlik, ham]))
        beklenen = self._ayar_gonderilen.get(kimlik)
        if beklenen is not None and ham != beklenen:
            self.get_logger().error(
                f'AYAR UYUŞMUYOR — kimlik {kimlik}: gönderdiğimiz {beklenen}, '
                f'kartta duran {ham} (ölçekli: {ayar_deger(kimlik, ham)}).',
                throttle_duration_sec=10.0)

    # ── 0x3D Jetson besleme paketi ─────────────────────────────────────────
    def _batarya_isle(self, stamp, paket_mv: int, dip_mv: int):
        """
        Jetson'ı besleyen DALY paketi. Traksiyon paketinden ayrı konuda
        yayınlanır: kimyaları farklı, eşikleri farklı ve biri bitince
        ötekinden hiçbir işaret gelmiyor.

        Paket veri üretemiyorsa kart 0x3D'yi HİÇ göndermiyor; sessizlik
        "ölçüm yok" demek, "0 V" demek değil. Bu yüzden burada sahte bir
        değer üretilmez, yalnız gelen yayınlanır.

        Sağlık kararı paket geriliminden değil EN DÜŞÜK HÜCREDEN verilir:
        paket gerilimi ortalamadır ve çökmüş tek bir hücreyi gizler.
        """
        m = BatteryState()
        m.header.stamp = stamp.to_msg()
        m.voltage      = paket_mv / 1000.0
        m.cell_voltage = [dip_mv / 1000.0]
        m.present      = True
        m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # Kimya teyit edilmedi; POWER_SUPPLY_TECHNOLOGY_UNKNOWN doğrusu.
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        if dip_mv < BMS_JETSON_UYARI_MV:
            m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            self.get_logger().error(
                f'JETSON BESLEME PAKETİ DÜŞÜK — en düşük hücre {dip_mv} mV '
                f'(eşik {BMS_JETSON_UYARI_MV} mV), paket {paket_mv} mV. '
                'Bu paket bittiğinde sürüş kartı da düşer.',
                throttle_duration_sec=10.0)
        else:
            m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self._batarya_pub.publish(m)

    # ── 0x3B sürüm ─────────────────────────────────────────────────────────
    def _surum_isle(self, protokol: int, yapi: int):
        """
        Protokol sürümü beklenen değerle karşılaştırılır; firmware yapı
        numarası karşılaştırılmaz, DEĞİŞİMİ izlenir.

        Ayrım kasıtlı: yapı numarası davranış değiştiren her yüklemede artıyor
        ve sabit bir değerle karşılaştırmak ilk yüklemede sahte alarm verirdi.
        Değişimi ise haber verilmemiş bir firmware güncellemesinin tek görünür
        izi — kartta bir sabit (hız tavanı, zaman aşımı, ölçek) değişmişse
        bunu başka hiçbir alan bildirmiyor.
        """
        self._surum_pub.publish(UInt16MultiArray(data=[protokol, yapi]))

        onceki_yapi, self._yapi = self._yapi, yapi
        if onceki_yapi is not None and yapi != onceki_yapi:
            self.get_logger().warn(
                f'KART FIRMWARE DEĞİŞTİ — yapı {onceki_yapi} → {yapi}. '
                'Davranış değiştiren bir yükleme yapılmış olabilir; otonom '
                'sürmeden önce hangi sabitin değiştiğini elektrik ekibine sorun.')
        if not surum_uyumlu(protokol, BEKLENEN_PROTOKOL_SURUMU):
            self._surum_uyari = True
            self.get_logger().error(
                f'PROTOKOL SÜRÜMÜ UYUŞMUYOR — kart {protokol}, köprü '
                f'{BEKLENEN_PROTOKOL_SURUMU} bekliyor. Paket anlamları değişmiş '
                'olabilir; sürüş sürdürülüyor ama telemetri yanlış okunuyor '
                'olabilir.', throttle_duration_sec=10.0)
        elif self._surum_uyari:
            self._surum_uyari = False
            self.get_logger().info(f'Protokol sürümü uyuştu ({protokol}).')
        else:
            self.get_logger().info(
                f'Kart protokol sürümü {protokol}, firmware yapı {yapi}.',
                throttle_duration_sec=60.0)

    # ── Odometri ───────────────────────────────────────────────────────────
    def _odometri(self, now, vx: float):
        now_sec = now.nanoseconds * 1e-9
        with self._lock:
            yaw_deg = self._imu_yaw
            onceki  = self._hiz_zamani
            self._hiz_zamani = now_sec
            yaw = math.radians(yaw_deg)

            if onceki is None:
                self._yaw_rad    = yaw
                self._yaw_onceki = yaw
                return

            dt = now_sec - onceki
            # NTP sıçraması, suspend/resume gibi saat anomalilerine karşı
            if not (0.0 < dt < 1.0):
                self._yaw_rad    = yaw
                self._yaw_onceki = yaw
                return

            d_yaw = _normalize(yaw - self._yaw_onceki)
            self._yaw_onceki = yaw
            self._vth = d_yaw / dt

            ds = vx * dt
            self._x += ds * math.cos(self._yaw_rad + d_yaw / 2.0)
            self._y += ds * math.sin(self._yaw_rad + d_yaw / 2.0)
            self._yaw_rad = yaw
            vth = self._vth

        self._yayinla(now, vx, vth)

    def _yayinla(self, stamp, vx, vth):
        qz = math.sin(self._yaw_rad / 2.0)
        qw = math.cos(self._yaw_rad / 2.0)
        t  = stamp.to_msg()

        if self._tf is not None:
            tf = TransformStamped()
            tf.header.stamp            = t
            tf.header.frame_id         = 'odom'
            tf.child_frame_id          = 'base_footprint'   # ekf.yaml ile eşleşmeli
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp            = t
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.angular.z   = vth
        odom.pose.covariance[0]   = 0.01
        odom.pose.covariance[7]   = 0.01
        odom.pose.covariance[35]  = 0.05
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.05
        self._pub.publish(odom)

    # ── /imu/data ──────────────────────────────────────────────────────────
    def _imu_yayinla(self, stamp, yaw_deg: float, pitch_deg: float,
                     roll_deg: float, kalib: int):
        """
        Kart ZYX Euler açılarını derece olarak veriyor: q = Rz(ψ)·Ry(θ)·Rx(φ).

        Kalibrasyon baytı çipin `CALIB_STAT`'ı: bit 7-6 sys, 5-4 gyr, 3-2 acc,
        1-0 mag. Sistem kalibrasyonu kartın eşiğinin altındayken yön güvenilmez
        ve kovaryans gevşetilir. sys 3 iken manyetometrenin 0 kalması normaldir,
        kalibrasyon ayrı ilerliyor.

        ⚠️ YAW İŞARETİ ÇEVRİLMİYOR: kartın verdiği açı olduğu gibi kuaterniyona
        giriyor. ROS yaw'ı SOLA doğru artar (REP-103); BNO055'in kendi yönü
        saat yönündedir. Kartın çevirip çevirmediği SAHADA DOĞRULANMADI —
        otonoma alıp aracı elle sola çevirince yaw artmalı. Yanlışsa yön tutan
        her denetleyici pozitif geri beslemeye döner.
        """
        φ = math.radians(roll_deg)
        θ = math.radians(pitch_deg)
        ψ = math.radians(yaw_deg)

        cψ, sψ = math.cos(ψ / 2), math.sin(ψ / 2)
        cθ, sθ = math.cos(θ / 2), math.sin(θ / 2)
        cφ, sφ = math.cos(φ / 2), math.sin(φ / 2)

        msg = Imu()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = cφ * cθ * cψ + sφ * sθ * sψ
        msg.orientation.x = sφ * cθ * cψ - cφ * sθ * sψ
        msg.orientation.y = cφ * sθ * cψ + sφ * cθ * sψ
        msg.orientation.z = cφ * cθ * sψ - sφ * sθ * cψ

        sys_kalib, _gyr, _acc, _mag = calib_stat_coz(kalib)
        yaw_var   = yaw_kovaryansi(sys_kalib)
        msg.orientation_covariance[0] = 0.005   # roll  σ² [rad²]
        msg.orientation_covariance[4] = 0.005   # pitch σ²
        msg.orientation_covariance[8] = yaw_var

        # Kart ham jiroskop ve ivme göndermiyor → EKF'e "bu alanı kullanma"
        # sinyali: kovaryans[0] = -1.
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    # ── /rc_input uyumluluk yayını ─────────────────────────────────────────
    def _rc_yayinla(self):
        """
        Dizinin şekli mod_yoneticisi, taret_rc_koprusu ve pano için korunuyor.
        Kart ham kanalları göndermediği için alanlar en yakın karşılıkla
        doldurulur:
          [0] gaz          ← 0x36 v0 (binde) µs'ye ölçeklenir
          [1] direksiyon   ← 0x3A v0, ham CH1
          [2] kip          ← kartın çözdüğü kipten türetilir
          [3] aux/lazer    ← karşılığı yok, kapalı tutulur
          [4] tilt         ← karşılığı yok, nötr
          [5] taret aktif  ← DRM_TARET (SwB)
          [6] ham CH9      ← 0x3A v1, teşhis

        [2] bilerek ham CH9 değildir: kanal üç konumlu ve orta konumu mod
        eşiğinin üstüne düşüyor, yani ham değeri eşikleyen taraf kullanılmayan
        orta kipi otonom okurdu. Kullanılmayan kip güvenli tarafa yazılır.
        """
        self._rc_msg.data = rc_dizisi(
            gaz_binde   = self._gaz_binde,
            ham_ch1     = self._ham_ch1,
            # Kart henüz kip bildirmediyse manuel varsayılır: bilinmeyeni
            # otonom saymak, kartın manuelde sürdüğü araca otonom komut
            # basmaya dönüşür.
            kip         = self._kip if self._kip is not None else KART_KIP_MANUEL,
            taret_aktif = bool(self._drm_bayrak & DRM_TARET),
            ham_ch9     = self._ham_ch9,
            otonom_kip  = KART_KIP_OTONOM,
        )
        self._rc_pub.publish(self._rc_msg)

    def _sureklide_zamani(self, now, alan: str) -> bool:
        """Sürekli bir değerin yayın sırası geldi mi."""
        simdi = now.nanoseconds * 1e-9
        if simdi - getattr(self, alan) < SUREKLI_PERIYOT_S:
            return False
        setattr(self, alan, simdi)
        return True

    def _durum_tazele(self):
        """
        Ayrık durum konuları yalnız değişince yayınlanıyor; köprüden sonra
        başlayan bir abone o değişimi kaçırır ve kartın durumunu hiç öğrenemez.
        Periyodik tekrar bu boşluğu kapatıyor.
        """
        if self._kip is not None:
            self._kip_pub.publish(UInt8(data=max(0, min(255, self._kip))))
        self._hata_pub.publish(UInt16(data=self._hata_bayrak))
        self._durum_pub.publish(UInt16(data=self._drm_bayrak))
        self._link_pub.publish(UInt16(data=self._jdr_bayrak))

    def _estop_yayinla(self):
        # Kart durumunu bildirmeden "basılı değil" yayınlamak, veri yokluğunu
        # olumlu bir cevaba çevirirdi. Kaynak ilk 0x34'e kadar susar.
        if self._estop_geldi:
            self._e_stop_force_pub.publish(Bool(data=self._estop_kart))

    # ── Giden ──────────────────────────────────────────────────────────────
    def _hb_gonder(self):
        self._paket_gonder(PKT_J_HB, 0, 0)

    def _atis_kilidi_denetle(self):
        """
        Atış kilidini süreyle sınırlar.

        Kilit açıkken her sürüş komutu sıfır hıza çevriliyor. /shoot_command
        nabız değil kenar sinyali: True ile False ayrı olaylar ve ikisinin
        arasında yayınlayan düğüm ölürse kilidi açacak kimse kalmaz — araç
        bir daha hiç hareket etmez ve bunun tek belirtisi aracın durması
        olur, log'da bir satır bile çıkmaz. Kilidi yalnız disiplinle
        (misyon_fsm'in False yayınlamayı unutmaması) korumak yetmiyor.

        Kilit düşerken lazer de kapatılıyor: sahibi ölmüş bir atış isteğinin
        lazeri süresiz yakık bırakması, kilidi açmaktan daha kötüdür.
        """
        if not self._lazer_aktif:
            return
        if self._lazer_zaman is None:
            self._lazer_zaman = self.get_clock().now()
            return
        yas = (self.get_clock().now() - self._lazer_zaman).nanoseconds * 1e-9
        if yas <= ATIS_AZAMI_S:
            return
        self._lazer_aktif = False
        self._lazer_zaman = None
        self._paket_gonder(PKT_J_LAZER, 0, 0)
        self.get_logger().error(
            f'ATIŞ KİLİDİ {yas:.1f}s sürdü (sınır {ATIS_AZAMI_S:.0f}s) — '
            'kilit açıldı ve lazer kapatıldı. /shoot_command kapatma komutu '
            'hiç gelmedi; yayınlayan düğüm ölmüş olabilir.')

    def _guvenlik_kontrol(self):
        self._lazer_kip_denetle()
        self._atis_kilidi_denetle()
        # E-STOP gerçek bir acil durum: taretin sönmesi ve merkeze dönmesi
        # istenen sonuçtur, PKT_J_DUR burada doğru pakettir.
        if self._e_stop_aktif:
            self._paket_gonder(PKT_J_DUR, 0, 0)
            return
        dt = (self.get_clock().now() - self._son_cmd).nanoseconds * 1e-9
        if dt > self._cmd_timeout:
            # Atış sürerken üst akış komut basmayı bilerek bırakıyor; bu
            # sessizliği DUR'a çevirmek tareti öldürür (bkz. _atis_duraklat).
            if self._lazer_aktif:
                self._atis_duraklat()
            else:
                self._paket_gonder(PKT_J_DUR, 0, 0)

    def _paket_gonder(self, komut: int, v0: int, v1: int):
        pkt = paket_olustur(komut, v0, v1)
        ser = self._ser
        if self._sim_mode or ser is None:
            self.get_logger().debug(
                f'[SIM] 0x{komut:02X} v0={v0} v1={v1} → {pkt.hex()}')
            return
        # Kapanmış porta yazmak SerialException DEĞİL, TypeError üretiyor:
        # pyserial kapanırken iç iptal borusunu None yapıyor ve select() onu
        # dosya tanıtıcısı sanıp patlıyor. Yarış penceresi dar ama gerçek —
        # kart her firmware yüklemesinde resetleniyor, okuma döngüsü portu
        # kapatıyor ve /cmd_vel geri çağrısı aynı anda yazmaya devam ediyor.
        # Yakalanmayan istisna düğümü öldürüyordu; açılış betiği düğümleri
        # denetlemediği için köprü bir daha kendiliğinden dönmüyor ve boşta
        # kalan portu f767_telemetri kapıyor. Bu yüzden hem önden denetim
        # hem geniş ağ var: burada ölmek, bir paketi kaybetmekten pahalı.
        try:
            if not ser.is_open:
                self._port_kapat()
                return
            ser.write(pkt)
        except (serial.SerialException, OSError, TypeError, AttributeError) as e:
            self.get_logger().warn(f'Seri yazma hatası: {e}',
                                   throttle_duration_sec=5.0)
            self._port_kapat()

    def destroy_node(self):
        self._calisiyor = False
        ser = self._ser
        if ser is not None and ser.is_open:
            try:
                ser.write(paket_olustur(PKT_J_DUR, 0, 0))
            except Exception:
                pass
        self._port_kapat()
        super().destroy_node()


def _normalize(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


def main(args=None):
    rclpy.init(args=args)
    node = SeriKopru()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
