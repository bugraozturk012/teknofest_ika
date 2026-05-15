"""
veri_paketi.py — LYDİA İKA Veri Paketi Kaydedici
===================================================
Şartname 6.14: Parkur boyunca 3 kameradan görüntü kaydı.
  - İleri sürüş kamerası  → /ileri_kamera/image_raw   (URDF: ileri_kamera_joint)
  - Geri sürüş kamerası   → /geri_kamera/image_raw    (URDF: geri_kamera_joint)
  - Nişan kamerası        → /nisan_kamera/image_raw   (URDF: nisan_kamera_joint)

Kullanım:
  ros2 run teknofest_ika veri_paketi

Kayıt kontrolü:
  ros2 topic pub /veri_paketi/kayit_baslat std_msgs/msg/Bool "data: true"
  ros2 topic pub /veri_paketi/kayit_baslat std_msgs/msg/Bool "data: false"

Kaydedilen dosyalar:
  ~/ika_kayitlar/<YYYY-MM-DD_HH-MM-SS>/
      ileri.mp4
      geri.mp4
      nisan.mp4
      meta.txt   ← zaman damgası, FPS, çözünürlük, kare sayısı

NOT: Kamera topic adları araçtaki gerçek topic'lere göre
     aşağıdaki TOPIC_* sabitlerinden ayarlanmalıdır.
"""

import os
import time
import threading
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
    CV_OK = True
except ImportError:
    CV_OK = False

from teknofest_ika.otonomi.topics import (
    CAMERA_FRONT_TOPIC as TOPIC_ILERI,
    CAMERA_REAR_TOPIC  as TOPIC_GERI,
    CAMERA_TARET_TOPIC as TOPIC_NISAN,
    MISYON_AKTIF_TOPIC, KAYIT_BASLAT_TOPIC, KAYIT_DURUMU_TOPIC,
)

KAYIT_DIZIN = os.path.expanduser('~/ika_kayitlar')  # Kayıt ana dizini
VIDEO_FPS   = 30                                     # Kamera topic frekansıyla eşleşmeli
VIDEO_CODEC = 'mp4v'                                 # MP4 codec
HEDEF_GENISLIK  = 640                                # Çıkış genişliği (px)
HEDEF_YUKSEKLIK = 480                                # Çıkış yüksekliği (px)

# ─── Kamera Kaydedici Yardımcı Sınıf ───────────────────────────────────────

class KameraKaydedici:
    """Tek bir kamera topic'ini MP4 dosyasına kaydeder."""

    def __init__(self, ad: str):
        self.ad   = ad
        self._writer  = None
        self._lock    = threading.Lock()
        self.kare_say = 0
        self.baslama  = None
        self._aktif   = False

    def baslat(self, kayit_yolu: str):
        with self._lock:
            if self._aktif:
                return
            fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
            self._writer = cv2.VideoWriter(
                kayit_yolu,
                fourcc,
                float(VIDEO_FPS),
                (HEDEF_GENISLIK, HEDEF_YUKSEKLIK)
            )
            self.kare_say = 0
            self.baslama  = time.time()
            self._aktif   = True

    def durdur(self):
        with self._lock:
            if self._writer and self._aktif:
                self._writer.release()
                self._writer = None
            self._aktif = False

    def kare_yaz(self, frame_bgr):
        with self._lock:
            if self._aktif and self._writer:
                resized = cv2.resize(frame_bgr, (HEDEF_GENISLIK, HEDEF_YUKSEKLIK))
                self._writer.write(resized)
                self.kare_say += 1

    @property
    def aktif(self):
        return self._aktif


# ─── Ana Node ────────────────────────────────────────────────────────────────

class VeriPaketi(Node):

    def __init__(self):
        super().__init__('veri_paketi')

        if not CV_OK:
            self.get_logger().error(
                'OpenCV veya cv_bridge bulunamadı! '
                'pip install opencv-python && apt install ros-humble-cv-bridge'
            )

        self._bridge = CvBridge() if CV_OK else None

        # Kaydediciler
        self._ileri = KameraKaydedici('ileri')
        self._geri  = KameraKaydedici('geri')
        self._nisan = KameraKaydedici('nisan')
        self._kayit_dizin = None
        self._kayit_aktif = False

        # QoS: kamera topic'leri genellikle BEST_EFFORT
        qos_cam = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Kamera subscriptionları
        self._sub_ileri = self.create_subscription(
            Image, TOPIC_ILERI,
            lambda msg: self._kare_isle(msg, self._ileri),
            qos_cam
        )
        self._sub_geri = self.create_subscription(
            Image, TOPIC_GERI,
            lambda msg: self._kare_isle(msg, self._geri),
            qos_cam
        )
        self._sub_nisan = self.create_subscription(
            Image, TOPIC_NISAN,
            lambda msg: self._kare_isle(msg, self._nisan),
            qos_cam
        )

        # Kayıt başlat/durdur komutu
        self._sub_kontrol = self.create_subscription(
            Bool, KAYIT_BASLAT_TOPIC,
            self._kontrol_cb, 10
        )

        # Durum yayıncısı (1 Hz)
        self._pub_durum = self.create_publisher(Bool, KAYIT_DURUMU_TOPIC, 10)
        self.create_timer(1.0, self._durum_yayinla)

        # Misyon FSM'den otomatik başlatma — /misyon/aktif dinle
        self._sub_misyon = self.create_subscription(
            Bool, MISYON_AKTIF_TOPIC,
            self._misyon_cb, 10
        )

        self.get_logger().info('VeriPaketi node başlatıldı.')
        self.get_logger().info(f'  İleri kamera : {TOPIC_ILERI}')
        self.get_logger().info(f'  Geri kamera  : {TOPIC_GERI}')
        self.get_logger().info(f'  Nişan kamera : {TOPIC_NISAN}')
        self.get_logger().info(f'  Kayıt dizini : {KAYIT_DIZIN}')

    # ── Kare İşleme ──────────────────────────────────────────────────────────

    def _kare_isle(self, msg: Image, kaydedici: KameraKaydedici):
        if not CV_OK or not kaydedici.aktif:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            kaydedici.kare_yaz(frame)
        except Exception as e:
            self.get_logger().warn(f'{kaydedici.ad} kare dönüşüm hatası: {e}')

    # ── Kontrol Callback ─────────────────────────────────────────────────────

    def _kontrol_cb(self, msg: Bool):
        if msg.data and not self._kayit_aktif:
            self._kayit_baslat()
        elif not msg.data and self._kayit_aktif:
            self._kayit_durdur()

    def _misyon_cb(self, msg: Bool):
        """Misyon FSM'den otomatik kayıt başlatma/durdurma."""
        self._kontrol_cb(msg)

    # ── Kayıt Yönetimi ───────────────────────────────────────────────────────

    def _kayit_baslat(self):
        zaman = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        dizin = os.path.join(KAYIT_DIZIN, zaman)
        Path(dizin).mkdir(parents=True, exist_ok=True)
        self._kayit_dizin = dizin

        if CV_OK:
            self._ileri.baslat(os.path.join(dizin, 'ileri.mp4'))
            self._geri.baslat(os.path.join(dizin, 'geri.mp4'))
            self._nisan.baslat(os.path.join(dizin, 'nisan.mp4'))
            self._kayit_aktif = True
        else:
            self.get_logger().error('OpenCV yok — kayıt başlatılamadı.')
            return
        self.get_logger().info(f'KAYIT BAŞLADI → {dizin}')

    def _kayit_durdur(self):
        if not self._kayit_aktif:
            return

        if CV_OK:
            self._ileri.durdur()
            self._geri.durdur()
            self._nisan.durdur()

        self._kayit_aktif = False

        # Meta dosyası yaz
        if self._kayit_dizin:
            self._meta_yaz()
            self.get_logger().info(
                f'KAYIT DURDURULDU → {self._kayit_dizin}\n'
                f'  İleri : {self._ileri.kare_say} kare\n'
                f'  Geri  : {self._geri.kare_say} kare\n'
                f'  Nişan : {self._nisan.kare_say} kare'
            )

    def _meta_yaz(self):
        """Hakem heyetine teslim için meta veri dosyası oluşturur."""
        meta_yol = os.path.join(self._kayit_dizin, 'meta.txt')
        sure_sn = time.time() - (self._ileri.baslama or time.time())
        with open(meta_yol, 'w', encoding='utf-8') as f:
            f.write('=== LYDİA İKA — Veri Paketi Meta Verisi ===\n')
            f.write(f'Takım       : MAGNESIA\n')
            f.write(f'Başvuru ID  : 4908667\n')
            f.write(f'Kayıt dizini: {self._kayit_dizin}\n')
            f.write(f'Kayıt tarihi: {datetime.now().isoformat()}\n')
            f.write(f'Koşu süresi : {sure_sn:.1f} s\n')
            f.write(f'\n--- Kamera Bilgileri ---\n')
            f.write(f'İleri kamera topic  : {TOPIC_ILERI}\n')
            f.write(f'Geri kamera topic   : {TOPIC_GERI}\n')
            f.write(f'Nişan kamera topic  : {TOPIC_NISAN}\n')
            f.write(f'Hedef FPS           : {VIDEO_FPS}\n')
            f.write(f'Çözünürlük          : {HEDEF_GENISLIK}x{HEDEF_YUKSEKLIK}\n')
            f.write(f'\n--- Kare Sayıları ---\n')
            f.write(f'ileri.mp4   : {self._ileri.kare_say} kare\n')
            f.write(f'geri.mp4    : {self._geri.kare_say} kare\n')
            f.write(f'nisan.mp4   : {self._nisan.kare_say} kare\n')
            f.write(f'\n--- Dosyalar ---\n')
            for dosya in ['ileri.mp4', 'geri.mp4', 'nisan.mp4']:
                yol = os.path.join(self._kayit_dizin, dosya)
                boyut = os.path.getsize(yol) if os.path.exists(yol) else 0
                f.write(f'{dosya}: {boyut / 1024:.1f} KB\n')

    # ── Durum Yayını ─────────────────────────────────────────────────────────

    def _durum_yayinla(self):
        msg = Bool()
        msg.data = self._kayit_aktif
        self._pub_durum.publish(msg)

        if self._kayit_aktif:
            self.get_logger().info(
                f'Kayıt devam ediyor — '
                f'İleri:{self._ileri.kare_say} '
                f'Geri:{self._geri.kare_say} '
                f'Nişan:{self._nisan.kare_say}',
                throttle_duration_sec=5.0
            )

    # ── Temiz Kapatma ────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._kayit_aktif:
            self.get_logger().warn('Node kapanıyor — kayıt durduruluyor.')
            self._kayit_durdur()
        super().destroy_node()


# ─── Giriş Noktası ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VeriPaketi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
