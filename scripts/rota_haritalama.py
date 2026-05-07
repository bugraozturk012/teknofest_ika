#!/usr/bin/env python3
"""
rota_haritalama.py — Eğim-Dayanıklı Haritalama Navigator
==========================================================
Düz zeminde: map → base_footprint TF (SLAM konumu)
Eğimde     : son bilinen harita konumu + odom deltası (EKF dead reckoning)

Düzeltilen sorunlar:
  - get_pose() veri gelmeden (0,0,0) döndürüyordu → hazırlık bayrakları eklendi
  - TF hazır kontrolü anında geçiyordu → gerçek TF başarı kontrolü eklendi
  - Dead reckoning rotasyonu map yaw yerine map→odom rotasyonu kullanıyor
  - Spin önleme: MIN_ILERI_HIZ her zaman korunur

Simülasyon notu:
  Gazebo'da araç diff drive plugin kullanıyor (Ackermann değil).
  Haritalama için sorun değil — fizik davranışı farklı olsa da
  SLAM haritayı doğru çizer.

Kullanım:
  Terminal 1: ros2 launch teknofest_ika slam_haritalama.launch.py
  Terminal 2: python3 ~/ika_ws/scripts/rota_haritalama.py
  Bitti:      ros2 run nav2_map_server map_saver_cli -f ~/ika_ws/maps/teknofest_harita

NOT: Harita kaydedildikten sonra bu script bir daha kullanılmaz.
     Yarışmada misyon_fsm.py + Nav2 navigasyonu devralır.
"""
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros

# ─── Navigasyon Parametreleri ───────────────────────────────────────────────
MAX_HIZ_DUZLEM   = 0.8    # Düz zeminde hedef hız [m/s]
MAX_HIZ_EGIM     = 1.0    # Eğimde hız — momentum için daha yüksek [m/s]
MAX_DONUS        = 0.9    # Maksimum dönüş hızı [rad/s]
MIN_ILERI_HIZ    = 0.25   # Minimum ileri hız — spin önleme [m/s]
TOLERANS_M       = 0.7    # Hedefe varış mesafesi [m]
TIMEOUT_S        = 120.0  # Waypoint başına zaman aşımı [s]
TF_BEKLEME_S     = 60.0   # Başlangıçta TF bekleme süresi [s]
TF_MAKS_YAS_S    = 0.5    # Bu kadar eski TF bayat sayılır [s]

KP_MESAFE        = 0.5    # İleri hız orantı katsayısı
KP_ACI           = 1.0    # Dönüş orantı katsayısı

# ─── Eğim Parametreleri ────────────────────────────────────────────────────
PITCH_EGIM_GIRIS = math.radians(5.0)   # Bu açı aşılınca dead reckoning'e geç
PITCH_EGIM_CIKIS = math.radians(2.0)   # Bu açının altına düşünce SLAM'a dön
EGIM_CIKIS_SURE  = 2.0                 # Çıkış için kaç saniye düz kalmalı [s]

VARSAYILAN_DOSYA = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'
)


def normalize(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class RotaHaritalama(Node):

    def __init__(self, waypoints: list):
        super().__init__('rota_haritalama')
        self.waypoints = waypoints

        # ── TF ────────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listen = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Hazırlık bayrakları ───────────────────────────────────────────
        # get_pose() gerçek veri gelmeden (0,0,0) döndürmemesi için
        self._tf_hazir   = False   # En az 1 başarılı TF alındı
        self._odom_hazir = False   # En az 1 odom mesajı alındı

        # ── EKF odometrisi (odom frame) ───────────────────────────────────
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        # ── Eğim dead reckoning durumu ────────────────────────────────────
        self._pitch         = 0.0
        self._egimde        = False
        self._duz_baslangic = 0.0

        # Anchor: eğime girerken kaydedilen son iyi konum (map frame)
        self._anc_map_x   = 0.0
        self._anc_map_y   = 0.0
        self._anc_map_yaw = 0.0

        # Anchor: eğime girerken kaydedilen odom konumu + yaw
        # map→odom rotasyonu = anc_map_yaw - anc_odom_yaw
        self._anc_odom_x   = 0.0
        self._anc_odom_y   = 0.0
        self._anc_odom_yaw = 0.0   # Dead reckoning rotasyonu için

        # ── Subscriptionlar ───────────────────────────────────────────────
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, qos_be)
        self.create_subscription(
            Imu, '/imu/data', self._imu_cb, qos_be)

        # ── cmd_vel publisher ─────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # ── IMU Callback ─────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.asin(sinp)

        abs_pitch = abs(self._pitch)
        now = time.time()

        if not self._egimde and abs_pitch > PITCH_EGIM_GIRIS:
            self._egimde = True
            # Anchor: eğime girerken tüm konum bilgisini kaydet
            self._anc_odom_x   = self._odom_x
            self._anc_odom_y   = self._odom_y
            self._anc_odom_yaw = self._odom_yaw
            # _anc_map_* → get_pose()'un son başarılı TF'ten güncellediği değer
            self._duz_baslangic = 0.0
            self.get_logger().info(
                f'[EĞİM] Dead reckoning başladı | '
                f'pitch={math.degrees(self._pitch):.1f}° | '
                f'map_anchor=({self._anc_map_x:.2f}, {self._anc_map_y:.2f}) | '
                f'odom_anchor=({self._anc_odom_x:.2f}, {self._anc_odom_y:.2f})'
            )

        elif self._egimde and abs_pitch < PITCH_EGIM_CIKIS:
            if self._duz_baslangic == 0.0:
                self._duz_baslangic = now
            elif now - self._duz_baslangic > EGIM_CIKIS_SURE:
                self._egimde = False
                self._duz_baslangic = 0.0
                self.get_logger().info('[EĞİM] SLAM konumuna geri dönüldü.')
        else:
            if self._egimde:
                self._duz_baslangic = 0.0

    # ── Odom Callback ─────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._odom_hazir = True

    # ── Konum Tahmini ─────────────────────────────────────────────────────
    def get_pose(self):
        """
        None döndürür  : Henüz yeterli veri yok (SLAM/odom hazır değil)
        (x, y, yaw)    : Başarılı konum tahmini

        Düz zeminde: map→base_footprint TF (SLAM)
        Eğimde     : son iyi harita konumu + EKF odom deltası

        Dead reckoning rotasyonu:
          map→odom rotasyon açısı = anc_map_yaw - anc_odom_yaw
          Bu açıyla odom delta'sı map frame'e döndürülür.
        """
        if not self._odom_hazir:
            return None

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if not self._egimde:
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                tf_yas = now_sec - (
                    t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)

                if tf_yas < TF_MAKS_YAS_S:
                    x = t.transform.translation.x
                    y = t.transform.translation.y
                    q = t.transform.rotation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    )
                    # Anchor güncelle — her başarılı TF'te taze tutulur
                    self._anc_map_x   = x
                    self._anc_map_y   = y
                    self._anc_map_yaw = yaw
                    self._anc_odom_x   = self._odom_x
                    self._anc_odom_y   = self._odom_y
                    self._anc_odom_yaw = self._odom_yaw
                    self._tf_hazir     = True
                    return x, y, yaw

                self.get_logger().warn(
                    f'TF bayat ({tf_yas:.2f}s) — dead reckoning',
                    throttle_duration_sec=2.0
                )

            except Exception:
                self.get_logger().warn(
                    'TF alınamadı — dead reckoning',
                    throttle_duration_sec=2.0
                )

        # Henüz hiç başarılı TF almadıysak veri yok
        if not self._tf_hazir:
            return None

        # Dead reckoning
        # map→odom rotasyonu: SLAM'ın odom frame'ini map frame'e hizalaması
        map_odom_rot = self._anc_map_yaw - self._anc_odom_yaw

        dx_odom = self._odom_x - self._anc_odom_x
        dy_odom = self._odom_y - self._anc_odom_y

        cos_r = math.cos(map_odom_rot)
        sin_r = math.sin(map_odom_rot)

        est_x   = self._anc_map_x + dx_odom * cos_r - dy_odom * sin_r
        est_y   = self._anc_map_y + dx_odom * sin_r + dy_odom * cos_r
        est_yaw = normalize(self._odom_yaw + map_odom_rot)

        return est_x, est_y, est_yaw

    # ── Araç Durdur ───────────────────────────────────────────────────────
    def dur(self):
        self._cmd_pub.publish(Twist())

    # ── Tek Waypoint'e Git ────────────────────────────────────────────────
    def git(self, isim: str, wx: float, wy: float) -> bool:
        self.get_logger().info(f'[{isim}] → ({wx:.2f}, {wy:.2f})')
        t0 = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if time.time() - t0 > TIMEOUT_S:
                self.get_logger().warn(f'TIMEOUT — {isim} atlanıyor')
                self.dur()
                return False

            pose = self.get_pose()
            if pose is None:
                # Veri henüz yok — bekle
                continue

            x, y, yaw = pose
            dx   = wx - x
            dy   = wy - y
            dist = math.hypot(dx, dy)

            if dist < TOLERANS_M:
                self.get_logger().info(f'OK  {isim} ({dist:.2f}m)')
                self.dur()
                return True

            max_hiz    = MAX_HIZ_EGIM if self._egimde else MAX_HIZ_DUZLEM
            hedef_aci  = math.atan2(dy, dx)
            aci_hatasi = normalize(hedef_aci - yaw)

            cmd = Twist()

            # İleri hız — MIN_ILERI_HIZ garantisi ile spin önleme
            ham_hiz = min(max_hiz, KP_MESAFE * dist)
            cmd.linear.x = max(MIN_ILERI_HIZ, ham_hiz)

            # Büyük açı hatasında yavaşla ama durma
            if abs(aci_hatasi) > math.radians(30):
                cmd.linear.x = MIN_ILERI_HIZ

            cmd.angular.z = max(-MAX_DONUS,
                                min(MAX_DONUS, KP_ACI * aci_hatasi))

            self._cmd_pub.publish(cmd)

        return False

    # ── Tüm Rotayı Çalıştır ──────────────────────────────────────────────
    def calistir(self):
        self.get_logger().info(
            f'SLAM + EKF hazır olana kadar bekleniyor '
            f'(max {TF_BEKLEME_S:.0f}s)...'
        )
        t0 = time.time()

        while rclpy.ok() and (time.time() - t0) < TF_BEKLEME_S:
            rclpy.spin_once(self, timeout_sec=0.5)

            # _tf_hazir: get_pose() içinde başarılı TF alınınca True oluyor
            if self._tf_hazir and self._odom_hazir:
                self.get_logger().info(
                    f'Sistem hazır ({time.time()-t0:.1f}s). '
                    f'Haritalama başlıyor.'
                )
                break
        else:
            self.get_logger().error(
                'Sistem hazır olmadı!\n'
                '  SLAM başlatıldı mı? → ros2 launch teknofest_ika slam_haritalama.launch.py\n'
                '  EKF çalışıyor mu?   → ros2 topic echo /odometry/filtered'
            )
            return

        toplam   = len(self.waypoints)
        basarili = 0
        self.get_logger().info(f'=== HARITALAMA BAŞLIYOR: {toplam} waypoint ===')

        for wp in self.waypoints:
            if self.git(wp['isim'], wp['x'], wp['y']):
                basarili += 1
            time.sleep(1.0)

        self.dur()
        self.get_logger().info(
            f'\n=== TAMAMLANDI: {basarili}/{toplam} başarılı ===\n'
            f'Haritayı kaydet:\n'
            f'  ros2 run nav2_map_server map_saver_cli '
            f'-f ~/ika_ws/maps/teknofest_harita'
        )


def main():
    dosya = os.path.abspath(
        sys.argv[1] if len(sys.argv) > 1 else VARSAYILAN_DOSYA
    )
    if not os.path.exists(dosya):
        print(f'HATA: {dosya} bulunamadı')
        sys.exit(1)

    with open(dosya, 'r', encoding='utf-8') as f:
        veri = yaml.safe_load(f)

    waypoints = [
        {
            'isim': a['isim'],
            'x':    float(a['waypoint']['x']),
            'y':    float(a['waypoint']['y']),
        }
        for a in veri['asamalar']
    ]

    rclpy.init()
    node = RotaHaritalama(waypoints)
    try:
        node.calistir()
    finally:
        node.dur()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
