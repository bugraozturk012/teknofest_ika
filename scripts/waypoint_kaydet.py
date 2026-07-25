#!/usr/bin/env python3
"""
waypoint_kaydet.py — Saha Waypoint Kaydedici
=============================================
Aracın bulunduğu noktayı harita koordinatı olarak okur ve waypoints.yaml
formatında yazar. Koordinat göz kararı tıklanmaz — araç fiziksel olarak
hedefe götürülür, konumu buradan okunur.

Konum kaynağı öncelik sırası:
  1. TF map → base_footprint   (SLAM/AMCL aktifken en doğru kaynak)
  2. /odometry/filtered        (TF yoksa EKF çıkışı, odom frame'inde)

Kullanım (Jetson'da, haritalama ya da lokalizasyon çalışırken):
  Terminal 1: ros2 launch teknofest_ika gercek_harita.launch.py
  Terminal 2: python3 ~/lydia_ws/scripts/waypoint_kaydet.py

  Aracı kumandayla noktaya götür → ENTER → isim yaz.
  Bitince 'q' → config/waypoint_kayit.yaml dosyasına yazılır.

Yazılan dosya doğrudan waypoints.yaml'ın 'asamalar' bloğuna yapıştırılabilir.
"""
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import tf2_ros

from teknofest_ika.otonomi.topics import EKF_ODOM_TOPIC

HARITA_FRAME = 'map'
ARAC_FRAME   = 'base_footprint'
TF_MAKS_YAS_S = 2.0

CIKTI_DOSYASI = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'waypoint_kayit.yaml'
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion → yaw [rad]. Yer aracı 2D hareket eder, roll/pitch atılır."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointKaydedici(Node):

    def __init__(self):
        super().__init__('waypoint_kaydedici')

        self._odom = None
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, EKF_ODOM_TOPIC, self._on_odom, qos)

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def konum_al(self):
        """(x, y, yaw, kaynak) döner; konum alınamazsa None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                HARITA_FRAME, ARAC_FRAME, rclpy.time.Time()
            )
            yas = (self.get_clock().now() - rclpy.time.Time.from_msg(
                tf.header.stamp)).nanoseconds / 1e9
            if yas <= TF_MAKS_YAS_S:
                t = tf.transform.translation
                r = tf.transform.rotation
                return t.x, t.y, quaternion_to_yaw(r.x, r.y, r.z, r.w), 'map (TF)'
            self.get_logger().warn(f'TF {yas:.1f}s bayat — odometriye düşülüyor.')
        except tf2_ros.TransformException:
            pass

        if self._odom is not None:
            p = self._odom.pose.pose.position
            o = self._odom.pose.pose.orientation
            return p.x, p.y, quaternion_to_yaw(o.x, o.y, o.z, o.w), 'odom (EKF)'

        return None


def yaz(kayitlar, yol):
    with open(yol, 'w') as f:
        f.write('# waypoint_kaydet.py çıktısı — waypoints.yaml asamalar bloğuna yapıştır\n')
        f.write('asamalar:\n\n')
        for k in kayitlar:
            f.write(f"  - isim: {k['isim']}\n")
            f.write( "    waypoint:\n")
            f.write(f"      x: {k['x']:.3f}\n")
            f.write(f"      y: {k['y']:.3f}\n")
            f.write(f"      yaw: {k['yaw']:.3f}\n")
            f.write( "    pas_gecilir: false\n")
            f.write(f"    aciklama: \"{k['kaynak']} — sahada ölçüldü\"\n\n")


def main():
    rclpy.init()
    node = WaypointKaydedici()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print('\n  Konum bekleniyor...', flush=True)
    for _ in range(100):
        if node.konum_al() is not None:
            break
        time.sleep(0.1)
    else:
        print('  HATA: Ne TF ne odometri geldi. SLAM/EKF çalışıyor mu?')
        rclpy.shutdown()
        sys.exit(1)

    print('\n' + '═' * 62)
    print('  WAYPOINT KAYDEDİCİ')
    print('═' * 62)
    print('  Aracı noktaya götür → ENTER    |    bitir → q + ENTER\n')

    kayitlar = []
    while True:
        girdi = input(f'  [{len(kayitlar)}] ENTER = kaydet, q = bitir > ').strip()
        if girdi.lower() == 'q':
            break

        konum = node.konum_al()
        if konum is None:
            print('     ! Konum alınamadı — atlandı.')
            continue

        x, y, yaw, kaynak = konum
        isim = input('     İsim (boş = WP_n) > ').strip() or f'WP_{len(kayitlar) + 1}'

        kayitlar.append({'isim': isim, 'x': x, 'y': y, 'yaw': yaw, 'kaynak': kaynak})
        print(f'     ✓ {isim}: x={x:.2f}  y={y:.2f}  yaw={math.degrees(yaw):.1f}°  [{kaynak}]')

        if len(kayitlar) >= 2:
            onceki = kayitlar[-2]
            mesafe = math.hypot(x - onceki['x'], y - onceki['y'])
            print(f'       ({onceki["isim"]} noktasına uzaklık: {mesafe:.2f} m)')

    if kayitlar:
        yol = os.path.abspath(CIKTI_DOSYASI)
        yaz(kayitlar, yol)
        print(f'\n  {len(kayitlar)} waypoint yazıldı:\n  {yol}\n')
    else:
        print('\n  Hiç waypoint kaydedilmedi.\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
