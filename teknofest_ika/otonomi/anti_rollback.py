#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
anti_rollback.py — Geri Kayma Önleme Kontrolcüsü

Dik eğimde aracın geri kaymasını tespit eder ve önler.
Tespit: IMU pitch > topics.IMU_PITCH_RAMP_THRESHOLD VE odom hız < -0.05 m/s

KASITLI GERİ GİTME MUAF: aynı tabloyu (burun yukarı + hız negatif) Nav2'nin
BackUp kurtarması da üretir. /mux/cmd_vel'de geri komut varken müdahale
edilmez — yoksa koruma kurtarmayı geri kayma sanıp karşı komut basar, araç
planlanamaz pozdan çıkamaz ve iki katman birbirine karşı çalışır.
Komut BAYATSA koruma açık kalır: komut bilinmiyorken varsayım "istenmeyen
kayma" olmalı.

Komut mimarisi (race condition yoktur):
  /anti_rollback/cmd   (Twist) → ackermann_converter okur, aktifken override yapar
  /anti_rollback/aktif (Bool)  → ackermann_converter bu flag'e göre karar verir
"""

import time   # süre ölçümleri time.monotonic() ile: Jetson'ın RTC'si ölü ve
             # saat düzeltmesi sıçradığında time.time() aralıkları
             # milyonlarca saniye okunur (ayrıntı: misyon_fsm.py)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from teknofest_ika.otonomi.topics import (
    IMU_TOPIC, ODOM_TOPIC, ANTI_ROLLBACK_CMD_TOPIC, ANTI_ROLLBACK_AKTIF_TOPIC,
    E_STOP_TOPIC, IMU_PITCH_RAMP_THRESHOLD, MUX_CMD_VEL_TOPIC,
    YOKUS_KALKIS_AKTIF_TOPIC,
    NAV2_CMD_BAYATLAMA_S,
)
from teknofest_ika.otonomi.pure_logic import (
    quat_to_roll_pitch_deg, rollback_mudahale_gerekli,
)

# topics.py IMU_PITCH_RAMP_THRESHOLD ile uyumlu — önceden burada bağımsız
# olarak 10.0° hardcode edilmişti, merkezi sabit güncellenirse bu dosya
# senkronize değildi (DRY ihlali).
RAMP_PITCH_THRESHOLD_DEG = IMU_PITCH_RAMP_THRESHOLD
ROLLBACK_VEL_THRESHOLD = 0.05
# Aşağı akışa giden komut bu değerden daha geriyse "geri gitmek isteniyor"
# sayılır. ackermann_converter'ın okuduğu topic dinleniyor (/mux/cmd_vel),
# yani aracın gerçekten uygulayacağı komut — anti_rollback'in kendi override'ı
# bu topic'e yazılmadığı için geri besleme oluşmaz.
GERI_KOMUT_ESIGI = 0.05
RECOVERY_SPEED         = 0.3
KONTROL_HZ             = 20.0


class AntiRollback(Node):

    def __init__(self):
        super().__init__('anti_rollback')

        self._pitch    = 0.0
        self._velocity = 0.0
        self._komut_vx = 0.0
        self._komut_t  = 0.0   # monotonik — son /mux/cmd_vel'in geliş anı
                               # 0.0 = hiç gelmedi; aşağıdaki bayatlık
                               # kontrolü bu değeri ayrıca eliyor.
        self._aktif    = False
        self._e_stop   = False
        self._yokus    = False   # §6.10 yokuş kalkışı sürüyor mu

        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Imu,      IMU_TOPIC,    self._imu_cb,   qos_be)
        self.create_subscription(Odometry, ODOM_TOPIC,   self._odom_cb,  qos_be)
        self.create_subscription(Bool,     E_STOP_TOPIC, self._estop_cb, 10)
        self.create_subscription(Bool, YOKUS_KALKIS_AKTIF_TOPIC,
                                 self._yokus_cb, 10)
        self.create_subscription(Twist,    MUX_CMD_VEL_TOPIC, self._komut_cb, qos_rel)

        # Twist komutu → ackermann_converter override eder (tek yazıcı garantisi)
        self._cmd_pub   = self.create_publisher(Twist, ANTI_ROLLBACK_CMD_TOPIC,   qos_rel)
        self._durum_pub = self.create_publisher(Bool,  ANTI_ROLLBACK_AKTIF_TOPIC, 10)

        self.create_timer(1.0 / KONTROL_HZ, self._kontrol)
        self.get_logger().info(
            f'AntiRollback hazır | '
            f'pitch_esik={RAMP_PITCH_THRESHOLD_DEG:.0f}° | '
            f'vel_esik={ROLLBACK_VEL_THRESHOLD} m/s | '
            f'recovery={RECOVERY_SPEED} m/s'
        )

    def _estop_cb(self, msg: Bool):
        self._e_stop = msg.data

    def _yokus_cb(self, msg: Bool):
        self._yokus = msg.data

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        _, self._pitch = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)

    def _odom_cb(self, msg: Odometry):
        self._velocity = msg.twist.twist.linear.x

    def _komut_cb(self, msg: Twist):
        self._komut_vx = msg.linear.x
        self._komut_t  = time.monotonic()

    def _kontrol(self):
        if self._e_stop:
            return

        # §6.10 yokuş kalkışı sürerken çekilme: RampaState aracı eğimde
        # BİLEREK frenle tutuyor ve fren basılıyken gaz veriyor. Burada
        # müdahale edilirse iki katman aynı anda hız komutu basar ve zorunlu
        # duruş bozulur — hakemin gördüğü şey "tam durdu" olmaktan çıkar.
        # Gerçek kayma yine korumasız kalmıyor: RampaState'in kendi
        # rollback_riskli sayacı 1,5 s sürerse aşamayı iptal ediyor.
        if self._yokus:
            if self._aktif:
                self._aktif = False
                self._durum_pub.publish(Bool(data=False))
            return

        # pure_logic.rollback_mudahale_gerekli — test_birim.py bu fonksiyonu
        # doğrudan test eder (önceden bu mantık testte ayrı yazılmıştı).
        komut_bayat = (self._komut_t == 0.0 or
                       time.monotonic() - self._komut_t > NAV2_CMD_BAYATLAMA_S)
        rollback = rollback_mudahale_gerekli(
            self._pitch, self._velocity, self._komut_vx, komut_bayat,
            RAMP_PITCH_THRESHOLD_DEG, ROLLBACK_VEL_THRESHOLD, GERI_KOMUT_ESIGI,
        )

        if rollback and not self._aktif:
            self.get_logger().warn(
                f'[AntiRollback] GERİ KAYMA! '
                f'pitch={self._pitch:.1f}° '
                f'vel={self._velocity:.3f} m/s → {RECOVERY_SPEED} m/s override'
            )
        elif not rollback and self._aktif:
            self.get_logger().info('[AntiRollback] Geri kayma sona erdi.')

        self._aktif = rollback
        self._durum_pub.publish(Bool(data=self._aktif))

        if self._aktif:
            cmd = Twist()
            cmd.linear.x  = RECOVERY_SPEED
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AntiRollback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
