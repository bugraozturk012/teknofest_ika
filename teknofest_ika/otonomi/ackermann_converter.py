#!/usr/bin/env python3
"""
ackermann_converter.py — cmd_vel → AckermannDriveStamped Dönüştürücü
LYDİA İKA Projesi | ROS2 Humble

─────────────────────────────────────────────────────────────────────────────
NEDEN GEREKLİ?
─────────────────────────────────────────────────────────────────────────────
Nav2, yerel kontrolcüsünün (RPP) ürettiği komutları geometry_msgs/Twist
formatında /cmd_vel topic'ine yayınlar. Bu format diferansiyel sürüş
modelini temel alır:

    Twist.linear.x  → ileri hız (m/s)
    Twist.angular.z → açısal hız (rad/s)

Ackermann kinematikli araçta ise seri köprü (seri_kopru.py) komutları
ackermann_msgs/AckermannDriveStamped formatında bekler:

    AckermannDrive.speed          → ileri hız (m/s)
    AckermannDrive.steering_angle → direksiyon açısı (rad)

Bu node ikisi arasındaki dönüşümü yapan köprüdür.

─────────────────────────────────────────────────────────────────────────────
MATEMATİKSEL TEMEL — BİSİKLET MODELİ KİNEMATİĞİ
─────────────────────────────────────────────────────────────────────────────
Ackermann aracı için basitleştirilmiş bisiklet modeli (single-track model)
şu kinematik ilişkiyi verir:

    v   = linear.x          [m/s]   — ileriye doğru hız
    ω   = angular.z         [rad/s] — dönüş açısal hızı

Dönüş yarıçapı:
    R = v / ω               [m]     (ω ≠ 0 koşuluyla)

Direksiyon açısı (ön tekerleğin orta noktası referansıyla):
    tan(δ) = L / R
    δ = arctan(L × ω / v)  [rad]

    Burada L = dingil arası (wheelbase) [m]

Fiziksel sınırlama:
    |δ| ≤ δ_max             — servonun mekanik limiti
    |v| ≤ v_max             — VESC akım/hız limiti

Özel durum (v ≈ 0, ω ≠ 0):
    Ackermann aracı yerinde dönemez. Nav2 bu komutu recovery
    davranışında üretebilir (spin behavior). Araç durur, direksiyon
    maksimuma alınır — Nav2 yeniden plan üretir.

─────────────────────────────────────────────────────────────────────────────
TF / TOPIC MİMARİSİ
─────────────────────────────────────────────────────────────────────────────
    Nav2 (RPP)
        │
        │ /cmd_vel (Twist)
        ▼
    [ackermann_converter]              ← bu node
        │
        │ /ackermann_cmd (AckermannDriveStamped)
        ▼
    [seri_kopru]
        │
        │ USB Seri (binary 8-byte)
        ▼
    Arduino Mega → VESC + Servo

─────────────────────────────────────────────────────────────────────────────
PARAMETRELER (ros2 param set ile çalışma zamanında değiştirilebilir)
─────────────────────────────────────────────────────────────────────────────
    wheelbase          : Dingil arası [m]     — PLACEHOLDER: araç ölçülünce güncelle
    max_steering_angle : Max direksiyon açısı [rad] — yaklaşık 30° = 0.5236 rad
    max_speed          : VESC hız sınırı [m/s]
    cmd_vel_timeout    : Bu süre içinde /cmd_vel gelmezse araç durdurulur [s]

─────────────────────────────────────────────────────────────────────────────
KURULUM:
    setup.py entry_points'e ekle:
        'ackermann_converter = teknofest_ika.ackermann_converter:main'

    colcon build --packages-select teknofest_ika --symlink-install
─────────────────────────────────────────────────────────────────────────────
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt16

from teknofest_ika.otonomi.topics import (
    MUX_CMD_VEL_TOPIC, ACKERMANN_CMD_TOPIC, E_STOP_TOPIC,
    ANTI_ROLLBACK_AKTIF_TOPIC, ANTI_ROLLBACK_CMD_TOPIC,
    FREN_KOMUT_TOPIC, FREN_IVME_ESIK_MIN, FREN_IVME_ESIK_MAX,
    FREN_TAM_DUR_ORAN, FREN_RAMP_PER_S,
)
from teknofest_ika.otonomi.pure_logic import (
    ackermann_steering, fren_hedef_hesapla, fren_yumusat,
)


class AckermannConverter(Node):
    """
    /cmd_vel (Twist) → /ackermann_cmd (AckermannDriveStamped) dönüştürücü.

    Bisiklet modeli kinematik dönüşümü kullanır:
        steering_angle = arctan(wheelbase × angular_z / linear_x)
    """

    def __init__(self):
        super().__init__('ackermann_converter')

        # ── Parametreler ──────────────────────────────────────────────────────
        # PLACEHOLDER: araç fiziken hazır olunca gerçek değerler ölçülecek.
        # Çalışma zamanında değiştirmek için:
        #   ros2 param set /ackermann_converter wheelbase 0.58
        self.declare_parameter('wheelbase', 0.55)
        self.declare_parameter('max_steering_angle', 0.5236)   # 30° = π/6
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)         # [s]

        self._L        = self.get_parameter('wheelbase').value
        self._delta_max = self.get_parameter('max_steering_angle').value
        self._v_max    = self.get_parameter('max_speed').value
        self._timeout  = self.get_parameter('cmd_vel_timeout').value

        self.get_logger().info(
            f'[AckermannConverter] Başlatıldı | '
            f'wheelbase={self._L:.3f}m | '
            f'δ_max={math.degrees(self._delta_max):.1f}° | '
            f'v_max={self._v_max:.1f}m/s'
        )

        # ── QoS ───────────────────────────────────────────────────────────────
        # /cmd_vel: Nav2 RELIABLE yayınlar → RELIABLE abone
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── E-STOP durumu ────────────────────────────────────────────────────
        # E-stop aktifken anti_rollback dahil TÜM komutlar yoksayılır.
        self._e_stop_aktif = False

        # ── Override state (anti_rollback) ───────────────────────────────────
        self._override_lock   = threading.Lock()
        self._override_active = False
        self._override_twist  = Twist()

        # ── Otomatik fren state'i ─────────────────────────────────────────────
        # Hedef hızdaki ani düşüşten fren oranı hesaplanır (bkz. pure_logic.
        # fren_hedef_hesapla) — ölçekler PLACEHOLDER, fiziksel testte kalibre
        # edilecek (topics.py: FREN_IVME_ESIK_MIN/MAX, FREN_TAM_DUR_ORAN).
        self._onceki_hiz   = 0.0
        self._onceki_zaman = None
        self._fren_orani   = 0.0   # [0-1], yumuşatılmış (rate-limited) çıkış

        # ── Subscriber: /mux/cmd_vel — mod_yoneticisi çıkışı ─────────────────
        self._sub = self.create_subscription(
            Twist,
            MUX_CMD_VEL_TOPIC,
            self._cmd_vel_callback,
            qos_reliable,
        )

        # ── Subscriber: anti_rollback override ───────────────────────────────
        from std_msgs.msg import Bool as BoolMsg
        self._override_sub = self.create_subscription(
            BoolMsg, ANTI_ROLLBACK_AKTIF_TOPIC,
            self._override_aktif_cb, 10
        )
        self._override_cmd_sub = self.create_subscription(
            Twist, ANTI_ROLLBACK_CMD_TOPIC,
            self._override_cmd_cb, qos_reliable
        )
        self.create_subscription(
            BoolMsg, E_STOP_TOPIC, self._e_stop_cb, 10
        )

        # ── Publisher: /ackermann_cmd ─────────────────────────────────────────
        self._pub = self.create_publisher(
            AckermannDriveStamped,
            ACKERMANN_CMD_TOPIC,
            qos_reliable,
        )

        # ── Publisher: /fren_komut ─────────────────────────────────────────────
        self._fren_pub = self.create_publisher(UInt16, FREN_KOMUT_TOPIC, 10)

        # Önceden tahsis edilmiş mesajlar — hot path'de GC baskısını azaltır
        self._ackermann_msg = AckermannDriveStamped()
        self._ackermann_msg.header.frame_id = 'base_footprint'
        self._stop_msg = AckermannDriveStamped()
        self._stop_msg.header.frame_id = 'base_footprint'

        # ── Güvenlik: timeout watchdog ────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(
            self._timeout / 2.0,
            self._watchdog_callback,
        )

    def _override_aktif_cb(self, msg) -> None:
        with self._override_lock:
            self._override_active = msg.data

    def _override_cmd_cb(self, twist: Twist) -> None:
        with self._override_lock:
            self._override_twist = twist

    def _e_stop_cb(self, msg) -> None:
        self._e_stop_aktif = msg.data

    # ── Ana dönüşüm callback'i ────────────────────────────────────────────────
    def _cmd_vel_callback(self, twist: Twist) -> None:
        # E-STOP: anti_rollback dahil TÜM komutları yoksay
        if self._e_stop_aktif:
            self._stop_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._stop_msg)
            self._onceki_hiz   = 0.0
            self._onceki_zaman = None
            self._fren_orani   = 0.0
            self._fren_pub.publish(UInt16(data=0))
            return

        # anti_rollback aktifse Nav2 komutunu yoksay
        with self._override_lock:
            if self._override_active:
                twist = self._override_twist

        self._last_cmd_time = self.get_clock().now()

        v = twist.linear.x      # İleri hız [m/s]
        ω = twist.angular.z     # Açısal hız [rad/s]

        # ── Direksiyon açısı hesabı — δ = arctan(L × ω / v) ───────────────────
        # v ≈ 0 durumunda (Ackermann yerinde dönemez) direksiyon ω işaretine
        # göre maksimuma alınır (Nav2 recovery/spin davranışı için bilinçli
        # bir karar — "son değer korunur" DEĞİLDİR). Klamplama (servo mekanik
        # limiti) dahil tüm mantık pure_logic.ackermann_steering()'dedir;
        # test_birim.py bu fonksiyonu doğrudan test eder.
        steering = ackermann_steering(v, ω, self._L, self._delta_max)

        # ── Hız sınırlaması (Karaşimşek/buja kontrolcü limiti) ────────────────
        speed = max(-self._v_max, min(self._v_max, v))

        # ── Otomatik fren — hedef hızdaki ani düşüşten oranı hesapla ─────────
        simdi = self.get_clock().now()
        if self._onceki_zaman is not None:
            dt = (simdi - self._onceki_zaman).nanoseconds / 1e9
            hedef_oran = fren_hedef_hesapla(
                self._onceki_hiz, speed, dt,
                FREN_IVME_ESIK_MIN, FREN_IVME_ESIK_MAX, FREN_TAM_DUR_ORAN,
            )
            self._fren_orani = fren_yumusat(
                self._fren_orani, hedef_oran, FREN_RAMP_PER_S / 1000.0, dt,
            )
        self._onceki_hiz   = speed
        self._onceki_zaman = simdi
        self._fren_pub.publish(UInt16(data=int(self._fren_orani * 1000)))

        # ── Mesaj güncelle ve yayınla ─────────────────────────────────────────
        self._ackermann_msg.header.stamp     = self.get_clock().now().to_msg()
        self._ackermann_msg.drive.speed          = float(speed)
        self._ackermann_msg.drive.steering_angle = float(steering)

        self._pub.publish(self._ackermann_msg)

        self.get_logger().debug(
            f'v={v:.3f} m/s | ω={ω:.3f} rad/s → '
            f'speed={speed:.3f} m/s | δ={math.degrees(steering):.2f}°'
        )

    # ── Güvenlik watchdog ─────────────────────────────────────────────────────
    def _watchdog_callback(self) -> None:
        """
        Belirli süre /cmd_vel gelmezse (Nav2 durdu, bağlantı kesildi vb.)
        araç durdurma komutu yayınlanır.

        Bu Nav2'nin built-in timeout mekanizmasına ek bir güvenlik katmanıdır.
        Şartname gereği araç komut kesilince durmalıdır.
        """
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > self._timeout:
            self._stop_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._stop_msg)
            self._onceki_hiz   = 0.0
            self._onceki_zaman = None
            self._fren_orani   = 0.0
            self._fren_pub.publish(UInt16(data=0))

            self.get_logger().warn(
                f'[WATCHDOG] /cmd_vel {elapsed:.2f}s süredir gelmiyor → araç durduruldu.',
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
