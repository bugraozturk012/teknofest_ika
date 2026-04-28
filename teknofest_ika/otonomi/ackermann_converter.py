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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


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

        # ── Subscriber: /cmd_vel ──────────────────────────────────────────────
        self._sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            qos_reliable,
        )

        # ── Publisher: /ackermann_cmd ─────────────────────────────────────────
        # seri_kopru.py bu topic'e abone olacak
        self._pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            qos_reliable,
        )

        # ── Güvenlik: timeout watchdog ────────────────────────────────────────
        # Nav2 durduğunda veya bağlantı kesildiğinde aracı frenlemek için.
        # _timeout süresinde /cmd_vel gelmezse sıfır komut yayınlanır.
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(
            self._timeout / 2.0,       # timeout'un yarısı kadar sıklıkta kontrol
            self._watchdog_callback,
        )

    # ── Ana dönüşüm callback'i ────────────────────────────────────────────────
    def _cmd_vel_callback(self, twist: Twist) -> None:
        """
        Gelen Twist mesajını bisiklet modeli kinematik dönüşümüyle
        AckermannDriveStamped mesajına çevirir.
        """
        self._last_cmd_time = self.get_clock().now()

        v = twist.linear.x      # İleri hız [m/s]
        ω = twist.angular.z     # Açısal hız [rad/s]

        # ── Direksiyon açısı hesabı ───────────────────────────────────────────
        #
        # δ = arctan(L × ω / v)
        #
        # v ≈ 0 durumu (duran araç, dönme komutu):
        #   Ackermann yerinde dönemez. Direksiyon maksimuma alınır,
        #   hız sıfır bırakılır. Nav2 recovery ile yeniden plan üretir.
        #
        # v küçük ama sıfır değil (yavaş hareket):
        #   arctan kararlı çalışır, klamplama ile sınırlandırılır.

        if abs(v) < 1e-4:
            # Araç neredeyse duruyorken dönme komutu — Ackermann için imkansız
            # Direksiyon açısı korunur (son değer), hız sıfırlanır
            steering = math.copysign(self._delta_max, ω) if abs(ω) > 1e-4 else 0.0
        else:
            # Bisiklet modeli formülü
            steering = math.atan2(self._L * ω, v)

        # ── Fiziksel sınırlama (servo mekanik limiti) ─────────────────────────
        steering = max(-self._delta_max, min(self._delta_max, steering))

        # ── Hız sınırlaması (VESC akım limiti) ───────────────────────────────
        speed = max(-self._v_max, min(self._v_max, v))

        # ── Mesaj oluştur ve yayınla ──────────────────────────────────────────
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.drive.speed          = float(speed)
        msg.drive.steering_angle = float(steering)
        # steering_angle_velocity: 0.0 → servo hızını kontrolcü belirler
        msg.drive.steering_angle_velocity = 0.0
        # acceleration: 0.0 → VESC'in kendi rampa kontrolcüsüne bırakılır
        msg.drive.acceleration = 0.0

        self._pub.publish(msg)

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
            # Durdurma komutu
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_footprint'
            stop_msg.drive.speed          = 0.0
            stop_msg.drive.steering_angle = 0.0
            self._pub.publish(stop_msg)

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
