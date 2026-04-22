#!/usr/bin/env python3
"""
seri_kopru.py  —  Arduino Mega ↔ Jetson Orin Nano Seri Köprü Node'u
=====================================================================

PROTOKOL (115200 baud, LF terminatör):
  Jetson → Mega : CMD:<hiz_mps>:<direksiyon_deg>\n
  Mega → Jetson : ENC:<sol_adc>:<sag_adc>\n
                  IMU:<yaw>:<pitch>:<roll>\n   (Nano'dan forward, derece)

  Örnek:
    Gönderilen : CMD:0.500:12.50\n
    Alınan     : ENC:512:515\n
                 IMU:45.12:-1.03:0.87\n

ENKODER:
  AS5600 analog çıkış → Arduino analogRead() → 10-bit (0–1023)
  TICKS_PER_REV = 1024

KİNEMATİK:
  Ackermann: δ = arctan(L × ω / v)
  Odometri : diferansiyel enkoder (sol + sağ tekerlek)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial


# ─── Araç Sabitleri ────────────────────────────────────────────────────────
DINGIL_ARASI      = 0.55    # Wheelbase [m]
TEKERLEK_ARALIGI  = 0.670   # Track width [m]
TEKERLEK_YARICI   = 0.180   # Tekerlek yarıçapı [m]
TICKS_PER_REV     = 1024    # analogRead 10-bit: 0–1023
MAX_DIREKSIYON    = 30.0    # [derece]

METRE_PER_TICK = (2.0 * math.pi * TEKERLEK_YARICI) / TICKS_PER_REV


# ─── Node ─────────────────────────────────────────────────────────────────
class SeriKopru(Node):

    def __init__(self):
        super().__init__('seri_kopru')

        # Parametreler
        self.declare_parameter('port',        '/dev/ttyUSB0')
        self.declare_parameter('baud',        115200)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('sim_mode',    False)

        self._port        = self.get_parameter('port').value
        self._baud        = self.get_parameter('baud').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._sim_mode    = self.get_parameter('sim_mode').value

        # Seri Port
        self._ser = None
        if not self._sim_mode:
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
                self.get_logger().info(f'Seri port: {self._port} @ {self._baud}')
            except serial.SerialException as e:
                self.get_logger().error(f'Port açılamadı: {e}')
                self._sim_mode = True
        else:
            self.get_logger().warn('Simülasyon modu aktif.')

        # Odometri
        self._x       = 0.0
        self._y       = 0.0
        self._theta   = 0.0
        self._prev_sol = None
        self._prev_sag = None
        self._lock     = threading.Lock()
        self._son_cmd  = self.get_clock().now()

        # QoS
        qos_cmd = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        qos_odom = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)

        # ROS
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos_cmd)
        self._pub      = self.create_publisher(Odometry, '/odom', qos_odom)
        self._imu_pub  = self.create_publisher(Imu, '/imu/data', qos_odom)
        self._tf       = TransformBroadcaster(self)

        # Okuma thread
        self._calisıyor = True
        if not self._sim_mode:
            threading.Thread(target=self._okuma_dongusu, daemon=True).start()

        self.create_timer(0.1, self._guvenlik_kontrol)
        self.get_logger().info('SeriKopru başlatıldı.')

    # ── /cmd_vel → Ackermann → Seri ────────────────────────────────────────
    def _cmd_cb(self, msg: Twist):
        self._son_cmd = self.get_clock().now()
        v = msg.linear.x
        w = msg.angular.z

        # Ackermann direksiyon açısı
        # v ≈ 0 → yerinde dönüş Ackermann'da fiziksel olarak mümkün değil
        delta_deg = 0.0
        if abs(v) > 0.01:
            delta_rad = math.atan2(DINGIL_ARASI * w, v)
            delta_deg = math.degrees(delta_rad)
            delta_deg = max(-MAX_DIREKSIYON, min(MAX_DIREKSIYON, delta_deg))

        self._seri_yaz(f'CMD:{v:.3f}:{delta_deg:.2f}\n')

    # ── Seri okuma (thread) ─────────────────────────────────────────────────
    def _okuma_dongusu(self):
        while self._calisıyor:
            try:
                satir = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if not satir:
                    continue

                now = self.get_clock().now()

                if satir.startswith('ENC:'):
                    parcalar = satir[4:].split(':')
                    if len(parcalar) == 2:
                        sol = int(parcalar[0])
                        sag = int(parcalar[1])
                        self._odometri(sol, sag)

                elif satir.startswith('IMU:'):
                    parcalar = satir[4:].split(':')
                    if len(parcalar) == 3:
                        yaw   = float(parcalar[0])
                        pitch = float(parcalar[1])
                        roll  = float(parcalar[2])
                        self._imu_yayinla(now, yaw, pitch, roll)

                elif satir.startswith('LOG:'):
                    self.get_logger().debug(f'[Arduino] {satir[4:]}')

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                time.sleep(0.2)
            except ValueError:
                self.get_logger().warn(f'Parse hatası: {satir}',
                                       throttle_duration_sec=5.0)

    # ── Odometri (10-bit analog AS5600, overflow korumalı) ──────────────────
    def _odometri(self, sol: int, sag: int):
        now = self.get_clock().now()
        with self._lock:
            if self._prev_sol is None:
                self._prev_sol, self._prev_sag = sol, sag
                return

            # 10-bit overflow: 0–1023 döngüsü
            d_sol = _delta(sol, self._prev_sol, TICKS_PER_REV)
            d_sag = _delta(sag, self._prev_sag, TICKS_PER_REV)
            self._prev_sol, self._prev_sag = sol, sag

        ds = d_sol * METRE_PER_TICK
        dd = d_sag * METRE_PER_TICK
        d_merkez = (ds + dd) / 2.0
        d_theta  = (dd - ds) / TEKERLEK_ARALIGI

        # Pose güncelle — 2. derece R-K
        self._x     += d_merkez * math.cos(self._theta + d_theta / 2.0)
        self._y     += d_merkez * math.sin(self._theta + d_theta / 2.0)
        self._theta  = _normalize(self._theta + d_theta)

        # Hız tahmini (50 Hz döngü varsayımı)
        vx  = d_merkez * 50.0
        vth = d_theta  * 50.0

        self._yayinla(now, vx, vth)

    # ── /odom + TF yayını ───────────────────────────────────────────────────
    def _yayinla(self, stamp, vx, vth):
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        t  = stamp.to_msg()

        tf = TransformStamped()
        tf.header.stamp          = t
        tf.header.frame_id       = 'odom'
        tf.child_frame_id        = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp            = t
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_link'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.angular.z   = vth
        # Diagonal kovaryans (deneysel olarak ayarla)
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.05
        self._pub.publish(odom)

    # ── /imu/data yayını — Madgwick Euler → quaternion (ZYX) ───────────────
    def _imu_yayinla(self, stamp, yaw_deg: float, pitch_deg: float, roll_deg: float):
        """
        Madgwick filtresi ZYX Euler açılarını (derece) quaternion'a çevirip
        sensor_msgs/Imu olarak yayınlar.

        Quaternion türetimi (ZYX intrinsic → XYZ extrinsic dönüşüm):
          q = Rz(ψ) · Ry(θ) · Rx(φ)
        """
        φ = math.radians(roll_deg)
        θ = math.radians(pitch_deg)
        ψ = math.radians(yaw_deg)

        cψ, sψ = math.cos(ψ / 2), math.sin(ψ / 2)
        cθ, sθ = math.cos(θ / 2), math.sin(θ / 2)
        cφ, sφ = math.cos(φ / 2), math.sin(φ / 2)

        qw = cφ * cθ * cψ + sφ * sθ * sψ
        qx = sφ * cθ * cψ - cφ * sθ * sψ
        qy = cφ * sθ * cψ + sφ * cθ * sψ
        qz = cφ * cθ * sψ - sφ * sθ * cψ

        msg = Imu()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Madgwick'in verdiği orientation güvenilir; deneysel kovaryans
        msg.orientation_covariance[0] = 0.005   # roll  σ² [rad²]
        msg.orientation_covariance[4] = 0.005   # pitch σ²
        msg.orientation_covariance[8] = 0.02    # yaw   σ² (manyetometre yok → gürültülü)

        # Nano jiroskop / ivmemetre ham değeri göndermiyor →
        # EKF'e "bu alanı kullanma" sinyali: kovaryans[0] = -1
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self._imu_pub.publish(msg)

    # ── Güvenlik: timeout → dur ──────────────────────────────────────────────
    def _guvenlik_kontrol(self):
        dt = (self.get_clock().now() - self._son_cmd).nanoseconds * 1e-9
        if dt > self._cmd_timeout:
            self._seri_yaz('CMD:0.000:0.00\n')

    def _seri_yaz(self, s: str):
        if self._sim_mode or self._ser is None:
            return
        try:
            self._ser.write(s.encode())
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def destroy_node(self):
        self._calisıyor = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b'CMD:0.000:0.00\n')
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


# ─── Yardımcılar ──────────────────────────────────────────────────────────
def _delta(yeni: int, eski: int, maks: int = 1024) -> int:
    """Analog AS5600: 0–(maks-1) döngüsünde overflow'u yakala."""
    d = yeni - eski
    yarim = maks // 2
    if d >  yarim: d -= maks
    if d < -yarim: d += maks
    return d

def _normalize(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ─── Main ─────────────────────────────────────────────────────────────────
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
