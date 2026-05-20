#!/usr/bin/env python3
"""
test_arac_koprusu.py — Kürşat'ın DC Motorlu Test Aracı Köprüsü
================================================================
Kürşat'ın firmware'i (lydia_mega_firmware) ile bizim ROS2 stack'ini
birbirine bağlar. seri_kopru.py ile AYNI topic'leri yayınlar, böylece
Nav2 / FSM / mod_yoneticisi hiç değişmez.

Farklar (seri_kopru'ya göre):
  - Baud rate   : 500 000 bps (Kürşat firmware'i)
  - Giriş topic : /mux/cmd_vel (Twist) — ackermann_converter atlanır
  - Çıkış paket : PKT_SURUCU → v1 = angular.z rad/s → centideg (Kürşat kinematiği)
  - Telemetri   : 29 byte blok [AA 55 | 14B MPU9250 | 12B BMI160 | 1B CRC]
  - Odometri    : enkoder yok → cmd_vel integrasyon (dead reckoning)
  - RC          : dummy [1500, 1500, 1800, 1000] µs → mod_yoneticisi FULL_AUTO görür
  - Taret/lazer : yok, test aracında donanım yok

Geçiş:
  Test aracı → ros2 launch teknofest_ika test_arac.launch.py
  Gerçek araç → ros2 launch teknofest_ika gercek_arac.launch.py
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import TransformBroadcaster
import serial

from teknofest_ika.otonomi.topics import (
    ODOM_TOPIC, IMU_TOPIC, RC_INPUT_TOPIC,
    E_STOP_FORCE_TOPIC, E_STOP_TOPIC,
)

# ─── Protokol Tanımları (Kürşat firmware ile eşleşmeli) ───────────────────────
_PKT_BASLA   = 0xAA
_PKT_BITIS   = 0x55
_PKT_BOYUT   = 8        # Komut paketi (Jetson → UNO)
_TELEM_BOYUT = 29       # Telemetri paketi (UNO → Jetson): 2+14+12+1

_PKT_SURUCU  = 0x01
_PKT_DUR     = 0x02
_PKT_HB      = 0x04

# ─── Test Aracı Sabitleri ─────────────────────────────────────────────────────
_WHEEL_BASE   = 0.135   # [m] Kürşat firmware'inden
_MAX_VEL_MMS  = 2000    # PKT_SURUCU v0 kırpma sınırı (mm/s)
_MAX_WZ_CD    = 32000   # PKT_SURUCU v1 kırpma sınırı (centideg)

# MPU9250 ölçek (mpu_write(GYROCFG, 0x08)=±500°/s, ACCCFG=0x00=±2g)
_GYRO_SCALE  = 65.5        # LSB / (°/s)
_ACCEL_SCALE = 16384.0     # LSB / g
_G           = 9.80665     # m/s²

# RC dummy: mod_yoneticisi ch5 > 1700 → FULL_AUTO
_RC_DUMMY = [1500.0, 1500.0, 1800.0, 1000.0]   # [ch1, ch2, ch5, ch3]


# ─── Madgwick AHRS (IMU + gyro, manyetometre yok) ────────────────────────────
class _Madgwick:
    def __init__(self, beta: float = 0.1):
        self.beta = beta
        self._q   = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self._q

        qd0 = 0.5 * (-q1*gx - q2*gy - q3*gz)
        qd1 = 0.5 * ( q0*gx + q2*gz - q3*gy)
        qd2 = 0.5 * ( q0*gy - q1*gz + q3*gx)
        qd3 = 0.5 * ( q0*gz + q1*gy - q2*gx)

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm > 0.0:
            ax, ay, az = ax/norm, ay/norm, az/norm
            _2q0, _2q1, _2q2 = 2*q0, 2*q1, 2*q2
            _4q0, _4q1, _4q2 = 4*q0, 4*q1, 4*q2
            _8q1, _8q2 = 8*q1, 8*q2
            q0q0, q1q1, q2q2, q3q3 = q0*q0, q1*q1, q2*q2, q3*q3

            s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
            s1 = (_4q1*q3q3 - 2*q3*ax + 4*q0q0*q1 - _2q0*ay
                  - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az)
            s2 = (4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - 2*q3*ay
                  - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az)
            s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay

            sn = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
            if sn > 0.0:
                s0, s1, s2, s3 = s0/sn, s1/sn, s2/sn, s3/sn
                qd0 -= self.beta * s0
                qd1 -= self.beta * s1
                qd2 -= self.beta * s2
                qd3 -= self.beta * s3

        q0 += qd0 * dt
        q1 += qd1 * dt
        q2 += qd2 * dt
        q3 += qd3 * dt
        n = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self._q = [q0/n, q1/n, q2/n, q3/n]
        return self._q

    @property
    def quaternion(self):
        return tuple(self._q)   # (w, x, y, z)

    @property
    def yaw_rad(self):
        w, x, y, z = self._q
        return math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z)


# ─── Paket yardımcıları ───────────────────────────────────────────────────────
def _komut_olustur(cmd: int, v0: int, v1: int) -> bytes:
    v0 = max(-32768, min(32767, v0))
    v1 = max(-32768, min(32767, v1))
    veri = struct.pack('>hh', v0, v1)
    crc  = cmd ^ veri[0] ^ veri[1] ^ veri[2] ^ veri[3]
    return struct.pack('BB4sBB', _PKT_BASLA, cmd, veri, crc, _PKT_BITIS)


def _telem_dogrula(buf: bytes) -> bool:
    """29 byte telemetri paketini doğrula: header + XOR checksum."""
    if len(buf) != _TELEM_BOYUT:
        return False
    if buf[0] != 0xAA or buf[1] != 0x55:
        return False
    crc = 0
    for b in buf[2:28]:
        crc ^= b
    return crc == buf[28]


def _telem_parse_mpu(buf: bytes):
    """
    MPU9250 ham verisini parse eder (byte 2-15).
    Döndürür: (ax, ay, az) m/s², (gx, gy, gz) rad/s
    """
    vals = struct.unpack_from('>7h', buf, 2)   # AX AY AZ TEMP GX GY GZ
    ax_g = vals[0] / _ACCEL_SCALE
    ay_g = vals[1] / _ACCEL_SCALE
    az_g = vals[2] / _ACCEL_SCALE
    gx_r = vals[4] / _GYRO_SCALE * math.pi / 180
    gy_r = vals[5] / _GYRO_SCALE * math.pi / 180
    gz_r = vals[6] / _GYRO_SCALE * math.pi / 180
    return ax_g, ay_g, az_g, gx_r, gy_r, gz_r


# ─── Ana Node ─────────────────────────────────────────────────────────────────
class TestAracKoprusu(Node):

    def __init__(self):
        super().__init__('test_arac_koprusu')

        self.declare_parameter('port',        '/dev/ttyACM0')
        self.declare_parameter('baud',        500000)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('sim_mode',    False)

        self._port        = self.get_parameter('port').value
        self._baud        = self.get_parameter('baud').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._sim_mode    = self.get_parameter('sim_mode').value

        # Seri port
        self._ser = None
        if not self._sim_mode:
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
                self.get_logger().info(f'Test araç portu: {self._port} @ {self._baud}')
            except serial.SerialException as e:
                self.get_logger().error(f'Port açılamadı: {e}')
                self._sim_mode = True

        if self._sim_mode:
            self.get_logger().warn('SİMÜLASYON modu — seri port yok.')

        # QoS
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # ── Abonelikler ─────────────────────────────────────────────────────
        # /cmd_vel: Nav2 doğrudan (test stack'ta mod_yoneticisi yok)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)
        self.create_subscription(Bool,  E_STOP_TOPIC,       self._estop_cb, 10)

        # ── Yayıncılar ──────────────────────────────────────────────────────
        self._odom_pub   = self.create_publisher(Odometry,         ODOM_TOPIC,        qos)
        self._imu_pub    = self.create_publisher(Imu,              IMU_TOPIC,         qos)
        self._rc_pub     = self.create_publisher(Float32MultiArray, RC_INPUT_TOPIC,    10)
        self._estop_pub  = self.create_publisher(Bool,             E_STOP_FORCE_TOPIC, 10)
        self._tf         = TransformBroadcaster(self)

        # ── Durum ────────────────────────────────────────────────────────────
        self._lock         = threading.Lock()
        self._e_stop_aktif = False
        self._son_cmd      = time.monotonic()
        self._last_hb_ms   = time.monotonic()

        # Odometri (dead reckoning)
        self._x      = 0.0
        self._y      = 0.0
        self._theta  = 0.0
        self._vx     = 0.0
        self._wz     = 0.0
        self._last_cmd_t = time.monotonic()

        # Madgwick
        self._madg      = _Madgwick(beta=0.1)
        self._last_imu_t = time.monotonic()

        # Önceden tahsis RC mesajı
        self._rc_msg      = Float32MultiArray()
        self._rc_msg.data = list(_RC_DUMMY)

        # ── Zamanlayıcılar ───────────────────────────────────────────────────
        self.create_timer(0.4,  self._hb_gonder)        # heartbeat 400ms
        self.create_timer(0.05, self._rc_yayinla)       # RC dummy 20Hz
        self.create_timer(0.05, self._odom_yayinla)     # odometri 20Hz
        self.create_timer(0.1,  self._guvenlik_kontrol) # cmd timeout 10Hz

        # Telemetri okuma thread'i
        self._calisıyor = True
        if not self._sim_mode:
            threading.Thread(target=self._okuma_dongusu, daemon=True).start()

        self.get_logger().info('TestAracKoprusu başlatıldı.')

    # ── /mux/cmd_vel callback ────────────────────────────────────────────────
    def _cmd_cb(self, msg: Twist):
        self._son_cmd = time.monotonic()

        if self._e_stop_aktif:
            self._paket_gonder(_PKT_DUR, 0, 0)
            return

        vx = msg.linear.x
        wz = msg.angular.z

        with self._lock:
            self._vx = vx
            self._wz = wz

        # Kürşat kinematiği:
        #   v0 = lineer hız mm/s
        #   v1 = centideg → firmware içinde: angular_vel = (v1/100) * π/180
        #   Dolayısıyla: v1 = wz [rad/s] * (180/π) * 100
        v0 = int(vx * 1000)
        v1 = int(math.degrees(wz) * 100)
        v0 = max(-_MAX_VEL_MMS, min(_MAX_VEL_MMS, v0))
        v1 = max(-_MAX_WZ_CD,   min(_MAX_WZ_CD,   v1))

        self._paket_gonder(_PKT_SURUCU, v0, v1)

    # ── E-STOP callback ──────────────────────────────────────────────────────
    def _estop_cb(self, msg: Bool):
        onceki = self._e_stop_aktif
        self._e_stop_aktif = msg.data
        if msg.data and not onceki:
            self._paket_gonder(_PKT_DUR, 0, 0)
            self.get_logger().error('!!! E-STOP — Motor durduruldu !!!')
        elif not msg.data and onceki:
            self.get_logger().warn('[E-STOP] Kaldırıldı.')

    # ── Telemetri okuma thread'i (29-byte blok) ───────────────────────────────
    def _okuma_dongusu(self):
        """
        Kürşat'ın UNO'sundan gelen 29 byte'lık telemetri bloğunu okur.
        Senkronizasyon: 0xAA arar, sonrasında 0x55 kontrolü yapar.
        """
        while self._calisıyor:
            try:
                b = self._ser.read(1)
                if not b or b[0] != 0xAA:
                    continue
                b2 = self._ser.read(1)
                if not b2 or b2[0] != 0x55:
                    continue
                kalan = self._ser.read(_TELEM_BOYUT - 2)
                if len(kalan) != _TELEM_BOYUT - 2:
                    continue
                buf = bytes([0xAA, 0x55]) + kalan
                if not _telem_dogrula(buf):
                    self.get_logger().warn(
                        'Telemetri CRC hatası', throttle_duration_sec=5.0)
                    continue
                self._telem_isle(buf)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                time.sleep(0.2)

    def _telem_isle(self, buf: bytes):
        now = time.monotonic()
        dt  = now - self._last_imu_t
        self._last_imu_t = now
        if dt <= 0 or dt > 1.0:
            return

        ax, ay, az, gx, gy, gz = _telem_parse_mpu(buf)
        w, x, y, z = self._madg.update(gx, gy, gz, ax, ay, az, dt)

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation_covariance[0] = 0.005
        msg.orientation_covariance[4] = 0.005
        msg.orientation_covariance[8] = 0.02

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.002
        msg.angular_velocity_covariance[4] = 0.002
        msg.angular_velocity_covariance[8] = 0.002

        msg.linear_acceleration.x = ax * _G
        msg.linear_acceleration.y = ay * _G
        msg.linear_acceleration.z = az * _G
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self._imu_pub.publish(msg)

    # ── Odometri yayını (cmd_vel dead reckoning, 20Hz) ────────────────────────
    def _odom_yayinla(self):
        now = time.monotonic()
        dt  = 0.05   # timer 50ms sabit — küçük hata kabul edilebilir

        with self._lock:
            vx = self._vx
            wz = self._wz

        dtheta = wz * dt
        dx = vx * math.cos(self._theta + dtheta / 2) * dt
        dy = vx * math.sin(self._theta + dtheta / 2) * dt

        self._x     += dx
        self._y     += dy
        self._theta  = math.remainder(self._theta + dtheta, 2 * math.pi)

        qz = math.sin(self._theta / 2)
        qw = math.cos(self._theta / 2)
        t  = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp             = t
        tf.header.frame_id          = 'odom'
        tf.child_frame_id           = 'base_footprint'
        tf.transform.translation.x  = self._x
        tf.transform.translation.y  = self._y
        tf.transform.rotation.z     = qz
        tf.transform.rotation.w     = qw
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp              = t
        odom.header.frame_id           = 'odom'
        odom.child_frame_id            = 'base_footprint'
        odom.pose.pose.position.x      = self._x
        odom.pose.pose.position.y      = self._y
        odom.pose.pose.orientation.z   = qz
        odom.pose.pose.orientation.w   = qw
        odom.twist.twist.linear.x      = vx
        odom.twist.twist.angular.z     = wz
        odom.pose.covariance[0]        = 0.05   # dead reckoning kayar → yüksek kovaryans
        odom.pose.covariance[7]        = 0.05
        odom.pose.covariance[35]       = 0.1
        odom.twist.covariance[0]       = 0.02
        odom.twist.covariance[35]      = 0.05
        self._odom_pub.publish(odom)

    # ── RC dummy yayını (20Hz) ────────────────────────────────────────────────
    def _rc_yayinla(self):
        # ch5=1800µs → mod_yoneticisi FULL_AUTO (>1700µs eşiği)
        self._rc_pub.publish(self._rc_msg)

    # ── Heartbeat ────────────────────────────────────────────────────────────
    def _hb_gonder(self):
        self._paket_gonder(_PKT_HB, 0, 0)

    # ── Komut timeout güvenliği ───────────────────────────────────────────────
    def _guvenlik_kontrol(self):
        if self._e_stop_aktif:
            self._paket_gonder(_PKT_DUR, 0, 0)
            return
        if time.monotonic() - self._son_cmd > self._cmd_timeout:
            with self._lock:
                self._vx = 0.0
                self._wz = 0.0
            self._paket_gonder(_PKT_DUR, 0, 0)

    # ── Seri paket gönderici ─────────────────────────────────────────────────
    def _paket_gonder(self, cmd: int, v0: int, v1: int):
        pkt = _komut_olustur(cmd, v0, v1)
        if self._sim_mode or self._ser is None:
            self.get_logger().debug(
                f'[SIM] 0x{cmd:02X} v0={v0} v1={v1}')
            return
        try:
            self._ser.write(pkt)
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def destroy_node(self):
        self._calisıyor = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(_komut_olustur(_PKT_DUR, 0, 0))
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


# ─── Main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TestAracKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
