# /teknofest_ika/karar_nodu.py
# Gazebo'dan veri alır, /cmd_vel yayınlar
# Çalıştır: ros2 run teknofest_ika karar_nodu

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math

class KararNodu(Node):
    def __init__(self):
        super().__init__('karar_nodu')

        # ── Gazebo'dan gelen veriler ──────────────────
        self.create_subscription(LaserScan, '/scan',     self.scan_cb,  10)
        self.create_subscription(Imu,       '/imu/data', self.imu_cb,   10)
        self.create_subscription(Odometry,  '/odom',     self.odom_cb,  10)

        # ── Gazebo'ya giden komut ─────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 10 Hz döngü ───────────────────────────────
        self.create_timer(0.1, self.karar_ver)

        # ── Sensör değerleri ──────────────────────────
        self.on_mesafe   = 999.0   # lidar önde kaç metre boş
        self.sol_mesafe  = 999.0   # sol bariyer mesafesi
        self.sag_mesafe  = 999.0   # sağ bariyer mesafesi
        self.egim_roll   = 0.0     # yan eğim açısı
        self.egim_pitch  = 0.0     # öne/arkaya eğim
        self.arac_x      = 0.0     # konumu
        self.arac_y      = 0.0

        # ── PID ───────────────────────────────────────
        self.pid = PID(kp=0.8, ki=0.0, kd=0.1, limit=1.0)

        # ── Durum ─────────────────────────────────────
        self.durum = 'duz_sur'

        self.get_logger().info('✅ Karar nodu başladı!')

    # ─────────────────────────────────────────────────
    # CALLBACK'LER — Gazebo her veri gönderdiğinde çalışır
    # ─────────────────────────────────────────────────

    def scan_cb(self, msg: LaserScan):
        """360° lidar verisini işle."""
        n = len(msg.ranges)
        def guveli(i):
            v = msg.ranges[i % n]
            return v if (msg.range_min < v < msg.range_max) else 999.0

        # Ön: ortadaki 30 ray'in minimumu
        on_rayler = [guveli(i) for i in range(-15, 15)]
        self.on_mesafe = min(on_rayler)

        # Sol bariyer: 60-120° arası
        sol_rayler = [guveli(i) for i in range(60, 120)]
        self.sol_mesafe = min(sol_rayler)

        # Sağ bariyer: 240-300° arası
        sag_rayler = [guveli(i) for i in range(240, 300)]
        self.sag_mesafe = min(sag_rayler)

    def imu_cb(self, msg: Imu):
        """IMU'dan euler açıları hesapla."""
        q = msg.orientation
        # Quaternion → Euler
        sinr = 2 * (q.w * q.x + q.y * q.z)
        cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.egim_roll = math.atan2(sinr, cosr)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.egim_pitch = math.asin(max(-1, min(1, sinp)))

    def odom_cb(self, msg: Odometry):
        """Konum bilgisini al."""
        self.arac_x = msg.pose.pose.position.x
        self.arac_y = msg.pose.pose.position.y

    # ─────────────────────────────────────────────────
    # ANA KARAR DÖNGÜSÜ
    # ─────────────────────────────────────────────────

    def karar_ver(self):
        komut = Twist()

        # Güvenlik: önde çok yakın engel
        if self.on_mesafe < 0.4:
            self.get_logger().warn(
                f'⚠️  ENGEL! Ön mesafe: {self.on_mesafe:.2f}m — duruyorum')
            self.cmd_pub.publish(Twist())  # dur
            return

        # Bariyer ortalama — sağdan sola fark
        # Pozitif → sola sap, negatif → sağa sap
        bariyer_hatasi = self.sag_mesafe - self.sol_mesafe
        angular_z = self.pid.hesapla(bariyer_hatasi)

        if self.durum == 'duz_sur':
            komut.linear.x  = 0.5
            komut.angular.z = angular_z

        elif self.durum == 'yavash':
            komut.linear.x  = 0.2
            komut.angular.z = angular_z

        elif self.durum == 'dur':
            komut.linear.x  = 0.0
            komut.angular.z = 0.0

        self.get_logger().info(
            f'[{self.durum}] ileri={komut.linear.x:.1f} '
            f'dönüş={komut.angular.z:.2f} | '
            f'ön={self.on_mesafe:.1f}m '
            f'sol={self.sol_mesafe:.1f}m '
            f'sağ={self.sag_mesafe:.1f}m',
            throttle_duration_sec=0.5  # 2 saniyede bir yazdır
        )

        self.cmd_pub.publish(komut)


# ─────────────────────────────────────────────────────
# PID KONTROLCÜ
# ─────────────────────────────────────────────────────

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.onceki_hata = 0.0

    def hesapla(self, hata, dt=0.1):
        self.integral += hata * dt
        # Integral sınırla (windup önleme)
        self.integral = max(-2.0, min(2.0, self.integral))
        turev = (hata - self.onceki_hata) / dt
        self.onceki_hata = hata
        cikti = self.kp*hata + self.ki*self.integral + self.kd*turev
        return max(-self.limit, min(self.limit, cikti))


def main(args=None):
    rclpy.init(args=args)
    node = KararNodu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # çıkarken dur
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()