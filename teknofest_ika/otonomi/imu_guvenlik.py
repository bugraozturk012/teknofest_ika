#!/usr/bin/env python3
"""
imu_guvenlik.py — IMU Tabanlı Hız Güvenlik Kontrolcüsü
=======================================================
IMU roll/pitch değerlerini izleyerek /speed_limit topic'ine
hız sınırı yayınlar. terrain_adapter bu sınırı Nav2'ye uygular.

Kurallar:
  |roll| > WARN  → hızı doğrusal düşür (WARN..STOP arası rampa)
  |roll| > STOP  → DUR (0 m/s)
  |roll| > ESTOP → /e_stop/force — devrilme
  pitch < -PITCH_DOWN → yokuş aşağı fren modu (FRENLEME_HIZ)
  diğer          → normal hız (NORMAL_MAX_HIZ)

Rampa çıktısı sıfır ile TABAN_HIZ arasında bir değer alamaz; o aralık
aracın kalkamadığı bölge, komut verilse de hareket üretmez.

Eşikler topics.IMU_ROLL_WARN/STOP/ESTOP_THRESHOLD ile tanımlı; WARN
şartname §6.5'in zorunlu kıldığı %20 yan eğimin (11,31°) üstündedir.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Float32, Bool

from teknofest_ika.otonomi.topics import (
    IMU_TOPIC, BATTERY_TOPIC, SPEED_LIMIT_TOPIC, E_STOP_FORCE_IMU_TOPIC,
    IMU_ROLL_WARN_THRESHOLD, IMU_ROLL_STOP_THRESHOLD,
    IMU_ROLL_ESTOP_THRESHOLD, IMU_PITCH_DOWN_THRESHOLD,
    BATTERY_WARN_SOC, BATTERY_CRITICAL_SOC,
)
from teknofest_ika.otonomi.pure_logic import quat_to_roll_pitch_deg, imu_guvenlik_hiz

# Rampanın referans hızı. Aracın otonomdaki gerçek tavanı 0.90 (arazi
# profili + velocity_smoother), yani bu değer nominalden yüksek ve kısıtlama
# yazılı eşikten daha geç bindirmeye başlıyor. İkisi birbirine bağlı: bu
# sayıyı gerçek tavana çekmek rampayı aşağı kaydırır ve yan eğimde komutu
# taban hızın altına iter. Eşiklerle birlikte, ölçüm geldikten sonra
# yeniden ele alınacak.
NORMAL_MAX_HIZ    = 2.0   # [m/s]
FRENLEME_HIZ      = 0.4   # [m/s]
# Kalkış sürtünmesi tabanı — sahada ölçüldü, nav2_params.yaml'daki
# min_approach_linear_velocity ve regulated_linear_scaling_min_speed ile
# aynı değer. Rampa sıfır ile bu değer arasında komut üretemez.
TABAN_HIZ         = 0.45  # [m/s]
YAYINLAMA_HZ      = 10.0

# Batarya eşikleri (8S LiPo — %100=33.6V, %0=28.0V) — topics.py'den merkezi
BATARYA_DUSUK_YUZDE   = int(BATTERY_WARN_SOC)      # %20 → hız 1.0 m/s ile kısıtlanır
BATARYA_KRITIK_YUZDE  = int(BATTERY_CRITICAL_SOC)  # %10 → hız 0.0 m/s (dur)
BATARYA_DUSUK_HIZ     = 1.0  # [m/s]


class ImuGuvenlik(Node):

    def __init__(self):
        super().__init__('imu_guvenlik')

        # Batarya kolu varsayılan olarak KAPALI. Araçtaki firmware 0x13
        # paketinin gerilim alanını direksiyon açısı için yeniden kullanıyor
        # (bkz. web_dashboard.ARAC_FIRMWARE_0X13), dolayısıyla seri_kopru'nun
        # türettiği BatteryState.percentage bu araçta gerilim değil direksiyon
        # açısından geliyor ve V_MIN=28.0'ın çok altında kaldığı için hep 0.0
        # okunuyor. Açık bırakılırsa /speed_limit sürekli 0.0 yayınlar;
        # terrain_adapter desired_linear_vel'i 0'a kırpar ve mod_yoneticisi
        # MANUEL dahil bütün modlarda mux çıkışını sıfırlar — araç hiç
        # hareket etmez, tablo da Nav2 arızası gibi görünür.
        # Gerilim gerçekten ölçülmeye başlarsa True yapılır.
        self.declare_parameter('batarya_kontrol', False)
        self._batarya_kontrol = bool(
            self.get_parameter('batarya_kontrol').value)

        self._roll         = 0.0
        self._pitch        = 0.0
        self._batarya_yuzde = 100   # %100 varsayılan (veri gelene kadar)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, IMU_TOPIC, self._imu_cb, qos)
        if self._batarya_kontrol:
            self.create_subscription(BatteryState, BATTERY_TOPIC,
                                     self._bat_cb, 10)

        self._pub       = self.create_publisher(Float32, SPEED_LIMIT_TOPIC,  10)
        self._estop_pub = self.create_publisher(Bool, E_STOP_FORCE_IMU_TOPIC, 10)

        self.create_timer(1.0 / YAYINLAMA_HZ, self._yayinla)
        self.get_logger().info(
            f'ImuGuvenlik hazır | '
            f'roll_uyari={IMU_ROLL_WARN_THRESHOLD}° | '
            f'roll_dur={IMU_ROLL_STOP_THRESHOLD}° | '
            f'roll_estop={IMU_ROLL_ESTOP_THRESHOLD}° | '
            f'batarya_kontrol={self._batarya_kontrol}'
        )

    def _bat_cb(self, msg: BatteryState):
        self._batarya_yuzde = int(msg.percentage * 100)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._roll, self._pitch = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)

    def _yayinla(self):
        roll_abs    = abs(self._roll)
        devrilme    = roll_abs >= IMU_ROLL_ESTOP_THRESHOLD

        # ── E-STOP (tek yayın noktası: e_stop_node toplar) ─────────────────
        self._estop_pub.publish(Bool(data=devrilme))
        if devrilme:
            self.get_logger().error(
                f'[ImuGuvenlik] DEVRİLME! roll={self._roll:.1f}° → /e_stop/force True',
                throttle_duration_sec=1.0,
            )

        # ── Hız sınırı hesabı (pure_logic.imu_guvenlik_hiz — tek doğruluk
        #    kaynağı, test_birim.py bu fonksiyonu doğrudan test eder) ────────
        hiz = imu_guvenlik_hiz(
            self._roll, self._pitch, self._batarya_yuzde,
            IMU_ROLL_WARN_THRESHOLD, IMU_ROLL_STOP_THRESHOLD,
            IMU_ROLL_ESTOP_THRESHOLD, IMU_PITCH_DOWN_THRESHOLD,
            NORMAL_MAX_HIZ, FRENLEME_HIZ,
            BATARYA_DUSUK_YUZDE, BATARYA_KRITIK_YUZDE, BATARYA_DUSUK_HIZ,
            TABAN_HIZ,
        )

        if roll_abs >= IMU_ROLL_STOP_THRESHOLD and not devrilme:
            self.get_logger().warn(
                f'[ImuGuvenlik] DUR — roll={self._roll:.1f}°',
                throttle_duration_sec=1.0,
            )
        elif roll_abs >= IMU_ROLL_WARN_THRESHOLD:
            self.get_logger().warn(
                f'[ImuGuvenlik] Yan eğim — roll={self._roll:.1f}° → {hiz:.2f} m/s',
                throttle_duration_sec=2.0,
            )
        elif self._pitch <= -IMU_PITCH_DOWN_THRESHOLD:
            self.get_logger().warn(
                f'[ImuGuvenlik] Yokuş aşağı — pitch={self._pitch:.1f}° → {hiz} m/s',
                throttle_duration_sec=2.0,
            )

        if self._batarya_yuzde < BATARYA_KRITIK_YUZDE:
            self.get_logger().error(
                f'[ImuGuvenlik] KRİTİK BATARYA %{self._batarya_yuzde} → dur',
                throttle_duration_sec=5.0,
            )
        elif self._batarya_yuzde < BATARYA_DUSUK_YUZDE:
            self.get_logger().warn(
                f'[ImuGuvenlik] Düşük batarya %{self._batarya_yuzde} → max {BATARYA_DUSUK_HIZ} m/s',
                throttle_duration_sec=10.0,
            )

        self._pub.publish(Float32(data=float(hiz)))


def main(args=None):
    rclpy.init(args=args)
    node = ImuGuvenlik()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
