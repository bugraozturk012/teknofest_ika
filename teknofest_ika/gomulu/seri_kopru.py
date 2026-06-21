#!/usr/bin/env python3
"""
seri_kopru.py  —  MCU ↔ Jetson Binary Seri Köprü Node'u
=========================================================
v2.0 — Binary protokol (ika_iletisim.h ile uyumlu)

PROTOKOL (115200 baud, 8N1 — ASCII YOK, saf binary):
  Paket boyutu : 8 byte sabit
  Format       : [0xAA][CMD][D0][D1][D2][D3][CRC][0x55]
  CRC          : XOR(CMD ^ D0 ^ D1 ^ D2 ^ D3)
  Byte order   : Big-endian (MSB önce)

  Jetson → MCU:
    PKT_SURUCU (0x01): int16 hiz_mms [mm/s], int16 yaw_cd [1/100°]
    PKT_DUR    (0x02): dur komutu
    PKT_LAZER  (0x03): int16 0/1 (kapat/aç)
    PKT_HB     (0x04): heartbeat

  MCU → Jetson:
    PKT_ENC   (0x10): uint16 sol_enc, uint16 sag_enc (0-1023 ADC)
    PKT_IMU_YP(0x11): int16 yaw_dd [1/10°], int16 pitch_dd [1/10°]
    PKT_IMU_R (0x12): int16 roll_dd [1/10°]

ENKODER:
  AS5600 analog → MCU ADC → 10-bit (0–1023)
  TICKS_PER_REV = 1024

KİNEMATİK:
  Ackermann: δ = arctan(L × ω / v)
  Odometri : diferansiyel enkoder (sol + sağ tekerlek)
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Bool, Int16, Float32MultiArray
from tf2_ros import TransformBroadcaster
import serial

from teknofest_ika.otonomi.topics import (
    ACKERMANN_CMD_TOPIC, SHOOT_RESULT_TOPIC, TARET_PAN_TOPIC, TARET_TILT_TOPIC,
    ODOM_TOPIC, IMU_TOPIC, BATTERY_TOPIC, RC_INPUT_TOPIC,
    E_STOP_FORCE_TOPIC, E_STOP_TOPIC, SHOOT_CMD_TOPIC,
)
from teknofest_ika.otonomi.pure_logic import (
    paket_olustur, paket_dogrula, encoder_delta, batarya_yuzdesi,
)


# ─── Binary Protokol Tanımları (ika_iletisim.h ile eşleşmeli) ─────────────
PKT_BOYUT   = 8
PKT_BASLA   = 0xAA
PKT_BITIS   = 0x55

# Komutlar
PKT_SURUCU    = 0x01   # Jetson→MEGA: hız + direksiyon
PKT_DUR       = 0x02   # Jetson→MEGA: acil dur
PKT_LAZER     = 0x03   # Jetson→MEGA: lazer aç/kapa → NANO'ya iletilir
PKT_HB        = 0x04   # Jetson→MEGA: heartbeat
PKT_SERVO_PAN = 0x05   # Jetson→MEGA: pan açısı [0-180°] → NANO'ya iletilir
PKT_SERVO_TLT = 0x06   # Jetson→MEGA: tilt açısı [0-180°] → NANO'ya iletilir
PKT_ESTOP_OUT = 0x07   # Jetson→MEGA: acil durdurma bildirimi (Arduino motor/servo kes)

PKT_ENC       = 0x10   # MEGA→Jetson: enkoder sol + sağ
PKT_IMU_YP    = 0x11   # MEGA→Jetson: yaw + pitch (1/10 derece)
PKT_IMU_R     = 0x12   # MEGA→Jetson: roll (1/10 derece)
PKT_AKIM      = 0x13   # MEGA→Jetson: motor akımı [mA] + batarya [mV]
# RC kanalları — Flysky FS-i6X alıcısından MEGA'ya gelen PWM değerleri
# MEGA firmware bu kanalları okuyup binary paket olarak iletir.
# PKT_RC : ch1=throttle [µs], ch2=steering [µs]
# PKT_RC2: ch5=mode_switch [µs], ch3=aux1 [µs]
PKT_RC        = 0x20   # MEGA→Jetson: RC ch1 + ch2 (throttle + steering, µs)
PKT_RC2       = 0x21   # MEGA→Jetson: RC ch5 + ch3 (mode switch + aux, µs)
PKT_ESTOP_IN  = 0x22   # MEGA→Jetson: Arduino E-STOP butonu algıladı


# _paket_olustur / _paket_dogrula → pure_logic.py'ye taşındı (DRY + rclpy
# bağımsız test edilebilirlik). Burada sadece isim uyumluluğu için yeniden
# bağlanır; PKT_BASLA/PKT_BITIS/PKT_BOYUT sabitleri pure_logic'teki
# değerlerle birebir aynıdır (ikisi de [0xAA]...[0x55], 8 byte).
_paket_olustur  = paket_olustur
_paket_dogrula  = paket_dogrula


def _v0_oku(ham: bytes) -> int:
    """Paketten ilk int16 değeri okur (big-endian)."""
    return struct.unpack('>h', ham[2:4])[0]


def _v1_oku(ham: bytes) -> int:
    """Paketten ikinci int16 değeri okur (big-endian)."""
    return struct.unpack('>h', ham[4:6])[0]


# ─── Araç Sabitleri ────────────────────────────────────────────────────────
# ⚠️  GERÇEK ARAÇ ÖLÇÜLERİNE GÖRE GÜNCELLE — gömülü ekibiyle doğrula
#
# NOT: Kinematik sabitler (dingil arası, max direksiyon açısı) artık
#      ackermann_converter.py'de yönetilir. Burası yalnızca odometri
#      hesabı için gereken mekanik sabitleri içerir.
#
# TEKERLEK_ARALIGI : Sol-sağ tekerlek merkez mesafesi [m]
# TEKERLEK_YARICI  : Tekerlek (veya palet tahrik dişlisi) yarıçapı [m]
# TICKS_PER_REV    : AS5600 10-bit ADC → 0–1023 (1024 tick/tur)
# MAX_DIREKSIYON   : Donanım güvenlik kısıtı [derece] — servo fiziksel limiti
#                    ackermann_converter zaten kırpar; bu son savunma hattıdır.
# MAX_HIZ_MS       : Donanım güvenlik kısıtı [m/s] — VESC akım/hız limiti.
#                    ackermann_converter zaten kırpar (max_speed parametresi);
#                    bu da aynı şekilde son savunma hattıdır — üst akış (Nav2,
#                    HizlanmaState, manuel override) hatalı/aşırı bir hız
#                    gönderirse dahi binary pakete bu değerin üstü yazılamaz.
TEKERLEK_ARALIGI  = 0.670   # [m] — ölçüp güncelle
TEKERLEK_YARICI   = 0.180   # [m] — NEMA23 + dişli kutusu çıkış yarıçapı
TICKS_PER_REV     = 1024    # AS5600 10-bit (sabit, değiştirme)
MAX_DIREKSIYON    = 30.0    # [derece] — donanım güvenlik limiti
MAX_HIZ_MS        = 3.0     # [m/s] — donanım güvenlik limiti (VESC sınırı)

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
        self._x        = 0.0
        self._y        = 0.0
        self._theta    = 0.0
        self._prev_sol  = None
        self._prev_sag  = None
        self._prev_enc_time = None   # dt tabanlı hız için
        self._lock      = threading.Lock()
        self._son_cmd   = self.get_clock().now()

        # QoS
        qos_cmd = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        # /odom ve /imu/data RELIABLE yayınlanır.
        # robot_localization (EKF) BEST_EFFORT abone olur — uyumlu.
        # RELIABLE→BEST_EFFORT her zaman çalışır; ters durumda uyarı verir.
        qos_odom = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)

        # ROS
        # /ackermann_cmd: ackermann_converter'dan gelir
        # (kinematik dönüşüm orada yapılır, burada doğrudan kullanılır)
        self.create_subscription(AckermannDriveStamped, ACKERMANN_CMD_TOPIC,
                                 self._cmd_cb, qos_cmd)
        self._shoot_conf_pub = self.create_publisher(Bool, SHOOT_RESULT_TOPIC, 10)

        # Servo komutları — misyon_fsm ShootApproachState'ten gelir
        self.create_subscription(Int16, TARET_PAN_TOPIC,
                                 lambda m: self._paket_gonder(PKT_SERVO_PAN, m.data, 0), 10)
        self.create_subscription(Int16, TARET_TILT_TOPIC,
                                 lambda m: self._paket_gonder(PKT_SERVO_TLT, m.data, 0), 10)
        self._pub      = self.create_publisher(Odometry, ODOM_TOPIC, qos_odom)
        self._imu_pub  = self.create_publisher(Imu, IMU_TOPIC, qos_odom)
        self._tf       = TransformBroadcaster(self)

        # Batarya durumu yayıncısı — INA219 → PKT_AKIM → /battery/status
        # 4S LiPo: 16.8V tam, 14.0V boş (4.2V / 3.5V per hücre)
        self._battery_pub = self.create_publisher(BatteryState, BATTERY_TOPIC, 10)
        self._BATARYA_V_MAX = 16.8
        self._BATARYA_V_MIN = 14.0
        self._BATARYA_UYARI = 14.8   # 3.7V/hücre × 4 → nominal = uyarı eşiği

        # RC kanal yayıncısı — mod_yoneticisi dinler
        # Format: [ch1_throttle_us, ch2_steering_us, ch5_mode_us, ch3_aux_us]
        self._rc_pub   = self.create_publisher(Float32MultiArray, RC_INPUT_TOPIC, 10)

        # RC kanalları biriktirici (PKT_RC + PKT_RC2 ayrı gelir)
        self._rc_ch1   = 1500.0
        self._rc_ch2   = 1500.0
        self._rc_ch5   = 1500.0   # Varsayılan: MANUAL (güvenli başlangıç)
        self._rc_ch3   = 1000.0   # ch3 aux — lazer tetikleyici

        # Önceden tahsis edilmiş mesajlar — hot path'de GC baskısını azaltır
        self._rc_msg   = Float32MultiArray()
        self._rc_msg.data = [0.0, 0.0, 0.0, 0.0]

        # Lazer aktifken hareketi kilitle (şartname: atışta -10 ceza)
        self._lazer_aktif = False

        # E-STOP durumu — True iken tüm hareket komutları yoksayılır,
        # _guvenlik_kontrol her 100ms'de PKT_ESTOP_OUT + PKT_DUR gönderir
        self._e_stop_aktif = False
        # Arduino'dan PKT_ESTOP_IN gelince /e_stop/force'a yaz (e_stop_node toplar)
        self._e_stop_force_pub = self.create_publisher(Bool, E_STOP_FORCE_TOPIC, 10)
        self.create_subscription(Bool, E_STOP_TOPIC, self._e_stop_cb, 10)

        # /shoot_command → PKT_LAZER
        self.create_subscription(Bool, SHOOT_CMD_TOPIC, self._shoot_cb, 10)

        # IMU parçalı veri biriktirici
        self._imu_yaw   = 0.0
        self._imu_pitch = 0.0

        # Okuma thread
        self._calisıyor = True
        if not self._sim_mode:
            threading.Thread(target=self._okuma_dongusu, daemon=True).start()

        # Heartbeat — 400 ms'de bir PKT_HB gönder (watchdog 2s sınırı için güvenli)
        self.create_timer(0.4, self._hb_gonder)
        self.create_timer(0.1, self._guvenlik_kontrol)
        self.get_logger().info('SeriKopru v2.0 binary mod başlatıldı.')

    # ── /ackermann_cmd → Binary Paket ──────────────────────────────────────
    # Kinematik dönüşüm ackermann_converter.py tarafından yapılmıştır.
    # Bu callback yalnızca değerleri ölçekleyip binary pakete dönüştürür.
    def _cmd_cb(self, msg: AckermannDriveStamped):
        self._son_cmd = self.get_clock().now()

        # E-STOP aktifken hareket komutunu yoksay
        if self._e_stop_aktif:
            return

        # Lazer aktifken hareket komutunu yoksay (şartname: atışta -10 ceza)
        if self._lazer_aktif:
            self._paket_gonder(PKT_DUR, 0, 0)
            return

        # Ayrılık ilkesi (Şartname §6.13/§7.8): güvenlik/timeout mantığı
        # (_guvenlik_kontrol) ile sürüş komutu işleme aynı node içinde
        # olsa da, burada beklenmeyen bir istisna (örn. bozuk mesaj alanı)
        # sessizce yutulup hareketin "son bilinen" hızda takılı kalmasına
        # izin verilmez — hata anında açıkça PKT_DUR gönderilir.
        try:
            v         = msg.drive.speed           # m/s
            delta_deg = math.degrees(msg.drive.steering_angle)  # rad → derece

            # Donanım güvenlik kısıtı — ackermann_converter zaten kırpar,
            # bu son savunma hattıdır (servo mekanik limit / VESC hız limiti).
            delta_deg = max(-MAX_DIREKSIYON, min(MAX_DIREKSIYON, delta_deg))
            v         = max(-MAX_HIZ_MS, min(MAX_HIZ_MS, v))
        except Exception as exc:
            self.get_logger().error(f'_cmd_cb hata: {exc} — PKT_DUR gönderildi.')
            self._paket_gonder(PKT_DUR, 0, 0)
            return

        # Ölçekleme:
        #   hiz_mms : m/s × 1000 → mm/s  (int16: −32.768 … +32.767 m/s)
        #   yaw_cd  : derece × 100 → 1/100°  (int16: −327.68 … +327.67°)
        hiz_mms = int(v * 1000.0)
        yaw_cd  = int(delta_deg * 100.0)

        self._paket_gonder(PKT_SURUCU, hiz_mms, yaw_cd)

    def _shoot_cb(self, msg: Bool):
        self._lazer_aktif = msg.data
        self._paket_gonder(PKT_LAZER, 1 if msg.data else 0, 0)
        if msg.data:
            self.get_logger().info('Lazer AÇIK — hareket kilitlendi.')
        else:
            self.get_logger().info('Lazer KAPALI — hareket serbest.')

    def _e_stop_cb(self, msg: Bool):
        onceki = self._e_stop_aktif
        self._e_stop_aktif = msg.data
        if msg.data and not onceki:
            # İlk aktifleşmede Arduino'ya hemen bildir
            self._paket_gonder(PKT_ESTOP_OUT, 1, 0)
            self._paket_gonder(PKT_DUR, 0, 0)
            self.get_logger().error('!!! E-STOP — Tüm hareket durduruldu !!!')
        elif not msg.data and onceki:
            self._paket_gonder(PKT_ESTOP_OUT, 0, 0)
            self.get_logger().warn('[E-STOP] Kaldırıldı — hareket izni verildi.')

    # ── Seri okuma — binary state machine (thread) ──────────────────────────
    def _okuma_dongusu(self):
        """
        MCU'dan gelen 8-byte binary paketleri senkronize eder.
        Strateji: 0xAA başlangıç byte'ını bekle, sonra 7 byte daha oku.
        Bu yaklaşım kablo gürültüsüne karşı dayanıklıdır.
        """
        while self._calisıyor:
            try:
                # Başlangıç byte'ını bekle
                byte = self._ser.read(1)
                if not byte or byte[0] != PKT_BASLA:
                    continue

                # Geri kalan 7 byte'ı oku
                kalan = self._ser.read(PKT_BOYUT - 1)
                if len(kalan) != PKT_BOYUT - 1:
                    continue

                ham = bytes([PKT_BASLA]) + kalan

                if not _paket_dogrula(ham):
                    self.get_logger().warn(
                        f'CRC hatası: {ham.hex()}',
                        throttle_duration_sec=5.0
                    )
                    continue

                self._paket_isle(ham)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                time.sleep(0.2)

    # ── Gelen Paket İşleyici ────────────────────────────────────────────────
    def _paket_isle(self, ham: bytes):
        komut = ham[1]
        now   = self.get_clock().now()

        if komut == PKT_ENC:
            sol, sag = struct.unpack('>HH', ham[2:6])
            self._odometri(sol, sag)

        elif komut == PKT_IMU_YP:
            # 1/10 derece → derece
            with self._lock:
                self._imu_yaw   = _v0_oku(ham) / 10.0
                self._imu_pitch = _v1_oku(ham) / 10.0

        elif komut == PKT_IMU_R:
            roll_deg = _v0_oku(ham) / 10.0
            with self._lock:
                yaw, pitch = self._imu_yaw, self._imu_pitch
            self._imu_yayinla(now, yaw, pitch, roll_deg)

        elif komut == PKT_LAZER:
            # MEGA/NANO lazer kapandığını echo ile bildiriyorsa onay ver
            # v0=0 → lazer kapandı → atış tamamlandı
            if _v0_oku(ham) == 0:
                self._shoot_conf_pub.publish(Bool(data=True))

        elif komut == PKT_RC:
            # RC ch1=throttle, ch2=steering (µs)
            self._rc_ch1 = float(_v0_oku(ham))
            self._rc_ch2 = float(_v1_oku(ham))
            self._rc_yayinla()

        elif komut == PKT_RC2:
            # RC ch5=mode_switch (µs), ch3=aux/lazer (µs)
            self._rc_ch5 = float(_v0_oku(ham))
            self._rc_ch3 = float(_v1_oku(ham))
            self._rc_yayinla()

        elif komut == PKT_ESTOP_IN:
            # Arduino E-STOP butonunu algıladı → /e_stop/force'a yaz (e_stop_node toplar)
            aktif = (_v0_oku(ham) != 0)
            self._e_stop_force_pub.publish(Bool(data=aktif))
            if aktif:
                self.get_logger().error('!!! E-STOP (Arduino) — Buton basıldı !!!')
            else:
                self.get_logger().warn('[E-STOP] Arduino: buton bırakıldı.')

        elif komut == PKT_AKIM:
            motor_ma   = _v0_oku(ham)            # mA
            batarya_mv = _v1_oku(ham)            # mV
            voltaj     = batarya_mv / 1000.0     # V
            akim       = motor_ma   / 1000.0     # A

            yuzdesi = batarya_yuzdesi(voltaj, self._BATARYA_V_MIN, self._BATARYA_V_MAX)

            bat = BatteryState()
            bat.header.stamp    = now.to_msg()
            bat.voltage         = voltaj
            bat.current         = akim
            bat.percentage      = yuzdesi
            bat.present         = True
            bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            bat.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_GOOD
                if voltaj >= self._BATARYA_UYARI
                else BatteryState.POWER_SUPPLY_HEALTH_DEAD
            )
            self._battery_pub.publish(bat)

            self.get_logger().info(
                f'Batarya: {voltaj:.2f} V  {yuzdesi*100:.0f}%  |  Akım: {akim:.2f} A',
                throttle_duration_sec=5.0
            )
            if voltaj < self._BATARYA_UYARI:
                self.get_logger().warn(
                    f'DÜŞÜK BATARYA: {voltaj:.2f} V ({yuzdesi*100:.0f}%) !!!',
                    throttle_duration_sec=10.0
                )

    # ── RC Yayını ────────────────────────────────────────────────────────────
    def _rc_yayinla(self):
        self._rc_msg.data[0] = self._rc_ch1
        self._rc_msg.data[1] = self._rc_ch2
        self._rc_msg.data[2] = self._rc_ch5
        self._rc_msg.data[3] = self._rc_ch3
        self._rc_pub.publish(self._rc_msg)

    # ── Odometri (10-bit analog AS5600, overflow korumalı) ──────────────────
    def _odometri(self, sol: int, sag: int):
        now     = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        with self._lock:
            if self._prev_sol is None:
                self._prev_sol, self._prev_sag = sol, sag
                self._prev_enc_time = now_sec
                return

            d_sol = _delta(sol, self._prev_sol, TICKS_PER_REV)
            d_sag = _delta(sag, self._prev_sag, TICKS_PER_REV)
            self._prev_sol, self._prev_sag = sol, sag

            dt = now_sec - self._prev_enc_time
            self._prev_enc_time = now_sec

        ds = d_sol * METRE_PER_TICK
        dd = d_sag * METRE_PER_TICK
        d_merkez = (ds + dd) / 2.0
        d_theta  = (dd - ds) / TEKERLEK_ARALIGI

        self._x     += d_merkez * math.cos(self._theta + d_theta / 2.0)
        self._y     += d_merkez * math.sin(self._theta + d_theta / 2.0)
        self._theta  = _normalize(self._theta + d_theta)

        # Gerçek dt ile hız hesabı — 0 < dt < 1s dışındaki değerler atılır
        # (NTP jump, suspend/resume gibi clock anomalileri karşı koruma)
        if 0.0 < dt < 1.0:
            vx  = d_merkez / dt
            vth = d_theta  / dt
        else:
            vx, vth = 0.0, 0.0

        self._yayinla(now, vx, vth)

    # ── /odom + TF yayını ───────────────────────────────────────────────────
    def _yayinla(self, stamp, vx, vth):
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        t  = stamp.to_msg()

        tf = TransformStamped()
        tf.header.stamp          = t
        tf.header.frame_id       = 'odom'
        tf.child_frame_id        = 'base_footprint'   # ekf_params.yaml ile eşleşmeli
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp            = t
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'
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

    # ── Heartbeat ────────────────────────────────────────────────────────────
    def _hb_gonder(self):
        self._paket_gonder(PKT_HB, 0, 0)

    # ── Güvenlik: /ackermann_cmd timeout → PKT_DUR ───────────────────────────
    def _guvenlik_kontrol(self):
        # E-STOP aktifken Arduino'ya sürekli dur komutu yağdır (10 Hz)
        if self._e_stop_aktif:
            self._paket_gonder(PKT_DUR, 0, 0)
            return
        dt = (self.get_clock().now() - self._son_cmd).nanoseconds * 1e-9
        if dt > self._cmd_timeout:
            self._paket_gonder(PKT_DUR, 0, 0)

    # ── Binary Paket Gönderici ───────────────────────────────────────────────
    def _paket_gonder(self, komut: int, v0: int, v1: int):
        """
        Binary paket oluşturur ve seri porta yazar.
        sim_mode'da işlem yapmaz (log'a basar).
        """
        pkt = _paket_olustur(komut, v0, v1)
        if self._sim_mode or self._ser is None:
            self.get_logger().debug(
                f'[SIM] PKT 0x{komut:02X} v0={v0} v1={v1} → {pkt.hex()}'
            )
            return
        try:
            self._ser.write(pkt)
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def destroy_node(self):
        self._calisıyor = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(_paket_olustur(PKT_DUR, 0, 0))
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


# ─── Yardımcılar ──────────────────────────────────────────────────────────
# _delta (enkoder overflow) → pure_logic.encoder_delta (DRY + test_birim.py
# bu fonksiyonu doğrudan import edip test eder).
_delta = encoder_delta


def _normalize(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


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
